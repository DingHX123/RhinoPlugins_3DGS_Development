using Rhino;
using Rhino.Geometry;
using System;
using System.IO;

namespace RhinoPlugins_Development
{
    public static class LasLoader
    {
        public static Splat[] LoadLas(string path, out string errorMessage)
        {
            errorMessage = string.Empty;
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 0x100000))
                using (BinaryReader reader = new BinaryReader(fs))
                {
                    if (fs.Length < 227)
                    {
                        errorMessage = "File is too short to be a valid LAS file.";
                        return null;
                    }

                    byte[] sig = reader.ReadBytes(4);
                    if (sig[0] != 0x4C || sig[1] != 0x41 || sig[2] != 0x53 || sig[3] != 0x46)
                    {
                        errorMessage = "Invalid LAS file signature. Must be 'LASF'.";
                        return null;
                    }

                    fs.Position = 0x18;
                    byte versionMajor = reader.ReadByte();
                    byte versionMinor = reader.ReadByte();

                    fs.Position = 0x5E;
                    ushort headerSize = reader.ReadUInt16();

                    fs.Position = 0x60;
                    uint offsetToPointData = reader.ReadUInt32();

                    fs.Position = 0x68;
                    byte pointFormatId = reader.ReadByte();
                    ushort pointRecordLength = reader.ReadUInt16();

                    fs.Position = 0x83;
                    double scaleX = reader.ReadDouble();
                    double scaleY = reader.ReadDouble();
                    double scaleZ = reader.ReadDouble();
                    double offsetX = reader.ReadDouble();
                    double offsetY = reader.ReadDouble();
                    double offsetZ = reader.ReadDouble();
                    long pointCount = reader.ReadUInt32();

                    if ((versionMajor > 1 || (versionMajor == 1 && versionMinor >= 4)) && headerSize >= 0x177 && fs.Length >= 0xFF)
                    {
                        fs.Position = 0xF7;
                        pointCount = (long)reader.ReadUInt64();
                    }

                    if (pointCount == 0)
                    {
                        errorMessage = "LAS file contains 0 points.";
                        return null;
                    }

                    int rgbOffset = -1;
                    switch (pointFormatId)
                    {
                        case 2: rgbOffset = 20; break;
                        case 3:
                        case 5: rgbOffset = 28; break;
                        case 7:
                        case 8:
                        case 10: rgbOffset = 30; break;
                    }

                    RhinoApp.WriteLine($"[LasLoader] Header parsed: LAS Version {versionMajor}.{versionMinor}, Format ID: {pointFormatId}, Record Length: {pointRecordLength} bytes, Point Count: {pointCount:N0}, RGB Offset: {rgbOffset}");

                    fs.Position = offsetToPointData;
                    Splat[] splats = new Splat[pointCount];
                    int batchSize = 20000;
                    byte[] batchBuffer = new byte[pointRecordLength * batchSize];
                    long loaded = 0;

                    while (loaded < pointCount)
                    {
                        int count = (int)Math.Min(batchSize, pointCount - loaded);
                        int byteCount = count * pointRecordLength;
                        int read = fs.Read(batchBuffer, 0, byteCount);
                        if (read < byteCount)
                        {
                            count = read / pointRecordLength;
                            if (count == 0) break;
                        }

                        for (int k = 0; k < count; k++)
                        {
                            int recOff = k * pointRecordLength;

                            int ix = BitConverter.ToInt32(batchBuffer, recOff);
                            int iy = BitConverter.ToInt32(batchBuffer, recOff + 4);
                            int iz = BitConverter.ToInt32(batchBuffer, recOff + 8);

                            double px = ix * scaleX + offsetX;
                            double py = iy * scaleY + offsetY;
                            double pz = iz * scaleZ + offsetZ;

                            byte r = 180, g = 180, b = 180;

                            if (rgbOffset > 0 && rgbOffset + 6 <= pointRecordLength)
                            {
                                ushort cr = BitConverter.ToUInt16(batchBuffer, recOff + rgbOffset);
                                ushort cg = BitConverter.ToUInt16(batchBuffer, recOff + rgbOffset + 2);
                                ushort cb = BitConverter.ToUInt16(batchBuffer, recOff + rgbOffset + 4);
                                r = cr > 255 ? (byte)(cr / 256) : (byte)cr;
                                g = cg > 255 ? (byte)(cg / 256) : (byte)cg;
                                b = cb > 255 ? (byte)(cb / 256) : (byte)cb;
                            }
                            else if (pointRecordLength >= 14)
                            {
                                ushort intensity = BitConverter.ToUInt16(batchBuffer, recOff + 12);
                                if (intensity > 0)
                                {
                                    r = g = b = intensity > 255 ? (byte)(intensity / 256) : (byte)intensity;
                                }
                            }

                            splats[loaded] = new Splat
                            {
                                X = (float)px, Y = (float)py, Z = (float)pz,
                                ScaleX = 1f, ScaleY = 1f, ScaleZ = 1f,
                                RotW = 1f, RotX = 0f, RotY = 0f, RotZ = 0f,
                                R = r, G = g, B = b, A = 255
                            };
                            loaded++;
                        }
                    }

                    if (loaded < splats.Length)
                    {
                        Array.Resize(ref splats, (int)loaded);
                        RhinoApp.WriteLine($"[LasLoader] Finished early, read {loaded:N0} points.");
                    }

                    float pcScale = SplatLoader.EstimatePointCloudScale(splats);
                    RhinoApp.WriteLine($"[LasLoader] Setting adaptive splat scale to: {pcScale:F4}");
                    for (int i = 0; i < splats.Length; i++)
                    {
                        splats[i].ScaleX = pcScale;
                        splats[i].ScaleY = pcScale;
                        splats[i].ScaleZ = pcScale;
                    }

                    SplatLoader.AutoCenterSplats(splats, out _);
                    return splats;
                }
            }
            catch (Exception ex)
            {
                errorMessage = ex.Message;
                return null;
            }
        }
    }
}