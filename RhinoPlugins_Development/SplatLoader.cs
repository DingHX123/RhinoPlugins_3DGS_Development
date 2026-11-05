using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;

namespace RhinoPlugins_Development
{
    public static class SplatLoader
    {
        private const float SH_C0 = 0.2820948f;

        public static void AutoCenterSplats(Splat[] splats, out Vector3d offsetApplied)
        {
            offsetApplied = Vector3d.Zero;
            if (splats == null || splats.Length == 0)
                return;

            double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue;
            double maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;

            for (int i = 0; i < splats.Length; i++)
            {
                if (splats[i].X < minX) minX = splats[i].X;
                if (splats[i].Y < minY) minY = splats[i].Y;
                if (splats[i].Z < minZ) minZ = splats[i].Z;
                if (splats[i].X > maxX) maxX = splats[i].X;
                if (splats[i].Y > maxY) maxY = splats[i].Y;
                if (splats[i].Z > maxZ) maxZ = splats[i].Z;
            }

            double cx = (minX + maxX) * 0.5;
            double cy = (minY + maxY) * 0.5;
            double cz = (minZ + maxZ) * 0.5;

            if (Math.Sqrt(cx * cx + cy * cy + cz * cz) > 1000.0)
            {
                offsetApplied = new Vector3d(-cx, -cy, -cz);
                for (int j = 0; j < splats.Length; j++)
                {
                    splats[j].X += (float)offsetApplied.X;
                    splats[j].Y += (float)offsetApplied.Y;
                    splats[j].Z += (float)offsetApplied.Z;
                }
                RhinoApp.WriteLine($"[SplatLoader] Points auto-centered. Centroid: ({cx:F2}, {cy:F2}, {cz:F2}). Applied translation: ({offsetApplied.X:F2}, {offsetApplied.Y:F2}, {offsetApplied.Z:F2})");
            }
        }

        public static float EstimatePointCloudScale(Splat[] splats)
        {
            if (splats == null || splats.Length == 0)
                return 0.02f;

            int sampleCount = Math.Min(100, splats.Length);
            int step = Math.Max(1, splats.Length / sampleCount);
            List<Point3d> samples = new List<Point3d>(sampleCount);
            for (int i = 0; i < splats.Length; i += step)
            {
                if (samples.Count >= sampleCount) break;
                samples.Add(new Point3d(splats[i].X, splats[i].Y, splats[i].Z));
            }

            double totalDist = 0.0;
            int count = 0;
            for (int j = 0; j < samples.Count; j++)
            {
                double minDist = double.MaxValue;
                Point3d pt = samples[j];
                for (int k = 0; k < splats.Length; k++)
                {
                    double dx = pt.X - splats[k].X;
                    double dy = pt.Y - splats[k].Y;
                    double dz = pt.Z - splats[k].Z;
                    double d2 = dx * dx + dy * dy + dz * dz;
                    if (d2 > 1e-9 && d2 < minDist)
                        minDist = d2;
                }
                if (minDist < double.MaxValue)
                {
                    totalDist += Math.Sqrt(minDist);
                    count++;
                }
            }

            if (count > 0)
            {
                float val = (float)(totalDist / count * 1.2);
                if (val < 0.0001f) val = 0.0001f;
                if (val > 100f) val = 100f;
                return val;
            }
            return 0.02f;
        }

        private static float Clamp(float val, float min, float max)
        {
            if (val < min) return min;
            if (val > max) return max;
            return val;
        }

        private static float Sigmoid(float x)
        {
            return 1f / (1f + (float)Math.Exp(-x));
        }

        private static int GetTypeSize(string type)
        {
            if (type == "char" || type == "uchar") return 1;
            if (type == "short" || type == "ushort") return 2;
            if (type == "int" || type == "uint" || type == "float") return 4;
            if (type == "double") return 8;
            return 4;
        }

        [StructLayout(LayoutKind.Explicit)]
        private struct FloatIntUnion
        {
            [FieldOffset(0)] public int IntValue;
            [FieldOffset(0)] public float FloatValue;
        }

        // BitConverter.Int32BitsToSingle is unavailable on .NET Framework 4.8
        private static float Int32BitsToSingle(int bits)
        {
            FloatIntUnion u = default(FloatIntUnion);
            u.IntValue = bits;
            return u.FloatValue;
        }

        private static float HalfToFloat(ushort halfBits)
        {
            int sign = (halfBits >> 15) & 0x1;
            int exponent = (halfBits >> 10) & 0x1F;
            int mantissa = halfBits & 0x3FF;

            if (exponent == 0)
            {
                if (mantissa == 0)
                    return sign != 0 ? -0f : 0f;
                exponent = 1;
                while ((mantissa & 0x400) == 0)
                {
                    mantissa <<= 1;
                    exponent--;
                }
                mantissa &= 0x3FF;
                exponent += 127 - 15 + 1;
            }
            else if (exponent == 31)
            {
                return mantissa == 0 ? (sign != 0 ? float.NegativeInfinity : float.PositiveInfinity) : float.NaN;
            }
            else
            {
                exponent += 127 - 15;
            }

            int floatBits = (sign << 31) | (exponent << 23) | (mantissa << 13);
            return Int32BitsToSingle(floatBits);
        }

        #region LoadPly

        /// <summary>最近一次 LoadPly 是否解析出真正的 splat 数据（含 scale/rot/f_dc/opacity），用于区分普通点云 PLY。</summary>
        public static bool LastPlyWasSplat { get; private set; }

        public static Splat[] LoadPly(string path, out string errorMessage)
        {
            errorMessage = string.Empty;
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 0x10000))
                using (BinaryReader reader = new BinaryReader(fs))
                {
                    if (ReadLine(reader) != "ply")
                    {
                        errorMessage = "Not a valid PLY file.";
                        return null;
                    }

                    int vertexCount = 0;
                    List<string> properties = new List<string>();
                    List<string> propertyTypes = new List<string>();
                    bool isBinaryLE = false;

                    string line;
                    while ((line = ReadLine(reader)) != null)
                    {
                        line = line.Trim();
                        if (line.Equals("end_header", StringComparison.OrdinalIgnoreCase))
                            break;

                        string[] tokens = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                        if (tokens.Length == 0) continue;

                        switch (tokens[0].ToLowerInvariant())
                        {
                            case "format":
                                if (tokens[1].ToLowerInvariant().StartsWith("binary_little_endian"))
                                    isBinaryLE = true;
                                else
                                {
                                    errorMessage = "Only binary_little_endian 1.0 format is supported currently.";
                                    return null;
                                }
                                break;
                            case "element":
                                if (tokens[1].ToLowerInvariant() == "vertex")
                                    vertexCount = int.Parse(tokens[2]);
                                break;
                            case "property":
                                propertyTypes.Add(tokens[1].ToLowerInvariant());
                                properties.Add(tokens[2].ToLowerInvariant());
                                break;
                        }
                    }

                    if (!isBinaryLE)
                    {
                        errorMessage = "PLY must be in binary_little_endian format.";
                        return null;
                    }
                    if (vertexCount == 0)
                    {
                        errorMessage = "Vertex count is 0.";
                        return null;
                    }

                    int idxX = properties.IndexOf("x");
                    int idxY = properties.IndexOf("y");
                    int idxZ = properties.IndexOf("z");
                    if (idxX < 0 || idxY < 0 || idxZ < 0)
                    {
                        errorMessage = "Missing required coordinate properties (x, y, z) in PLY header.";
                        return null;
                    }

                    int idxScale0 = properties.IndexOf("scale_0");
                    int idxScale1 = properties.IndexOf("scale_1");
                    int idxScale2 = properties.IndexOf("scale_2");
                    int idxRot0 = properties.IndexOf("rot_0");
                    int idxRot1 = properties.IndexOf("rot_1");
                    int idxRot2 = properties.IndexOf("rot_2");
                    int idxRot3 = properties.IndexOf("rot_3");
                    int idxFdc0 = properties.IndexOf("f_dc_0");
                    int idxFdc1 = properties.IndexOf("f_dc_1");
                    int idxFdc2 = properties.IndexOf("f_dc_2");
                    int idxOpacity = properties.IndexOf("opacity");

                    bool isSplat = idxScale0 >= 0 && idxScale1 >= 0 && idxScale2 >= 0 &&
                                   idxRot0 >= 0 && idxRot1 >= 0 && idxRot2 >= 0 && idxRot3 >= 0 &&
                                   idxFdc0 >= 0 && idxFdc1 >= 0 && idxFdc2 >= 0 && idxOpacity >= 0;
                    LastPlyWasSplat = isSplat;

                    int idxR = properties.IndexOf("red");
                    if (idxR < 0) idxR = properties.IndexOf("diffuse_red");
                    if (idxR < 0) idxR = properties.IndexOf("r");
                    int idxG = properties.IndexOf("green");
                    if (idxG < 0) idxG = properties.IndexOf("diffuse_green");
                    if (idxG < 0) idxG = properties.IndexOf("g");
                    int idxB = properties.IndexOf("blue");
                    if (idxB < 0) idxB = properties.IndexOf("diffuse_blue");
                    if (idxB < 0) idxB = properties.IndexOf("b");
                    int idxA = properties.IndexOf("alpha");
                    if (idxA < 0) idxA = properties.IndexOf("a");
                    int idxIntensity = properties.IndexOf("intensity");

                    RhinoApp.WriteLine($"PLY Header mapping: isSplat={isSplat}, x={idxX}, scale0={idxScale0}, rot0={idxRot0}, f_dc0={idxFdc0}, opacity={idxOpacity}, colors=(r={idxR}, g={idxG}, b={idxB})");

                    int stride = 0;
                    int[] sizes = new int[properties.Count];
                    for (int i = 0; i < properties.Count; i++)
                    {
                        sizes[i] = GetTypeSize(propertyTypes[i]);
                        stride += sizes[i];
                    }

                    RhinoApp.WriteLine($"Computed vertex record stride: {stride} bytes");

                    Splat[] splats = new Splat[vertexCount];
                    byte[] buffer = new byte[stride];
                    GCHandle handle = GCHandle.Alloc(buffer, GCHandleType.Pinned);
                    IntPtr basePtr = handle.AddrOfPinnedObject();

                    try
                    {
                        for (int j = 0; j < vertexCount; j++)
                        {
                            if (fs.Read(buffer, 0, stride) < stride)
                            {
                                errorMessage = $"Unexpected end of file at vertex {j}.";
                                return null;
                            }

                            float px = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxX);
                            float py = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxY);
                            float pz = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxZ);

                            float scaleX, scaleY, scaleZ;
                            float rotW, rotX, rotY, rotZ;
                            byte r, g, b, a;

                            if (isSplat)
                            {
                                scaleX = (float)Math.Exp(ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxScale0));
                                scaleY = (float)Math.Exp(ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxScale1));
                                scaleZ = (float)Math.Exp(ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxScale2));

                                rotW = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxRot0);
                                rotX = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxRot1);
                                rotY = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxRot2);
                                rotZ = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxRot3);

                                float rotNorm = rotW * rotW + rotX * rotX + rotY * rotY + rotZ * rotZ;
                                if (rotNorm > 1e-6f)
                                {
                                    float invNorm = (float)Math.Sqrt(rotNorm);
                                    rotW /= invNorm; rotX /= invNorm; rotY /= invNorm; rotZ /= invNorm;
                                }
                                else
                                {
                                    rotW = 1f; rotX = 0f; rotY = 0f; rotZ = 0f;
                                }

                                float fdc0 = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxFdc0);
                                float fdc1 = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxFdc1);
                                float fdc2 = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxFdc2);
                                float opacity = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxOpacity);

                                r = (byte)(Clamp(0.5f + SH_C0 * fdc0, 0f, 1f) * 255f);
                                g = (byte)(Clamp(0.5f + SH_C0 * fdc1, 0f, 1f) * 255f);
                                b = (byte)(Clamp(0.5f + SH_C0 * fdc2, 0f, 1f) * 255f);
                                a = (byte)(Sigmoid(opacity) * 255f);
                            }
                            else
                            {
                                scaleX = scaleY = scaleZ = 1f;
                                rotW = 1f; rotX = 0f; rotY = 0f; rotZ = 0f;

                                if (idxR >= 0 && idxG >= 0 && idxB >= 0)
                                {
                                    float rv = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxR);
                                    float gv = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxG);
                                    float bv = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxB);

                                    if (propertyTypes[idxR] == "float" || propertyTypes[idxR] == "double")
                                    {
                                        r = (byte)(Clamp(rv, 0f, 1f) * 255f);
                                        g = (byte)(Clamp(gv, 0f, 1f) * 255f);
                                        b = (byte)(Clamp(bv, 0f, 1f) * 255f);
                                    }
                                    else
                                    {
                                        r = (byte)Clamp(rv, 0f, 255f);
                                        g = (byte)Clamp(gv, 0f, 255f);
                                        b = (byte)Clamp(bv, 0f, 255f);
                                    }
                                }
                                else if (idxIntensity >= 0)
                                {
                                    float iv = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxIntensity);
                                    r = g = b = (iv > 255f) ? (byte)(iv / 256f) : (byte)iv;
                                }
                                else
                                {
                                    r = g = b = 180;
                                }

                                if (idxA >= 0)
                                {
                                    float av = ReadPropertyAsFloat(basePtr, properties, propertyTypes, sizes, idxA);
                                    a = (propertyTypes[idxA] != "float" && propertyTypes[idxA] != "double")
                                        ? (byte)Clamp(av, 0f, 255f)
                                        : (byte)(Clamp(av, 0f, 1f) * 255f);
                                }
                                else
                                {
                                    a = 255;
                                }
                            }

                            splats[j] = new Splat
                            {
                                X = px, Y = py, Z = pz,
                                ScaleX = scaleX, ScaleY = scaleY, ScaleZ = scaleZ,
                                RotW = rotW, RotX = rotX, RotY = rotY, RotZ = rotZ,
                                R = r, G = g, B = b, A = a
                            };
                        }
                    }
                    finally
                    {
                        handle.Free();
                    }

                    if (!isSplat)
                    {
                        float pcScale = EstimatePointCloudScale(splats);
                        RhinoApp.WriteLine($"[SplatLoader] Setting point cloud adaptive splat scale to: {pcScale:F4}");
                        for (int m = 0; m < splats.Length; m++)
                        {
                            splats[m].ScaleX = pcScale;
                            splats[m].ScaleY = pcScale;
                            splats[m].ScaleZ = pcScale;
                        }
                    }

                    AutoCenterSplats(splats, out _);
                    return splats;
                }
            }
            catch (Exception ex)
            {
                errorMessage = ex.Message;
                return null;
            }
        }

        private static float ReadPropertyAsFloat(IntPtr basePtr, List<string> properties, List<string> propertyTypes, int[] sizes, int targetIndex)
        {
            if (targetIndex < 0 || targetIndex >= properties.Count)
                return 0f;

            int offset = 0;
            for (int i = 0; i < targetIndex; i++)
                offset += sizes[i];

            string type = propertyTypes[targetIndex];
            IntPtr ptr = IntPtr.Add(basePtr, offset);

            switch (type)
            {
                case "char": return (sbyte)Marshal.ReadByte(ptr);
                case "uchar": return Marshal.ReadByte(ptr);
                case "short": return Marshal.ReadInt16(ptr);
                case "ushort": return (ushort)Marshal.ReadInt16(ptr);
                case "int": return Marshal.ReadInt32(ptr);
                case "uint": return Marshal.ReadInt32(ptr);
                case "float": return Marshal.PtrToStructure<float>(ptr);
                case "double": return (float)Marshal.PtrToStructure<double>(ptr);
                default: return 0f;
            }
        }

        #endregion

        #region LoadSplat

        public static Splat[] LoadSplat(string path, out string errorMessage)
        {
            errorMessage = string.Empty;
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 0x100000))
                {
                    long length = fs.Length;

                    if (length >= 12)
                    {
                        byte[] header = new byte[12];
                        fs.Position = 0;
                        if (fs.Read(header, 0, 12) >= 12)
                        {
                            byte b0 = header[0];
                            uint v1 = BitConverter.ToUInt32(header, 4);
                            uint v2 = BitConverter.ToUInt32(header, 8);
                            if (b0 < 10 && v1 != 0 && v1 < 10000 && v2 <= v1 && v2 != 0)
                            {
                                RhinoApp.WriteLine("[SplatLoader] Detected KSplat format by header analysis inside LoadSplat.");
                                fs.Position = 0;
                                return LoadKSplatInternal(fs, out errorMessage);
                            }
                        }
                    }

                    fs.Position = 0;

                    if (length == 0 || length % 32 != 0)
                    {
                        errorMessage = "File length is not a multiple of 32 bytes. Not a valid .splat file.";
                        return null;
                    }

                    int splatCount = (int)(length / 32);
                    Splat[] splats = new Splat[splatCount];
                    int batchSize = 20000;
                    byte[] buffer = new byte[32 * batchSize];
                    int index = 0;

                    while (index < splatCount)
                    {
                        int count = Math.Min(batchSize, splatCount - index);
                        int byteCount = count * 32;
                        int read = fs.Read(buffer, 0, byteCount);
                        if (read < byteCount)
                        {
                            count = read / 32;
                            if (count == 0) break;
                        }

                        for (int i = 0; i < count; i++)
                        {
                            int off = i * 32;
                            float px = BitConverter.ToSingle(buffer, off);
                            float py = BitConverter.ToSingle(buffer, off + 4);
                            float pz = BitConverter.ToSingle(buffer, off + 8);
                            float sx = BitConverter.ToSingle(buffer, off + 12);
                            float sy = BitConverter.ToSingle(buffer, off + 16);
                            float sz = BitConverter.ToSingle(buffer, off + 20);
                            byte r = buffer[off + 24];
                            byte g = buffer[off + 25];
                            byte b = buffer[off + 26];
                            byte a = buffer[off + 27];

                            float rw = (buffer[off + 28] - 128f) / 128f;
                            float rx = (buffer[off + 29] - 128f) / 128f;
                            float ry = (buffer[off + 30] - 128f) / 128f;
                            float rz = (buffer[off + 31] - 128f) / 128f;

                            float rotNorm = rw * rw + rx * rx + ry * ry + rz * rz;
                            if (rotNorm > 1e-6f)
                            {
                                float inv = (float)Math.Sqrt(rotNorm);
                                rw /= inv; rx /= inv; ry /= inv; rz /= inv;
                            }
                            else
                            {
                                rw = 1f; rx = 0f; ry = 0f; rz = 0f;
                            }

                            splats[index] = new Splat
                            {
                                X = px, Y = py, Z = pz,
                                ScaleX = sx, ScaleY = sy, ScaleZ = sz,
                                RotW = rw, RotX = rx, RotY = ry, RotZ = rz,
                                R = r, G = g, B = b, A = a
                            };
                            index++;
                        }
                    }

                    if (index < splatCount)
                        Array.Resize(ref splats, index);

                    RhinoApp.WriteLine($"[SplatLoader] Loaded {splats.Length:N0} splats from .splat file.");
                    AutoCenterSplats(splats, out _);
                    return splats;
                }
            }
            catch (Exception ex)
            {
                errorMessage = ex.Message;
                return null;
            }
        }

        #endregion

        #region LoadKSplat

        public static Splat[] LoadKSplat(string path, out string errorMessage)
        {
            errorMessage = string.Empty;
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 0x100000))
                {
                    return LoadKSplatInternal(fs, out errorMessage);
                }
            }
            catch (Exception ex)
            {
                errorMessage = ex.Message;
                return null;
            }
        }

        private static Splat[] LoadKSplatInternal(Stream fs, out string errorMessage)
        {
            errorMessage = string.Empty;
            try
            {
                byte[] headerBuffer = new byte[0x1000];
                if (fs.Read(headerBuffer, 0, 0x1000) < 0x1000)
                {
                    errorMessage = "Invalid KSplat file header.";
                    return null;
                }

                byte versionMajor = headerBuffer[0];
                byte versionMinor = headerBuffer[1];
                uint sectionCount = BitConverter.ToUInt32(headerBuffer, 4);
                uint maxSectionCount = BitConverter.ToUInt32(headerBuffer, 8);
                uint totalSplatCount = BitConverter.ToUInt32(headerBuffer, 12);
                uint maxSplatCount = BitConverter.ToUInt32(headerBuffer, 16);
                ushort compression = BitConverter.ToUInt16(headerBuffer, 20);

                RhinoApp.WriteLine($"[KSplatLoader] Header: version={versionMajor}.{versionMinor}, sections={sectionCount}, splats={maxSplatCount}/{totalSplatCount}, compression={compression}");

                if (sectionCount == 0 || totalSplatCount == 0)
                {
                    errorMessage = "Splat or section count is 0.";
                    return null;
                }

                int sectionHeaderSize = (int)(sectionCount * 0x400);
                byte[] sectionBuffer = new byte[sectionHeaderSize];
                if (fs.Read(sectionBuffer, 0, sectionHeaderSize) < sectionHeaderSize)
                {
                    errorMessage = "Failed to read section headers.";
                    return null;
                }

                SectionHeader[] sections = new SectionHeader[sectionCount];
                for (int i = 0; i < sectionCount; i++)
                {
                    int off = i * 0x400;
                    sections[i] = new SectionHeader
                    {
                        SplatCount = BitConverter.ToUInt32(sectionBuffer, off),
                        MaxSplatCount = BitConverter.ToUInt32(sectionBuffer, off + 4),
                        BucketSize = BitConverter.ToUInt32(sectionBuffer, off + 8),
                        BucketCount = BitConverter.ToUInt32(sectionBuffer, off + 12),
                        BucketBlockSize = BitConverter.ToSingle(sectionBuffer, off + 16),
                        BucketStorageSizeBytes = BitConverter.ToUInt16(sectionBuffer, off + 20),
                        CompressionScaleRange = BitConverter.ToUInt32(sectionBuffer, off + 24),
                        StorageSizeBytes = BitConverter.ToUInt32(sectionBuffer, off + 28),
                        FullBucketCount = BitConverter.ToUInt32(sectionBuffer, off + 32),
                        PartiallyFilledBucketCount = BitConverter.ToUInt32(sectionBuffer, off + 36),
                        SphericalHarmonicsDegree = BitConverter.ToUInt16(sectionBuffer, off + 40)
                    };
                }

                int posSize = compression == 0 ? 12 : 6;
                int scaleSize = compression == 0 ? 12 : 6;
                int rotSize = compression == 0 ? 16 : 8;
                int colorSize = 4;
                int shFloatSize;
                if (compression == 0) shFloatSize = 4;
                else if (compression == 1) shFloatSize = 2;
                else shFloatSize = 1;

                Splat[] splats = new Splat[totalSplatCount];
                int index = 0;

                for (int j = 0; j < sectionCount; j++)
                {
                    SectionHeader sec = sections[j];
                    int shDegree = sec.SphericalHarmonicsDegree != 0 ? (sec.SphericalHarmonicsDegree == 1 ? 9 : 24) : 0;
                    int recordSize = posSize + scaleSize + rotSize + colorSize + shFloatSize * shDegree;

                    byte[] data = new byte[sec.StorageSizeBytes];
                    if (fs.Read(data, 0, (int)sec.StorageSizeBytes) < sec.StorageSizeBytes)
                    {
                        errorMessage = $"Failed to read data block for section {j}.";
                        return null;
                    }

                    uint[] partialBucketLengths = null;
                    float[] bucketCenters = null;
                    int metadataOffset = 0;

                    if (compression >= 1)
                    {
                        partialBucketLengths = new uint[sec.PartiallyFilledBucketCount];
                        for (int m = 0; m < sec.PartiallyFilledBucketCount; m++)
                            partialBucketLengths[m] = BitConverter.ToUInt32(data, m * 4);

                        metadataOffset = (int)(sec.PartiallyFilledBucketCount * 4);
                        bucketCenters = new float[sec.BucketCount * 3];
                        for (int n = 0; n < sec.BucketCount; n++)
                        {
                            int boff = metadataOffset + n * sec.BucketStorageSizeBytes;
                            bucketCenters[n * 3] = BitConverter.ToSingle(data, boff);
                            bucketCenters[n * 3 + 1] = BitConverter.ToSingle(data, boff + 4);
                            bucketCenters[n * 3 + 2] = BitConverter.ToSingle(data, boff + 8);
                        }

                        metadataOffset = (int)(sec.PartiallyFilledBucketCount * 4 + sec.BucketCount * sec.BucketStorageSizeBytes);
                    }

                    float quantScale = (sec.BucketBlockSize * 0.5f) / sec.CompressionScaleRange;

                    for (uint k = 0; k < sec.MaxSplatCount; k++)
                    {
                        float px, py, pz, sx, sy, sz, rw, rx, ry, rz;
                        byte r, g, b, a;

                        int recOff = metadataOffset + (int)(k * recordSize);

                        if (compression == 0)
                        {
                            px = BitConverter.ToSingle(data, recOff);
                            py = BitConverter.ToSingle(data, recOff + 4);
                            pz = BitConverter.ToSingle(data, recOff + 8);
                            sx = BitConverter.ToSingle(data, recOff + 12);
                            sy = BitConverter.ToSingle(data, recOff + 16);
                            sz = BitConverter.ToSingle(data, recOff + 20);
                            rw = BitConverter.ToSingle(data, recOff + 24);
                            rx = BitConverter.ToSingle(data, recOff + 28);
                            ry = BitConverter.ToSingle(data, recOff + 32);
                            rz = BitConverter.ToSingle(data, recOff + 36);
                            r = data[recOff + 40];
                            g = data[recOff + 41];
                            b = data[recOff + 42];
                            a = data[recOff + 43];
                        }
                        else
                        {
                            ushort px16 = BitConverter.ToUInt16(data, recOff);
                            ushort py16 = BitConverter.ToUInt16(data, recOff + 2);
                            ushort pz16 = BitConverter.ToUInt16(data, recOff + 4);
                            int bucketIdx = GetBucketIndex(k, sec.FullBucketCount, sec.BucketSize, partialBucketLengths, sec.MaxSplatCount);
                            float bcx = bucketCenters[bucketIdx * 3];
                            float bcy = bucketCenters[bucketIdx * 3 + 1];
                            float bcz = bucketCenters[bucketIdx * 3 + 2];
                            px = (px16 - sec.CompressionScaleRange) * quantScale + bcx;
                            py = (py16 - sec.CompressionScaleRange) * quantScale + bcy;
                            pz = (pz16 - sec.CompressionScaleRange) * quantScale + bcz;

                            sx = HalfToFloat(BitConverter.ToUInt16(data, recOff + 6));
                            sy = HalfToFloat(BitConverter.ToUInt16(data, recOff + 8));
                            sz = HalfToFloat(BitConverter.ToUInt16(data, recOff + 10));
                            rw = HalfToFloat(BitConverter.ToUInt16(data, recOff + 12));
                            rx = HalfToFloat(BitConverter.ToUInt16(data, recOff + 14));
                            ry = HalfToFloat(BitConverter.ToUInt16(data, recOff + 16));
                            rz = HalfToFloat(BitConverter.ToUInt16(data, recOff + 18));
                            r = data[recOff + 20];
                            g = data[recOff + 21];
                            b = data[recOff + 22];
                            a = data[recOff + 23];
                        }

                        float rotNorm = rw * rw + rx * rx + ry * ry + rz * rz;
                        if (rotNorm > 1e-6f)
                        {
                            float inv = (float)Math.Sqrt(rotNorm);
                            rw /= inv; rx /= inv; ry /= inv; rz /= inv;
                        }
                        else
                        {
                            rw = 1f; rx = 0f; ry = 0f; rz = 0f;
                        }

                        if (index < splats.Length)
                        {
                            splats[index] = new Splat
                            {
                                X = px, Y = py, Z = pz,
                                ScaleX = sx, ScaleY = sy, ScaleZ = sz,
                                RotW = rw, RotX = rx, RotY = ry, RotZ = rz,
                                R = r, G = g, B = b, A = a
                            };
                            index++;
                        }
                    }
                }

                if (index < splats.Length)
                    Array.Resize(ref splats, index);

                AutoCenterSplats(splats, out _);
                return splats;
            }
            catch (Exception ex)
            {
                errorMessage = ex.Message;
                return null;
            }
        }

        private static int GetBucketIndex(uint localSplatIndex, uint fullBucketCount, uint bucketSize, uint[] partialBucketLengths, uint splatCount)
        {
            uint fullTotal = fullBucketCount * bucketSize;
            if (localSplatIndex < fullTotal)
                return (int)(localSplatIndex / bucketSize);

            uint offset = fullTotal;
            int bucketIdx = (int)fullBucketCount;
            for (int i = 0; offset < splatCount; i++)
            {
                uint len = partialBucketLengths[i];
                if (localSplatIndex >= offset && localSplatIndex < offset + len)
                    return bucketIdx;
                offset += len;
                bucketIdx++;
            }
            return bucketIdx;
        }

        #endregion

        #region 导出（Ply / Splat / KSplat）

        /// <summary>
        /// 导出前把 Rhino 坐标系（导入时绕 X 轴旋转 -90°）反变换回 3DGS 原始坐标系。
        /// </summary>
        private static Splat[] GetExportReadySplats(Splat[] splats)
        {
            Splat[] result = new Splat[splats.Length];
            float qw = 0.7071068f; // cos(-45°)
            float qx = 0.7071068f; // sin(-45°)

            System.Threading.Tasks.Parallel.For(0, splats.Length, i =>
            {
                Splat s = splats[i];
                result[i] = new Splat
                {
                    X = s.X,
                    Y = -s.Z,
                    Z = s.Y,
                    ScaleX = s.ScaleX,
                    ScaleY = s.ScaleY,
                    ScaleZ = s.ScaleZ,
                    RotW = qw * s.RotW - qx * s.RotX,
                    RotX = qw * s.RotX + qx * s.RotW,
                    RotY = qw * s.RotY - qx * s.RotZ,
                    RotZ = qw * s.RotZ + qx * s.RotY,
                    R = s.R,
                    G = s.G,
                    B = s.B,
                    A = s.A
                };
            });
            return result;
        }

        /// <summary>导出为标准 3DGS 二进制 PLY（坐标/缩放/旋转/颜色/不透明度全部反编码回原始参数空间）。</summary>
        public static bool ExportPly(string path, Splat[] splats, out string errorMessage)
        {
            errorMessage = string.Empty;
            splats = GetExportReadySplats(splats);
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None, 0x10000))
                using (BinaryWriter writer = new BinaryWriter(fs))
                {
                    StringBuilder header = new StringBuilder();
                    header.AppendLine("ply");
                    header.AppendLine("format binary_little_endian 1.0");
                    header.AppendLine($"element vertex {splats.Length}");
                    header.AppendLine("property float x");
                    header.AppendLine("property float y");
                    header.AppendLine("property float z");
                    header.AppendLine("property float scale_0");
                    header.AppendLine("property float scale_1");
                    header.AppendLine("property float scale_2");
                    header.AppendLine("property float rot_0");
                    header.AppendLine("property float rot_1");
                    header.AppendLine("property float rot_2");
                    header.AppendLine("property float rot_3");
                    header.AppendLine("property float f_dc_0");
                    header.AppendLine("property float f_dc_1");
                    header.AppendLine("property float f_dc_2");
                    header.AppendLine("property float opacity");
                    header.AppendLine("end_header");
                    writer.Write(Encoding.ASCII.GetBytes(header.ToString()));

                    for (int i = 0; i < splats.Length; i++)
                    {
                        Splat s = splats[i];
                        float logSx = (float)Math.Log(Math.Max(1e-8f, s.ScaleX));
                        float logSy = (float)Math.Log(Math.Max(1e-8f, s.ScaleY));
                        float logSz = (float)Math.Log(Math.Max(1e-8f, s.ScaleZ));
                        // 颜色反算到 SH DC 系数空间
                        float fdc0 = (s.R / 255f - 0.5f) / 0.2820948f;
                        float fdc1 = (s.G / 255f - 0.5f) / 0.2820948f;
                        float fdc2 = (s.B / 255f - 0.5f) / 0.2820948f;
                        // 不透明度反算到 logit 空间
                        float opacity = Math.Min(Math.Max(s.A / 255f, 0.0001f), 0.9999f);
                        float logitOpacity = (float)Math.Log(opacity / (1f - opacity));

                        writer.Write(s.X);
                        writer.Write(s.Y);
                        writer.Write(s.Z);
                        writer.Write(logSx);
                        writer.Write(logSy);
                        writer.Write(logSz);
                        writer.Write(s.RotW);
                        writer.Write(s.RotX);
                        writer.Write(s.RotY);
                        writer.Write(s.RotZ);
                        writer.Write(fdc0);
                        writer.Write(fdc1);
                        writer.Write(fdc2);
                        writer.Write(logitOpacity);
                    }
                }
                return true;
            }
            catch (Exception ex)
            {
                errorMessage = ex.Message;
                return false;
            }
        }

        /// <summary>导出为 antimatter15/splat 查看器格式（位置3f + 缩放3f + 颜色4b + 四元数4b，共 32 字节/点）。</summary>
        public static bool ExportSplat(string path, Splat[] splats, out string errorMessage)
        {
            errorMessage = string.Empty;
            splats = GetExportReadySplats(splats);
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None, 0x100000))
                using (BinaryWriter writer = new BinaryWriter(fs))
                {
                    for (int i = 0; i < splats.Length; i++)
                    {
                        Splat s = splats[i];
                        writer.Write(s.X);
                        writer.Write(s.Y);
                        writer.Write(s.Z);
                        writer.Write(s.ScaleX);
                        writer.Write(s.ScaleY);
                        writer.Write(s.ScaleZ);
                        writer.Write(s.R);
                        writer.Write(s.G);
                        writer.Write(s.B);
                        writer.Write(s.A);
                        // 四元数 [-1,1] 线性量化到 [0,255]
                        byte q0 = (byte)Math.Min(Math.Max((int)Math.Round(s.RotW * 128f + 128f), 0), 255);
                        byte q1 = (byte)Math.Min(Math.Max((int)Math.Round(s.RotX * 128f + 128f), 0), 255);
                        byte q2 = (byte)Math.Min(Math.Max((int)Math.Round(s.RotY * 128f + 128f), 0), 255);
                        byte q3 = (byte)Math.Min(Math.Max((int)Math.Round(s.RotZ * 128f + 128f), 0), 255);
                        writer.Write(q0);
                        writer.Write(q1);
                        writer.Write(q2);
                        writer.Write(q3);
                    }
                }
                return true;
            }
            catch (Exception ex)
            {
                errorMessage = ex.Message;
                return false;
            }
        }

        /// <summary>导出为 mkkellogg/GaussianSplats 3DGS 格式（未压缩变体，44 字节/点）。</summary>
        public static bool ExportKSplat(string path, Splat[] splats, out string errorMessage)
        {
            errorMessage = string.Empty;
            splats = GetExportReadySplats(splats);
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None, 0x100000))
                using (BinaryWriter writer = new BinaryWriter(fs))
                {
                    // 主头部（4096 字节，保留区）
                    byte[] header = new byte[0x1000];
                    header[0] = 1; // version major
                    header[1] = 0; // version minor
                    Array.Copy(BitConverter.GetBytes((uint)1), 0, header, 4, 4);  // max section count
                    Array.Copy(BitConverter.GetBytes((uint)1), 0, header, 8, 4);  // section count
                    Array.Copy(BitConverter.GetBytes((uint)splats.Length), 0, header, 12, 4); // max splat count
                    Array.Copy(BitConverter.GetBytes((uint)splats.Length), 0, header, 16, 4); // splat count
                    Array.Copy(BitConverter.GetBytes((ushort)0), 0, header, 20, 2); // reserved
                    writer.Write(header);

                    // 段头部（1024 字节，保留区）
                    byte[] sectionHeader = new byte[0x400];
                    Array.Copy(BitConverter.GetBytes((uint)splats.Length), 0, sectionHeader, 0, 4);  // splat count
                    Array.Copy(BitConverter.GetBytes((uint)splats.Length), 0, sectionHeader, 4, 4);  // max splat count
                    Array.Copy(BitConverter.GetBytes((uint)(splats.Length * 0x2c)), 0, sectionHeader, 0x1c, 4); // bucket storage size (44 字节/点，未压缩)
                    writer.Write(sectionHeader);

                    for (int i = 0; i < splats.Length; i++)
                    {
                        Splat s = splats[i];
                        writer.Write(s.X);
                        writer.Write(s.Y);
                        writer.Write(s.Z);
                        writer.Write(s.ScaleX);
                        writer.Write(s.ScaleY);
                        writer.Write(s.ScaleZ);
                        writer.Write(s.RotW);
                        writer.Write(s.RotX);
                        writer.Write(s.RotY);
                        writer.Write(s.RotZ);
                        writer.Write(s.R);
                        writer.Write(s.G);
                        writer.Write(s.B);
                        writer.Write(s.A);
                    }
                }
                return true;
            }
            catch (Exception ex)
            {
                errorMessage = ex.Message;
                return false;
            }
        }

        #endregion

        private static string ReadLine(BinaryReader br)
        {
            StringBuilder sb = new StringBuilder();
            try
            {
                while (true)
                {
                    char ch = br.ReadChar();
                    if (ch == '\n') return sb.ToString();
                    if (ch != '\r') sb.Append(ch);
                }
            }
            catch (EndOfStreamException)
            {
                return sb.Length > 0 ? sb.ToString() : null;
            }
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct SectionHeader
        {
            public uint SplatCount;
            public uint MaxSplatCount;
            public uint BucketSize;
            public uint BucketCount;
            public float BucketBlockSize;
            public ushort BucketStorageSizeBytes;
            public uint CompressionScaleRange;
            public uint StorageSizeBytes;
            public uint FullBucketCount;
            public uint PartiallyFilledBucketCount;
            public ushort SphericalHarmonicsDegree;
        }
    }
}