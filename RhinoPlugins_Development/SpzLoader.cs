using System;
using System.IO;
using System.IO.Compression;

namespace RhinoPlugins_Development
{
    public static class SpzLoader
    {
        private const float SH_C0 = 0.2820948f;
        private const float SQRT1_2 = 0.7071068f;
        private const float COLOR_SCALE = 0.15f;
        private const uint NGSP_MAGIC = 0x5053474E;
        private const int HEADER_SIZE = 16;
        private const uint VERSION_FIRST_THREE = 2;
        private const uint VERSION_SMALLEST_THREE = 3;

        private static float Clamp(float v, float min, float max)
        {
            if (v < min) return min;
            if (v > max) return max;
            return v;
        }

        public static Splat[] LoadSpz(string path, out string errorMessage)
        {
            errorMessage = string.Empty;
            try
            {
                return ParseSpz(File.ReadAllBytes(path), out errorMessage);
            }
            catch (Exception ex)
            {
                errorMessage = ex.Message;
                return null;
            }
        }

        private static Splat[] ParseSpz(byte[] fileBytes, out string errorMessage)
        {
            errorMessage = string.Empty;

            if (fileBytes.Length < 10)
            {
                errorMessage = "File is too small to be a valid SPZ file.";
                return null;
            }

            byte[] buffer;
            try
            {
                using (MemoryStream ms = new MemoryStream(fileBytes))
                using (GZipStream gz = new GZipStream(ms, CompressionMode.Decompress))
                using (MemoryStream outMs = new MemoryStream())
                {
                    gz.CopyTo(outMs);
                    buffer = outMs.ToArray();
                }
            }
            catch (Exception ex)
            {
                errorMessage = "Failed to decompress SPZ file: " + ex.Message;
                return null;
            }

            if (buffer.Length < HEADER_SIZE)
            {
                errorMessage = "SPZ decompressed data is too short to contain a valid header.";
                return null;
            }

            uint magic = ReadUInt32LE(buffer, 0);
            uint version = ReadUInt32LE(buffer, 4);
            uint pointCount = ReadUInt32LE(buffer, 8);
            byte degree = buffer[12];
            byte fractionalBits = buffer[13];
            byte _ = buffer[14];

            if (magic != NGSP_MAGIC)
            {
                errorMessage = $"Invalid SPZ magic (expected 0x{NGSP_MAGIC:X8}, got 0x{magic:X8}).";
                return null;
            }

            if (version < VERSION_FIRST_THREE || version > VERSION_SMALLEST_THREE)
            {
                errorMessage = $"Unsupported SPZ version {version}. Only versions 2 and 3 are supported.";
                return null;
            }

            bool isSmallestThree = version >= VERSION_SMALLEST_THREE;
            int shDim = ShDimForDegree(degree);
            int quatBytes = isSmallestThree ? 4 : 3;

            long positionsSize = pointCount * 9L;
            long opacitiesSize = pointCount;
            long scalesSize = pointCount * 3L;
            long quatsSize = pointCount * 3L;
            long shSize = pointCount * (long)shDim * 3;
            long totalDataSize = positionsSize + opacitiesSize + scalesSize + quatsSize + shSize;
            long expectedSize = HEADER_SIZE + totalDataSize;

            if (buffer.Length < expectedSize)
            {
                errorMessage = $"SPZ decompressed data too short. Expected {expectedSize} bytes, got {buffer.Length}.";
                return null;
            }

            long posOffset = HEADER_SIZE;
            long opacityOffset = posOffset + positionsSize;
            long scaleOffset = opacityOffset + opacitiesSize;
            long quatOffset = scaleOffset + scalesSize;
            long shOffset = quatOffset + quatsSize;

            float quantScale = 1f / (1 << fractionalBits);

            Splat[] splats = new Splat[pointCount];
            for (int i = 0; i < pointCount; i++)
            {
                long pOff = posOffset + i * 9L;
                float rawX = Read24BitFixed(buffer, pOff) * quantScale;
                float rawY = Read24BitFixed(buffer, pOff + 3) * quantScale;
                float rawZ = Read24BitFixed(buffer, pOff + 6) * quantScale;
                float px = rawX;
                float py = -rawY;
                float pz = -rawZ;

                long sOff = scaleOffset + i * 3L;
                float sx = (float)Math.Exp(buffer[sOff] / 16f - 10f);
                float sy = (float)Math.Exp(buffer[sOff + 1] / 16f - 10f);
                float sz = (float)Math.Exp(buffer[sOff + 2] / 16f - 10f);

                long qOff = shOffset - quatsSize + i * quatBytes;
                float rw, rx, ry, rz;
                if (isSmallestThree)
                    UnpackSmallestThree(buffer, qOff, out rw, out rx, out ry, out rz);
                else
                    UnpackFirstThree(buffer, qOff, out rw, out rx, out ry, out rz);

                byte opacity = buffer[opacityOffset + i];

                long cOff = quatOffset + i * 3L;
                float sh0 = buffer[cOff] / 255f - 0.5f;
                float sh1 = buffer[cOff + 1] / 255f - 0.5f;
                float sh2 = buffer[cOff + 2] / 255f - 0.5f;
                float r = Clamp(0.5f + 1.880632f * sh0, 0f, 1f);
                float g = Clamp(0.5f + 1.880632f * sh1, 0f, 1f);
                float b = Clamp(0.5f + 1.880632f * sh2, 0f, 1f);

                splats[i] = new Splat
                {
                    X = px, Y = py, Z = pz,
                    ScaleX = sx, ScaleY = sy, ScaleZ = sz,
                    RotW = rw, RotX = rx, RotY = ry, RotZ = rz,
                    R = (byte)(r * 255f),
                    G = (byte)(g * 255f),
                    B = (byte)(b * 255f),
                    A = opacity
                };
            }

            return splats;
        }

        private static int Read24BitFixed(byte[] data, long offset)
        {
            int val = data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16);
            if ((val & 0x800000) != 0)
                val |= ~0xFFFFFF;
            return val;
        }

        private static uint ReadUInt32LE(byte[] data, int offset)
        {
            return (uint)(data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) | (data[offset + 3] << 24));
        }

        private static int ShDimForDegree(int degree)
        {
            switch (degree)
            {
                case 0: return 0;
                case 1: return 3;
                case 2: return 8;
                case 3: return 15;
                case 4: return 24;
                default: return 0;
            }
        }

        private static void UnpackFirstThree(byte[] data, long offset, out float w, out float x, out float y, out float z)
        {
            x = (data[offset] - 127.5f) / 127.5f;
            y = (data[offset + 1] - 127.5f) / 127.5f;
            z = (data[offset + 2] - 127.5f) / 127.5f;
            float w2 = 1f - x * x - y * y - z * z;
            w = w2 > 0f ? (float)Math.Sqrt(w2) : 0f;

            float norm = (float)Math.Sqrt(w * w + x * x + y * y + z * z);
            if (norm > 1e-6f)
            {
                w /= norm; x /= norm; y /= norm; z /= norm;
            }
            else
            {
                w = 1f; x = 0f; y = 0f; z = 0f;
            }
        }

        private static void UnpackSmallestThree(byte[] data, long offset, out float outW, out float outX, out float outY, out float outZ)
        {
            uint packed = (uint)(data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) | (data[offset + 3] << 24));
            int largestIdx = (int)(packed >> 30);

            float[] components = new float[4];
            float sumSquares = 0f;

            for (int i = 3; i >= 0; i--)
            {
                if (i != largestIdx)
                {
                    uint bits = packed & 0x1FF;
                    uint sign = (packed >> 9) & 1;
                    packed >>= 10;
                    float val = SQRT1_2 * bits / 511f;
                    if (sign == 1) val = -val;
                    components[i] = val;
                    sumSquares += val * val;
                }
            }

            components[largestIdx] = sumSquares < 1f ? (float)Math.Sqrt(1f - sumSquares) : 0f;

            outX = components[0];
            outY = components[1];
            outZ = components[2];
            outW = components[3];
        }
    }
}