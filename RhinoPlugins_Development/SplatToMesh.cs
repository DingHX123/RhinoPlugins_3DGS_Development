using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;

namespace RhinoPlugins_Development
{
    /// <summary>
    /// 把高斯泼溅数据转换为网格：密度网格化 + Otsu 阈值 + Surface Nets 提取 + Taubin 平滑 + 顶点着色。
    /// （兼容 net48：不使用 MathF / Math.Clamp / Array.Fill / unsafe 指针写）
    /// </summary>
    public static class SplatToMesh
    {
        // 单位立方体的 8 个角点与 12 条边（Surface Nets 用）
        private static readonly Tuple<int, int, int>[] _corners8 = new Tuple<int, int, int>[]
        {
            Tuple.Create(0, 0, 0), Tuple.Create(1, 0, 0),
            Tuple.Create(0, 1, 0), Tuple.Create(1, 1, 0),
            Tuple.Create(0, 0, 1), Tuple.Create(1, 0, 1),
            Tuple.Create(0, 1, 1), Tuple.Create(1, 1, 1)
        };

        private static readonly Tuple<int, int>[] _edges12 = new Tuple<int, int>[]
        {
            Tuple.Create(0, 1), Tuple.Create(0, 2), Tuple.Create(0, 4),
            Tuple.Create(1, 3), Tuple.Create(1, 5), Tuple.Create(2, 3),
            Tuple.Create(2, 6), Tuple.Create(3, 7), Tuple.Create(4, 5),
            Tuple.Create(4, 6), Tuple.Create(5, 7), Tuple.Create(6, 7)
        };

        private static float ClampF(float v, float min, float max)
        {
            if (v < min) return min;
            if (v > max) return max;
            return v;
        }

        private static int ClampI(int v, int min, int max)
        {
            if (v < min) return min;
            if (v > max) return max;
            return v;
        }

        // ==================== 主入口 ====================

        public static Mesh Execute(
            Splat[] splats,
            int resolution = 384,
            int kernelMax = 5,
            int smoothIterations = 15,
            float levelBias = 0.4f,
            int minComponent = 500,
            float minOpacity = 0.02f,
            float colorSharpen = 2f,
            Action<float> progress = null)
        {
            Action<float> report = progress ?? (delegate { });

            // 按不透明度过滤
            byte minAlpha = (byte)ClampF(minOpacity * 255f, 0f, 255f);
            List<Splat> filtered = new List<Splat>(splats.Length);
            for (int i = 0; i < splats.Length; i++)
            {
                if (splats[i].A >= minAlpha)
                    filtered.Add(splats[i]);
            }
            Splat[] gs = filtered.ToArray();
            if (gs.Length == 0)
                return null;

            float[] density, colNum, colNorm;
            Vector3 origin;
            float voxel;
            int dx, dy, dz;
            BuildDensityGrid(gs, resolution, kernelMax, colorSharpen,
                out density, out colNum, out colNorm, out origin, out voxel, out dx, out dy, out dz,
                f => report(0.35f * f));
            report(0.35f);

            // 求密度范围
            float maxDensity = float.MinValue;
            float minDensity = float.MaxValue;
            foreach (float d in density)
            {
                if (d > maxDensity) maxDensity = d;
                if (d < minDensity) minDensity = d;
            }

            // 收集显著密度值用于 Otsu
            float significantThreshold = maxDensity * 0.001f;
            int significantCount = 0;
            for (int j = 0; j < density.Length; j++)
            {
                if (density[j] > significantThreshold)
                    significantCount++;
            }
            if (significantCount == 0)
                return null;

            float[] values = new float[significantCount];
            int valueIndex = 0;
            for (int k = 0; k < density.Length; k++)
            {
                if (density[k] > significantThreshold)
                    values[valueIndex++] = density[k];
            }

            float level = OtsuLevel(values, 256) * levelBias;
            float span = maxDensity - minDensity;
            level = ClampF(level, minDensity + (1e-06f * span), maxDensity - (1e-06f * span));
            report(0.4f);

            // Surface Nets 提取
            float[] verts;
            int[] faces;
            SurfaceNets(density, level, voxel, origin, dx, dy, dz, out verts, out faces);
            density = null;
            report(0.55f);

            if (verts.Length == 0 || faces.Length == 0)
                return null;

            if (minComponent > 0)
                CleanComponents(ref verts, ref faces, minComponent);
            if (verts.Length == 0 || faces.Length == 0)
                return null;

            report(0.65f);

            if (smoothIterations > 0)
                TaubinSmooth(ref verts, faces, smoothIterations, 0.5f, -0.53f);
            report(0.75f);

            // 顶点着色
            int vertexCount = verts.Length / 3;
            float[] colors = SampleVertexColours(colNum, colNorm, verts, origin, voxel, dx, dy, dz);
            report(0.95f);

            Mesh mesh = new Mesh();
            for (int m = 0; m < vertexCount; m++)
                mesh.Vertices.Add(verts[m * 3], verts[m * 3 + 1], verts[m * 3 + 2]);

            int faceCount = faces.Length / 3;
            for (int n = 0; n < faceCount; n++)
                mesh.Faces.AddFace(faces[n * 3], faces[n * 3 + 1], faces[n * 3 + 2]);

            for (int c = 0; c < vertexCount; c++)
            {
                byte r = (byte)ClampF(colors[c * 3] * 255f, 0f, 255f);
                byte g = (byte)ClampF(colors[c * 3 + 1] * 255f, 0f, 255f);
                byte b = (byte)ClampF(colors[c * 3 + 2] * 255f, 0f, 255f);
                mesh.VertexColors.Add(Color.FromArgb(r, g, b));
            }

            mesh.Normals.ComputeNormals();
            mesh.Compact();
            report(1f);
            return mesh;
        }

        // ==================== 密度网格 ====================

        private static void BuildDensityGrid(
            Splat[] gs, int resolution, int kernelMax, float colorSharpen,
            out float[] density, out float[] colNum, out float[] colNorm,
            out Vector3 origin, out float voxel, out int dx, out int dy, out int dz,
            Action<float> progress)
        {
            int count = gs.Length;

            // 包围盒 + 平均尺度
            Vector3 min = new Vector3(gs[0].X, gs[0].Y, gs[0].Z);
            Vector3 max = min;
            float avgScale = 0f;
            for (int i = 0; i < count; i++)
            {
                Vector3 p = new Vector3(gs[i].X, gs[i].Y, gs[i].Z);
                min = Vector3.Min(min, p);
                max = Vector3.Max(max, p);
                avgScale += (gs[i].ScaleX + gs[i].ScaleY + gs[i].ScaleZ) / 3f;
            }
            avgScale /= count;

            float padding = 4f * avgScale;
            min -= new Vector3(padding);
            max += new Vector3(padding);

            Vector3 size = max - min;
            float largest = Math.Max(size.X, Math.Max(size.Y, size.Z));
            voxel = Math.Max(largest / resolution, 1e-08f);
            origin = min;
            dx = (int)Math.Ceiling(size.X / voxel) + 1;
            dy = (int)Math.Ceiling(size.Y / voxel) + 1;
            dz = (int)Math.Ceiling(size.Z / voxel) + 1;

            int cellCount = dx * dy * dz;
            density = new float[cellCount];
            colNum = new float[cellCount * 3];
            colNorm = new float[cellCount];
            bool sharpen = colorSharpen != 1f;

            // 预计算逆协方差与核半径
            float[] cov = new float[count * 9];
            int[] radii = new int[count];
            for (int i = 0; i < count; i++)
            {
                InverseCov3x3(gs[i], cov, i * 9);
                float maxScale = Math.Max(gs[i].ScaleX, Math.Max(gs[i].ScaleY, gs[i].ScaleZ));
                radii[i] = ClampI((int)Math.Ceiling(3f * maxScale / voxel), 1, kernelMax);
            }

            int progressStep = Math.Max(1, count / 100);
            for (int k = 0; k < count; k++)
            {
                int kernelRadius = radii[k];
                float opacity = gs[k].A / 255f;
                Vector3 color = new Vector3(gs[k].R / 255f, gs[k].G / 255f, gs[k].B / 255f);

                float c00 = cov[k * 9];
                float c01 = cov[k * 9 + 1];
                float c02 = cov[k * 9 + 2];
                float c10 = cov[k * 9 + 3];
                float c11 = cov[k * 9 + 4];
                float c12 = cov[k * 9 + 5];
                float c20 = cov[k * 9 + 6];
                float c21 = cov[k * 9 + 7];
                float c22 = cov[k * 9 + 8];

                float gx = (gs[k].X - origin.X) / voxel;
                float gy = (gs[k].Y - origin.Y) / voxel;
                float gz = (gs[k].Z - origin.Z) / voxel;
                int cx = (int)Math.Round(gx);
                int cy = (int)Math.Round(gy);
                int cz = (int)Math.Round(gz);

                for (int m = -kernelRadius; m <= kernelRadius; m++)
                {
                    for (int n = -kernelRadius; n <= kernelRadius; n++)
                    {
                        for (int o = -kernelRadius; o <= kernelRadius; o++)
                        {
                            int ix = cx + m;
                            int iy = cy + n;
                            int iz = cz + o;
                            if (ix >= dx || iy >= dy || iz >= dz)
                                continue;

                            float ox = (ix - gx) * voxel;
                            float oy = (iy - gy) * voxel;
                            float oz = (iz - gz) * voxel;
                            float dotX = c00 * ox + c01 * oy + c02 * oz;
                            float dotY = c10 * ox + c11 * oy + c12 * oz;
                            float dotZ = c20 * ox + c21 * oy + c22 * oz;
                            float mahalanobis = ox * dotX + oy * dotY + oz * dotZ;

                            if (mahalanobis < 9f)
                            {
                                float weight = opacity * (float)Math.Exp(-0.5f * mahalanobis);
                                int index = (ix * dy * dz) + (iy * dz) + iz;
                                density[index] += weight;

                                float weighted = sharpen ? (float)Math.Pow(weight, colorSharpen) : weight;
                                colNum[index * 3] += weighted * color.X;
                                colNum[index * 3 + 1] += weighted * color.Y;
                                colNum[index * 3 + 2] += weighted * color.Z;
                                if (sharpen)
                                    colNorm[index] += weighted;
                            }
                        }
                    }
                }

                if (k % progressStep == 0 && progress != null)
                    progress((float)k / count);
            }

            if (!sharpen)
                Array.Copy(density, colNorm, cellCount);
        }

        // ==================== 逆协方差矩阵 ====================

        private static void InverseCov3x3(Splat s, float[] buf, int offset)
        {
            float rotW = s.RotW, rotX = s.RotX, rotY = s.RotY, rotZ = s.RotZ;

            // 归一化四元数
            float norm = (float)Math.Sqrt(rotW * rotW + rotX * rotX + rotY * rotY + rotZ * rotZ);
            if (norm > 1e-12f)
            {
                rotW /= norm; rotX /= norm; rotY /= norm; rotZ /= norm;
            }

            // 旋转矩阵
            float r00 = 1f - 2f * (rotY * rotY + rotZ * rotZ);
            float r01 = 2f * (rotX * rotY - rotW * rotZ);
            float r02 = 2f * (rotX * rotZ + rotW * rotY);
            float r10 = 2f * (rotX * rotY + rotW * rotZ);
            float r11 = 1f - 2f * (rotX * rotX + rotZ * rotZ);
            float r12 = 2f * (rotY * rotZ - rotW * rotX);
            float r20 = 2f * (rotX * rotZ - rotW * rotY);
            float r21 = 2f * (rotY * rotZ + rotW * rotX);
            float r22 = 1f - 2f * (rotX * rotX + rotY * rotY);

            // 逆尺度
            float invSX = 1f / Math.Max(s.ScaleX * s.ScaleX, 1e-16f);
            float invSY = 1f / Math.Max(s.ScaleY * s.ScaleY, 1e-16f);
            float invSZ = 1f / Math.Max(s.ScaleZ * s.ScaleZ, 1e-16f);

            buf[offset] = (r00 * r00 * invSX) + (r01 * r01 * invSY) + (r02 * r02 * invSZ);
            buf[offset + 1] = (r00 * r10 * invSX) + (r01 * r11 * invSY) + (r02 * r12 * invSZ);
            buf[offset + 2] = (r00 * r20 * invSX) + (r01 * r21 * invSY) + (r02 * r22 * invSZ);
            buf[offset + 3] = (r10 * r00 * invSX) + (r11 * r01 * invSY) + (r12 * r02 * invSZ);
            buf[offset + 4] = (r10 * r10 * invSX) + (r11 * r11 * invSY) + (r12 * r12 * invSZ);
            buf[offset + 5] = (r10 * r20 * invSX) + (r11 * r21 * invSY) + (r12 * r22 * invSZ);
            buf[offset + 6] = (r20 * r00 * invSX) + (r21 * r01 * invSY) + (r22 * r02 * invSZ);
            buf[offset + 7] = (r20 * r10 * invSX) + (r21 * r11 * invSY) + (r22 * r12 * invSZ);
            buf[offset + 8] = (r20 * r20 * invSX) + (r21 * r21 * invSY) + (r22 * r22 * invSZ);
        }

        // ==================== Otsu 阈值 ====================

        private static float OtsuLevel(float[] values, int bins = 256)
        {
            float min = values[0];
            float max = values[0];
            foreach (float v in values)
            {
                if (v < min) min = v;
                if (v > max) max = v;
            }
            if (max - min < 1e-12f)
                return min;

            double[] histogram = new double[bins];
            double[] centers = new double[bins];
            double binWidth = (max - min) / bins;
            for (int i = 0; i < bins; i++)
                centers[i] = min + (i + 0.5) * binWidth;

            foreach (float v in values)
            {
                int bin = ClampI((int)((v - min) / binWidth), 0, bins - 1);
                histogram[bin]++;
            }

            double totalWeight = 0.0;
            double totalSum = 0.0;
            for (int k = 0; k < bins; k++)
            {
                totalWeight += histogram[k];
                totalSum += histogram[k] * centers[k];
            }

            double bestVariance = -1.0;
            double bestLevel = centers[0];
            double cumWeight = 0.0;
            double cumSum = 0.0;
            for (int m = 0; m < bins; m++)
            {
                cumWeight += histogram[m];
                cumSum += histogram[m] * centers[m];
                double remainingWeight = totalWeight - cumWeight;
                if (cumWeight > 0.0 && remainingWeight > 0.0)
                {
                    double meanA = cumSum / cumWeight;
                    double meanB = (totalSum - cumSum) / remainingWeight;
                    double variance = cumWeight * remainingWeight * (meanA - meanB) * (meanA - meanB);
                    if (variance > bestVariance)
                    {
                        bestVariance = variance;
                        bestLevel = centers[m];
                    }
                }
            }
            return (float)bestLevel;
        }

        // ==================== 三线性采样 ====================

        private static float SampleTrilinear1(float[] grid, int dx, int dy, int dz, float gx, float gy, float gz)
        {
            gx = ClampF(gx, 0f, dx - 1.0001f);
            gy = ClampF(gy, 0f, dy - 1.0001f);
            gz = ClampF(gz, 0f, dz - 1.0001f);
            int x0 = (int)gx, y0 = (int)gy, z0 = (int)gz;
            float fx = gx - x0, fy = gy - y0, fz = gz - z0;
            int x1 = Math.Min(x0 + 1, dx - 1);
            int y1 = Math.Min(y0 + 1, dy - 1);
            int z1 = Math.Min(z0 + 1, dz - 1);

            float result = 0f;
            for (int i = 0; i <= 1; i++)
            {
                for (int j = 0; j <= 1; j++)
                {
                    for (int k = 0; k <= 1; k++)
                    {
                        float w = ((i == 0 ? 1f - fx : fx) * (j == 0 ? 1f - fy : fy)) * (k == 0 ? 1f - fz : fz);
                        int xi = i == 0 ? x0 : x1;
                        int yi = j == 0 ? y0 : y1;
                        int zi = k == 0 ? z0 : z1;
                        result += w * grid[(xi * dy * dz) + (yi * dz) + zi];
                    }
                }
            }
            return result;
        }

        private static void SampleTrilinear3(float[] grid, int dx, int dy, int dz, float gx, float gy, float gz,
            out float r, out float g, out float b)
        {
            gx = ClampF(gx, 0f, dx - 1.0001f);
            gy = ClampF(gy, 0f, dy - 1.0001f);
            gz = ClampF(gz, 0f, dz - 1.0001f);
            int x0 = (int)gx, y0 = (int)gy, z0 = (int)gz;
            float fx = gx - x0, fy = gy - y0, fz = gz - z0;
            int x1 = Math.Min(x0 + 1, dx - 1);
            int y1 = Math.Min(y0 + 1, dy - 1);
            int z1 = Math.Min(z0 + 1, dz - 1);

            r = g = b = 0f;
            for (int i = 0; i <= 1; i++)
            {
                for (int j = 0; j <= 1; j++)
                {
                    for (int k = 0; k <= 1; k++)
                    {
                        float w = ((i == 0 ? 1f - fx : fx) * (j == 0 ? 1f - fy : fy)) * (k == 0 ? 1f - fz : fz);
                        int xi = i == 0 ? x0 : x1;
                        int yi = j == 0 ? y0 : y1;
                        int zi = k == 0 ? z0 : z1;
                        int index = ((xi * dy * dz) + (yi * dz) + zi) * 3;
                        r += w * grid[index];
                        g += w * grid[index + 1];
                        b += w * grid[index + 2];
                    }
                }
            }
        }

        private static float[] SampleVertexColours(float[] colNum, float[] colNorm, float[] verts,
            Vector3 origin, float voxel, int dx, int dy, int dz)
        {
            int vertexCount = verts.Length / 3;
            float[] colors = new float[vertexCount * 3];
            for (int i = 0; i < vertexCount; i++)
            {
                float gx = (verts[i * 3] - origin.X) / voxel;
                float gy = (verts[i * 3 + 1] - origin.Y) / voxel;
                float gz = (verts[i * 3 + 2] - origin.Z) / voxel;

                float r, g, b;
                SampleTrilinear3(colNum, dx, dy, dz, gx, gy, gz, out r, out g, out b);
                float norm = SampleTrilinear1(colNorm, dx, dy, dz, gx, gy, gz);
                float invNorm = 1f / Math.Max(norm, 1e-08f);

                colors[i * 3] = ClampF(r * invNorm, 0f, 1f);
                colors[i * 3 + 1] = ClampF(g * invNorm, 0f, 1f);
                colors[i * 3 + 2] = ClampF(b * invNorm, 0f, 1f);
            }
            return colors;
        }

        // ==================== Surface Nets ====================

        private static float VolumeAt(int x, int y, int z, float[] vol, int dy, int dz)
        {
            return vol[(x * dy * dz) + (y * dz) + z];
        }

        private static int VertexIdAt(int x, int y, int z, int[] vid, int cy, int cz)
        {
            return vid[(x * cy * cz) + (y * cz) + z];
        }

        private static void SurfaceNets(float[] vol, float level, float voxel, Vector3 origin,
            int dx, int dy, int dz, out float[] outVerts, out int[] outFaces)
        {
            int cx = dx - 1, cy = dy - 1, cz = dz - 1;
            if (cx < 1 || cy < 1 || cz < 1)
            {
                outVerts = new float[0];
                outFaces = new int[0];
                return;
            }

            // 每个 cell 至多一个顶点
            int[] vid = new int[cx * cy * cz];
            for (int i = 0; i < vid.Length; i++)
                vid[i] = -1;

            List<float> vertices = new List<float>();
            float[] cornerValues = new float[8];

            for (int i = 0; i < cx; i++)
            {
                for (int j = 0; j < cy; j++)
                {
                    for (int k = 0; k < cz; k++)
                    {
                        // 8 个角点密度
                        bool anyInside = false;
                        bool allInside = true;
                        for (int c = 0; c < 8; c++)
                        {
                            Tuple<int, int, int> corner = _corners8[c];
                            cornerValues[c] = VolumeAt(i + corner.Item1, j + corner.Item2, k + corner.Item3, vol, dy, dz);
                            bool inside = cornerValues[c] >= level;
                            anyInside |= inside;
                            allInside &= inside;
                        }

                        // 表面 cell：既有内部也有外部角点
                        if (!anyInside || allInside)
                            continue;

                        // 沿 12 条边线性插值求交点，取平均得到顶点位置
                        float sumX = 0f, sumY = 0f, sumZ = 0f;
                        int crossings = 0;
                        for (int e = 0; e < 12; e++)
                        {
                            int a = _edges12[e].Item1;
                            int b = _edges12[e].Item2;
                            if ((cornerValues[a] >= level) == (cornerValues[b] >= level))
                                continue;

                            Tuple<int, int, int> ca = _corners8[a];
                            Tuple<int, int, int> cb = _corners8[b];
                            float delta = cornerValues[b] - cornerValues[a];
                            float t = Math.Abs(delta) > 1e-12f ? (level - cornerValues[a]) / delta : 0.5f;
                            t = ClampF(t, 0f, 1f);

                            sumX += ca.Item1 + (t * (cb.Item1 - ca.Item1));
                            sumY += ca.Item2 + (t * (cb.Item2 - ca.Item2));
                            sumZ += ca.Item3 + (t * (cb.Item3 - ca.Item3));
                            crossings++;
                        }

                        if (crossings != 0)
                        {
                            float inv = 1f / crossings;
                            sumX *= inv;
                            sumY *= inv;
                            sumZ *= inv;

                            float vx = origin.X + ((i + sumX) * voxel);
                            float vy = origin.Y + ((j + sumY) * voxel);
                            float vz = origin.Z + ((k + sumZ) * voxel);

                            vid[(i * cy * cz) + (j * cz) + k] = vertices.Count / 3;
                            vertices.Add(vx);
                            vertices.Add(vy);
                            vertices.Add(vz);
                        }
                    }
                }
            }

            // 沿三个轴方向扫描相邻 cell，密度符号变化处生成四边形
            List<int> faces = new List<int>();

            // X 轴方向
            for (int i = 0; i < cx - 1; i++)
            {
                for (int j = 1; j < cy; j++)
                {
                    for (int k = 1; k < cz; k++)
                    {
                        bool insideA = VolumeAt(i, j, k, vol, dy, dz) >= level;
                        bool insideB = VolumeAt(i + 1, j, k, vol, dy, dz) >= level;
                        if (insideA != insideB)
                        {
                            int a = VertexIdAt(i, j, k, vid, cy, cz);
                            int b = VertexIdAt(i, j - 1, k, vid, cy, cz);
                            int c = VertexIdAt(i, j - 1, k - 1, vid, cy, cz);
                            int d = VertexIdAt(i, j, k - 1, vid, cy, cz);
                            EmitQuad(faces, a, b, c, d, insideA);
                        }
                    }
                }
            }

            // Y 轴方向
            for (int i = 1; i < cx; i++)
            {
                for (int j = 0; j < cy - 1; j++)
                {
                    for (int k = 1; k < cz; k++)
                    {
                        bool insideA = VolumeAt(i, j, k, vol, dy, dz) >= level;
                        bool insideB = VolumeAt(i, j + 1, k, vol, dy, dz) >= level;
                        if (insideA != insideB)
                        {
                            int a = VertexIdAt(i, j, k, vid, cy, cz);
                            int b = VertexIdAt(i, j, k - 1, vid, cy, cz);
                            int c = VertexIdAt(i - 1, j, k - 1, vid, cy, cz);
                            int d = VertexIdAt(i - 1, j, k, vid, cy, cz);
                            EmitQuad(faces, a, b, c, d, insideA);
                        }
                    }
                }
            }

            // Z 轴方向
            for (int i = 1; i < cx; i++)
            {
                for (int j = 1; j < cy; j++)
                {
                    for (int k = 0; k < cz - 1; k++)
                    {
                        bool insideA = VolumeAt(i, j, k, vol, dy, dz) >= level;
                        bool insideB = VolumeAt(i, j, k + 1, vol, dy, dz) >= level;
                        if (insideA != insideB)
                        {
                            int a = VertexIdAt(i, j, k, vid, cy, cz);
                            int b = VertexIdAt(i - 1, j, k, vid, cy, cz);
                            int c = VertexIdAt(i - 1, j - 1, k, vid, cy, cz);
                            int d = VertexIdAt(i, j - 1, k, vid, cy, cz);
                            EmitQuad(faces, a, b, c, d, insideA);
                        }
                    }
                }
            }

            outVerts = vertices.ToArray();
            outFaces = faces.ToArray();
        }

        private static void EmitQuad(List<int> faces, int a, int b, int c, int d, bool flip)
        {
            if (a >= 0 && b >= 0 && c >= 0 && d >= 0)
            {
                if (!flip)
                {
                    faces.Add(a); faces.Add(b); faces.Add(d);
                    faces.Add(b); faces.Add(c); faces.Add(d);
                }
                else
                {
                    faces.Add(a); faces.Add(d); faces.Add(b);
                    faces.Add(b); faces.Add(d); faces.Add(c);
                }
            }
        }

        // ==================== Taubin 平滑 ====================

        private static void TaubinSmooth(ref float[] verts, int[] faces, int iters, float lambda = 0.5f, float mu = -0.53f)
        {
            int vertexCount = verts.Length / 3;
            int faceCount = faces.Length / 3;

            // 邻接表
            HashSet<int>[] adjacency = new HashSet<int>[vertexCount];
            for (int i = 0; i < vertexCount; i++)
                adjacency[i] = new HashSet<int>();
            for (int j = 0; j < faceCount; j++)
            {
                int a = faces[j * 3], b = faces[j * 3 + 1], c = faces[j * 3 + 2];
                adjacency[a].Add(b); adjacency[a].Add(c);
                adjacency[b].Add(a); adjacency[b].Add(c);
                adjacency[c].Add(a); adjacency[c].Add(b);
            }

            int[][] neighbors = new int[vertexCount][];
            for (int i = 0; i < vertexCount; i++)
            {
                HashSet<int> set = adjacency[i];
                int[] list = new int[set.Count];
                set.CopyTo(list);
                neighbors[i] = list;
            }

            float[] updated = new float[vertexCount * 3];
            for (int iter = 0; iter < iters * 2; iter++)
            {
                // 交替收缩/扩张
                float factor = iter % 2 == 0 ? lambda : mu;
                Array.Copy(verts, updated, verts.Length);
                for (int v = 0; v < vertexCount; v++)
                {
                    float sumX = 0f, sumY = 0f, sumZ = 0f;
                    int[] nbrs = neighbors[v];
                    for (int n = 0; n < nbrs.Length; n++)
                    {
                        sumX += verts[nbrs[n] * 3];
                        sumY += verts[nbrs[n] * 3 + 1];
                        sumZ += verts[nbrs[n] * 3 + 2];
                    }
                    float invCount = 1f / Math.Max(1, nbrs.Length);
                    updated[v * 3] = verts[v * 3] + factor * (sumX * invCount - verts[v * 3]);
                    updated[v * 3 + 1] = verts[v * 3 + 1] + factor * (sumY * invCount - verts[v * 3 + 1]);
                    updated[v * 3 + 2] = verts[v * 3 + 2] + factor * (sumZ * invCount - verts[v * 3 + 2]);
                }
                float[] swap = updated;
                updated = verts;
                verts = swap;
            }
        }

        // ==================== 连通分量清理 ====================

        private static int UnionFind(int[] parent, int x)
        {
            while (parent[x] != x)
            {
                parent[x] = parent[parent[x]];  // 路径压缩
                x = parent[x];
            }
            return x;
        }

        private static void Union(int[] parent, int a, int b)
        {
            int ra = UnionFind(parent, a);
            int rb = UnionFind(parent, b);
            if (ra != rb)
                parent[ra] = rb;
        }

        private static void CleanComponents(ref float[] verts, ref int[] faces, int minVerts)
        {
            int vertexCount = verts.Length / 3;
            int faceCount = faces.Length / 3;

            // 并查集
            int[] parent = new int[vertexCount];
            for (int i = 0; i < vertexCount; i++)
                parent[i] = i;
            for (int j = 0; j < faceCount; j++)
            {
                Union(parent, faces[j * 3], faces[j * 3 + 1]);
                Union(parent, faces[j * 3], faces[j * 3 + 2]);
            }

            // 分量编号
            Dictionary<int, int> rootToComponent = new Dictionary<int, int>();
            int[] componentOf = new int[vertexCount];
            for (int k = 0; k < vertexCount; k++)
            {
                int root = UnionFind(parent, k);
                int component;
                if (!rootToComponent.TryGetValue(root, out component))
                {
                    component = rootToComponent.Count;
                    rootToComponent[root] = component;
                }
                componentOf[k] = component;
            }
            int componentCount = rootToComponent.Count;

            // 每个分量的顶点数 / 面数 / 有符号体积
            int[] componentVerts = new int[componentCount];
            int[] componentFaces = new int[componentCount];
            double[] signedVolume = new double[componentCount];
            for (int m = 0; m < vertexCount; m++)
                componentVerts[componentOf[m]]++;

            for (int n = 0; n < faceCount; n++)
            {
                int a = faces[n * 3], b = faces[n * 3 + 1], c = faces[n * 3 + 2];
                int comp = componentOf[a];
                componentFaces[comp]++;

                double ax = verts[a * 3], ay = verts[a * 3 + 1], az = verts[a * 3 + 2];
                double bx = verts[b * 3], by = verts[b * 3 + 1], bz = verts[b * 3 + 2];
                double cx = verts[c * 3], cy = verts[c * 3 + 1], cz = verts[c * 3 + 2];
                double crossX = by * cz - bz * cy;
                double crossY = bz * cx - bx * cz;
                double crossZ = bx * cy - by * cx;
                signedVolume[comp] += ax * crossX + ay * crossY + az * crossZ;
            }

            // 找最大分量
            int mainComponent = -1;
            int bestFaceCount = -1;
            for (int c = 0; c < componentCount; c++)
            {
                if (componentVerts[c] >= minVerts && componentFaces[c] > bestFaceCount)
                {
                    bestFaceCount = componentFaces[c];
                    mainComponent = c;
                }
            }

            if (mainComponent < 0)
            {
                verts = new float[0];
                faces = new int[0];
                return;
            }

            // 每个分量的包围盒
            float[] bboxMin = new float[componentCount * 3];
            float[] bboxMax = new float[componentCount * 3];
            for (int c = 0; c < componentCount; c++)
            {
                bboxMin[c * 3] = float.MaxValue; bboxMin[c * 3 + 1] = float.MaxValue; bboxMin[c * 3 + 2] = float.MaxValue;
                bboxMax[c * 3] = float.MinValue; bboxMax[c * 3 + 1] = float.MinValue; bboxMax[c * 3 + 2] = float.MinValue;
            }
            for (int v = 0; v < vertexCount; v++)
            {
                int comp = componentOf[v] * 3;
                bboxMin[comp] = Math.Min(bboxMin[comp], verts[v * 3]);
                bboxMin[comp + 1] = Math.Min(bboxMin[comp + 1], verts[v * 3 + 1]);
                bboxMin[comp + 2] = Math.Min(bboxMin[comp + 2], verts[v * 3 + 2]);
                bboxMax[comp] = Math.Max(bboxMax[comp], verts[v * 3]);
                bboxMax[comp + 1] = Math.Max(bboxMax[comp + 1], verts[v * 3 + 1]);
                bboxMax[comp + 2] = Math.Max(bboxMax[comp + 2], verts[v * 3 + 2]);
            }

            // 剔除与主分量包围盒重合但绕向相反的"内衬"分量
            int mainBase = mainComponent * 3;
            float epsilon = 0.0001f * Math.Max(
                bboxMax[mainBase] - bboxMin[mainBase],
                Math.Max(bboxMax[mainBase + 1] - bboxMin[mainBase + 1],
                         bboxMax[mainBase + 2] - bboxMin[mainBase + 2]));
            double mainVolumeSign = Math.Sign(signedVolume[mainComponent]);

            bool[] componentKeep = new bool[componentCount];
            for (int c = 0; c < componentCount; c++)
            {
                if (componentVerts[c] < minVerts)
                {
                    componentKeep[c] = false;
                    continue;
                }

                int baseIndex = c * 3;
                bool sameBox =
                    bboxMin[baseIndex] >= bboxMin[mainBase] - epsilon &&
                    bboxMax[baseIndex] <= bboxMax[mainBase] + epsilon &&
                    bboxMin[baseIndex + 1] >= bboxMin[mainBase + 1] - epsilon &&
                    bboxMax[baseIndex + 1] <= bboxMax[mainBase + 1] + epsilon &&
                    bboxMin[baseIndex + 2] >= bboxMin[mainBase + 2] - epsilon &&
                    bboxMax[baseIndex + 2] <= bboxMax[mainBase + 2] + epsilon;
                bool isInnerShell = sameBox && Math.Sign(signedVolume[c]) != mainVolumeSign && c != mainComponent;
                componentKeep[c] = !isInnerShell;
            }

            // 重排顶点与面
            bool[] vertexKeep = new bool[vertexCount];
            for (int v = 0; v < vertexCount; v++)
            {
                if (componentKeep[componentOf[v]])
                    vertexKeep[v] = true;
            }

            int[] newIndex = new int[vertexCount];
            for (int v = 0; v < newIndex.Length; v++)
                newIndex[v] = -1;
            int nextIndex = 0;
            List<float> newVerts = new List<float>();
            for (int v = 0; v < vertexCount; v++)
            {
                if (vertexKeep[v])
                {
                    newIndex[v] = nextIndex++;
                    newVerts.Add(verts[v * 3]);
                    newVerts.Add(verts[v * 3 + 1]);
                    newVerts.Add(verts[v * 3 + 2]);
                }
            }

            List<int> newFaces = new List<int>();
            for (int f = 0; f < faceCount; f++)
            {
                int a = newIndex[faces[f * 3]];
                int b = newIndex[faces[f * 3 + 1]];
                int c = newIndex[faces[f * 3 + 2]];
                if (a >= 0 && b >= 0 && c >= 0)
                {
                    newFaces.Add(a);
                    newFaces.Add(b);
                    newFaces.Add(c);
                }
            }

            verts = newVerts.ToArray();
            faces = newFaces.ToArray();
        }
    }
}
