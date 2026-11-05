using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace RhinoPlugins_Development
{
    /// <summary>AutoCrop 剔除强度（对齐参考实现 CropStrength）。</summary>
    public enum SplatCropStrength
    {
        Low,
        Medium,
        High
    }

    /// <summary>AutoCrop 结果：Keep[i] 为 true 表示保留该 splat。</summary>
    public class SplatAutoCropResult
    {
        public bool[] Keep;
        public int Kept;

        public static SplatAutoCropResult Fallback(int count)
        {
            bool[] keep = new bool[count];
            for (int i = 0; i < count; i++)
                keep[i] = true;
            return new SplatAutoCropResult { Keep = keep, Kept = count };
        }
    }

    /// <summary>
    /// 移植自参考实现 GaussianSplat.Data.AutoCrop：基于体素密度聚类的离群 splat（floaters）
    /// 自动剔除。floaters 是针状伪影的主要来源之一。
    /// </summary>
    public static class SplatAutoCrop
    {
        public static SplatAutoCropResult Compute(Splat[] splats, SplatCropStrength strength)
        {
            int count = splats.Length;
            Params p = For(strength);
            byte[] mask = new byte[count];

            // splat 的"尺寸"取 3D 协方差对角线的最大标准差（与参考实现一致）
            float[] v = new float[count];
            for (int i = 0; i < count; i++)
                v[i] = MaxWorldStd(splats[i]);

            float medianV = Percentile(v, 0.5f, null);
            for (int j = 0; j < count; j++)
            {
                Splat s = splats[j];
                if ((s.A / 255f) < p.MinAlpha ||
                    v[j] > medianV * p.HugeFactor ||
                    !IsFinite(s.X) || !IsFinite(s.Y) || !IsFinite(s.Z))
                {
                    mask[j] = 1;
                }
            }

            float[][] axis = new float[3][];
            for (int k = 0; k < 3; k++)
            {
                axis[k] = new float[count];
                for (int i = 0; i < count; i++)
                {
                    Splat s = splats[i];
                    axis[k][i] = k == 0 ? s.X : (k == 1 ? s.Y : s.Z);
                }
            }

            Point3d pMin = new Point3d(
                Percentile(axis[0], 0.005f, mask),
                Percentile(axis[1], 0.005f, mask),
                Percentile(axis[2], 0.005f, mask));
            Point3d pMax = new Point3d(
                Percentile(axis[0], 0.995f, mask),
                Percentile(axis[1], 0.995f, mask),
                Percentile(axis[2], 0.995f, mask));
            Point3d center = new Point3d(
                Percentile(axis[0], 0.5f, mask),
                Percentile(axis[1], 0.5f, mask),
                Percentile(axis[2], 0.5f, mask));

            bool[] keepFirst;
            BoundingBox? firstBox = Cluster(splats, mask, Expand(new BoundingBox(pMin, pMax), 0.05), center, p, out keepFirst);
            if (!firstBox.HasValue || !firstBox.Value.IsValid)
                return SplatAutoCropResult.Fallback(count);

            BoundingBox bounds = Expand(firstBox.Value, 0.15);
            bool[] keep;
            if (!Cluster(splats, mask, bounds, bounds.Center, p, out keep).HasValue)
                return SplatAutoCropResult.Fallback(count);

            int kept = 0;
            for (int i = 0; i < count; i++)
            {
                if (mask[i] == 0)
                {
                    if (keep[i])
                        kept++;
                    else
                        mask[i] = 1;
                }
            }
            if (kept == 0)
                return SplatAutoCropResult.Fallback(count);

            bool[] result = new bool[count];
            for (int i = 0; i < count; i++)
                result[i] = mask[i] == 0;
            return new SplatAutoCropResult { Keep = result, Kept = kept };
        }

        private static BoundingBox? Cluster(Splat[] splats, byte[] hidden, BoundingBox bounds, Point3d center, Params p, out bool[] keepSplat)
        {
            int count = splats.Length;
            keepSplat = new bool[count];
            double maxDiag = bounds.Diagonal.MaximumCoordinate;
            if (maxDiag <= 0.0)
                return null;

            double cell = maxDiag / 128.0;
            Point3d bmin = bounds.Min;
            int nx = Math.Max(1, (int)Math.Ceiling(bounds.Diagonal.X / cell));
            int ny = Math.Max(1, (int)Math.Ceiling(bounds.Diagonal.Y / cell));
            int nz = Math.Max(1, (int)Math.Ceiling(bounds.Diagonal.Z / cell));

            int[] cellOf = new int[count];
            float[] density = new float[nx * ny * nz];
            for (int i = 0; i < count; i++)
            {
                cellOf[i] = -1;
                if (hidden[i] != 0)
                    continue;
                Splat s = splats[i];
                int ix = (int)Math.Floor((s.X - bmin.X) / cell);
                int iy = (int)Math.Floor((s.Y - bmin.Y) / cell);
                int iz = (int)Math.Floor((s.Z - bmin.Z) / cell);
                if (ix >= 0 && iy >= 0 && iz >= 0 && ix < nx && iy < ny && iz < nz)
                {
                    int idx = (iz * ny + iy) * nx + ix;
                    cellOf[i] = idx;
                    density[idx] += s.A / 255f;
                }
            }

            // 阈值：占用体素密度的加权中位数 * DensityK
            List<float> occupied = new List<float>();
            double total = 0.0;
            foreach (float d in density)
            {
                if (d > 0f)
                {
                    occupied.Add(d);
                    total += d;
                }
            }
            if (occupied.Count == 0)
                return null;

            occupied.Sort();
            double half = 0.0;
            float medianCell = occupied[occupied.Count - 1];
            for (int j = occupied.Count - 1; j >= 0; j--)
            {
                half += occupied[j];
                if (half >= total * 0.5)
                {
                    medianCell = occupied[j];
                    break;
                }
            }
            float threshold = medianCell * p.DensityK;

            // BFS 密度聚类
            int[] labels = new int[density.Length];
            List<double> weights = new List<double>();
            List<Vector3d> centroids = new List<Vector3d>();
            List<Point3d> cellMin = new List<Point3d>();
            List<Point3d> cellMax = new List<Point3d>();
            Queue<int> queue = new Queue<int>();
            for (int k = 0; k < density.Length; k++)
            {
                if (labels[k] == 0 && density[k] > 0f && density[k] >= threshold)
                {
                    int id = weights.Count + 1;
                    double weight = 0.0;
                    Vector3d weightedPos = new Vector3d();
                    Point3d cMin = new Point3d(nx, ny, nz);
                    Point3d cMax = new Point3d(-1.0, -1.0, -1.0);
                    labels[k] = id;
                    queue.Enqueue(k);
                    while (queue.Count > 0)
                    {
                        int cur = queue.Dequeue();
                        int cx = cur % nx;
                        int cy = (cur / nx) % ny;
                        int cz = cur / (nx * ny);
                        weight += density[cur];
                        weightedPos += new Vector3d(cx, cy, cz) * density[cur];
                        cMin = new Point3d(Math.Min(cMin.X, cx), Math.Min(cMin.Y, cy), Math.Min(cMin.Z, cz));
                        cMax = new Point3d(Math.Max(cMax.X, cx), Math.Max(cMax.Y, cy), Math.Max(cMax.Z, cz));
                        for (int dz = -1; dz <= 1; dz++)
                        {
                            for (int dy = -1; dy <= 1; dy++)
                            {
                                for (int dx = -1; dx <= 1; dx++)
                                {
                                    int tx = cx + dx, ty = cy + dy, tz = cz + dz;
                                    if (tx >= 0 && ty >= 0 && tz >= 0 && tx < nx && ty < ny && tz < nz)
                                    {
                                        int nIdx = (tz * ny + ty) * nx + tx;
                                        if (labels[nIdx] == 0 && density[nIdx] > 0f && density[nIdx] >= threshold)
                                        {
                                            labels[nIdx] = id;
                                            queue.Enqueue(nIdx);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    weights.Add(weight);
                    centroids.Add(weightedPos / weight);
                    cellMin.Add(cMin);
                    cellMax.Add(cMax);
                }
            }
            if (weights.Count == 0)
                return null;

            // 以距离中心的加权得分选取主簇
            Vector3d centerGrid = new Vector3d(
                (center.X - bmin.X) / cell,
                (center.Y - bmin.Y) / cell,
                (center.Z - bmin.Z) / cell);
            int best = 0;
            double bestScore = double.MinValue;
            for (int m = 0; m < weights.Count; m++)
            {
                double dist = (centroids[m] - centerGrid).Length / 128.0;
                double score = weights[m] / (1.0 + (16.0 * dist * dist));
                if (score > bestScore)
                {
                    bestScore = score;
                    best = m;
                }
            }

            double bestSpan = (cellMax[best] - cellMin[best]).Length + 1.0;
            bool[] keepCluster = new bool[weights.Count + 1];
            Point3d gridMin = cellMin[best];
            Point3d gridMax = cellMax[best];
            for (int n = 0; n < weights.Count; n++)
            {
                bool keep = n == best;
                if (!keep && weights[n] >= weights[best] * p.KeepFrac)
                {
                    double gapX = Math.Max(0.0, Math.Max(cellMin[n].X - cellMax[best].X, cellMin[best].X - cellMax[n].X));
                    double gapY = Math.Max(0.0, Math.Max(cellMin[n].Y - cellMax[best].Y, cellMin[best].Y - cellMax[n].Y));
                    double gapZ = Math.Max(0.0, Math.Max(cellMin[n].Z - cellMax[best].Z, cellMin[best].Z - cellMax[n].Z));
                    keep = Math.Sqrt(gapX * gapX + gapY * gapY + gapZ * gapZ) <= p.Gap * bestSpan;
                }
                keepCluster[n + 1] = keep;
                if (keep)
                {
                    gridMin = new Point3d(Math.Min(gridMin.X, cellMin[n].X), Math.Min(gridMin.Y, cellMin[n].Y), Math.Min(gridMin.Z, cellMin[n].Z));
                    gridMax = new Point3d(Math.Max(gridMax.X, cellMax[n].X), Math.Max(gridMax.Y, cellMax[n].Y), Math.Max(gridMax.Z, cellMax[n].Z));
                }
            }

            for (int i = 0; i < count; i++)
            {
                int cellIdx = cellOf[i];
                if (cellIdx < 0)
                    continue;
                int cx = cellIdx % nx;
                int cy = (cellIdx / nx) % ny;
                int cz = cellIdx / (nx * ny);
                bool found = false;
                for (int dz = -1; dz <= 1 && !found; dz++)
                {
                    for (int dy = -1; dy <= 1 && !found; dy++)
                    {
                        for (int dx = -1; dx <= 1 && !found; dx++)
                        {
                            int tx = cx + dx, ty = cy + dy, tz = cz + dz;
                            if (tx >= 0 && ty >= 0 && tz >= 0 && tx < nx && ty < ny && tz < nz)
                            {
                                found = keepCluster[labels[(tz * ny + ty) * nx + tx]];
                            }
                        }
                    }
                }
                keepSplat[i] = found;
            }

            return new BoundingBox(
                new Point3d(bmin.X + (gridMin.X - 1.0) * cell, bmin.Y + (gridMin.Y - 1.0) * cell, bmin.Z + (gridMin.Z - 1.0) * cell),
                new Point3d(bmin.X + (gridMax.X + 2.0) * cell, bmin.Y + (gridMax.Y + 2.0) * cell, bmin.Z + (gridMax.Z + 2.0) * cell));
        }

        /// <summary>3D 协方差对角线的最大标准差（协方差 = (R·S)(R·S)^T，四元数与缩放计算，与参考实现一致）。</summary>
        private static float MaxWorldStd(Splat s)
        {
            float qw = s.RotW, qx = s.RotX, qy = s.RotY, qz = s.RotZ;
            float r00 = 1f - 2f * (qy * qy + qz * qz);
            float r01 = 2f * (qx * qy - qw * qz);
            float r02 = 2f * (qx * qz + qw * qy);
            float r10 = 2f * (qx * qy + qw * qz);
            float r11 = 1f - 2f * (qx * qx + qz * qz);
            float r12 = 2f * (qy * qz - qw * qx);
            float r20 = 2f * (qx * qz - qw * qy);
            float r21 = 2f * (qy * qz + qw * qx);
            float r22 = 1f - 2f * (qx * qx + qy * qy);
            float sx = s.ScaleX, sy = s.ScaleY, sz = s.ScaleZ;
            float c00 = r00 * sx * (r00 * sx) + r01 * sy * (r01 * sy) + r02 * sz * (r02 * sz);
            float c11 = r10 * sx * (r10 * sx) + r11 * sy * (r11 * sy) + r12 * sz * (r12 * sz);
            float c22 = r20 * sx * (r20 * sx) + r21 * sy * (r21 * sy) + r22 * sz * (r22 * sz);
            return (float)Math.Sqrt(Math.Max(c00, Math.Max(c11, c22)));
        }

        private static BoundingBox Expand(BoundingBox b, double frac)
        {
            Vector3d diag = b.Diagonal * frac;
            return new BoundingBox(b.Min - diag, b.Max + diag);
        }

        private static float Percentile(float[] v, float q, byte[] mask)
        {
            int length = v.Length;
            int step = Math.Max(1, length / 200000);
            List<float> list = new List<float>(Math.Min(length, 200001));
            for (int i = 0; i < length; i += step)
            {
                if ((mask == null || mask[i] == 0) && IsFinite(v[i]))
                    list.Add(v[i]);
            }
            if (list.Count == 0)
                return 0f;
            list.Sort();
            int idx = (int)(q * (list.Count - 1));
            if (idx < 0) idx = 0;
            if (idx > list.Count - 1) idx = list.Count - 1;
            return list[idx];
        }

        private static Params For(SplatCropStrength s)
        {
            if (s == SplatCropStrength.Low)
                return new Params(0.01176471f, 60f, 0.01f, 0.01f, 0.5f, 0.002f);
            if (s == SplatCropStrength.High)
                return new Params(0.07843138f, 15f, 0.08f, 0.1f, 0.05f, 0.02f);
            return new Params(0.03137255f, 30f, 0.03f, 0.03f, 0.15f, 0.005f);
        }

        private static bool IsFinite(float v)
        {
            return !float.IsNaN(v) && !float.IsInfinity(v);
        }

        private struct Params
        {
            public readonly float MinAlpha;
            public readonly float HugeFactor;
            public readonly float DensityK;
            public readonly float KeepFrac;
            public readonly float Gap;
            public readonly float Trim;

            public Params(float minAlpha, float hugeFactor, float densityK, float keepFrac, float gap, float trim)
            {
                MinAlpha = minAlpha;
                HugeFactor = hugeFactor;
                DensityK = densityK;
                KeepFrac = keepFrac;
                Gap = gap;
                Trim = trim;
            }
        }
    }

    /// <summary>
    /// 移植自参考实现 Placement.TryLevel：RANSAC 搜索地面平面（法向与 Z 夹角 ≤45° 且上下点数均衡），
    /// 主成分细化法向后整平到 XY 平面，并将模型 XY 居中、地面置于 Z=0。
    /// </summary>
    public static class SplatFloorLeveler
    {
        public static bool TryLevel(Splat[] splats, out Transform leveling, out double tiltDegrees)
        {
            leveling = Transform.Identity;
            tiltDegrees = 0.0;

            List<Point3d> samples = SampleWorld(splats, 150000);
            if (samples.Count < 100)
                return false;

            BoundingBox box = new BoundingBox(samples);
            double diag = box.Diagonal.Length;
            if (diag <= 0.0)
                return false;
            double band = diag * 0.004;

            Random random = new Random(12345);
            List<Point3d> subsample = new List<Point3d>();
            int subStep = Math.Max(1, samples.Count / 20000);
            for (int i = 0; i < samples.Count; i += subStep)
                subsample.Add(samples[i]);

            double minAbsZ = Math.Cos(0.25 * Math.PI);
            int bestInliers = 0;
            Vector3d bestNormal = new Vector3d();
            Point3d bestPoint = new Point3d();
            for (int iter = 0; iter < 1500; iter++)
            {
                Point3d p0 = samples[random.Next(samples.Count)];
                Point3d p1 = samples[random.Next(samples.Count)];
                Point3d p2 = samples[random.Next(samples.Count)];
                Vector3d n = Normalized(Vector3d.CrossProduct(p1 - p0, p2 - p0));
                if (n.IsZero || Math.Abs(n.Z) < minAbsZ)
                    continue;

                int inliers = 0, above = 0, below = 0;
                for (int j = 0; j < subsample.Count; j++)
                {
                    double d = (subsample[j] - p0) * n;
                    if (Math.Abs(d) < band)
                        inliers++;
                    else if (d > 0.0)
                        above++;
                    else
                        below++;
                }
                if (Math.Min(above, below) <= subsample.Count * 0.15 && inliers > bestInliers)
                {
                    bestInliers = inliers;
                    bestNormal = n;
                    bestPoint = p0;
                }
            }
            if (bestInliers < subsample.Count * 0.05)
                return false;

            // 平面内点的主成分细化（最小特征向量为法向）
            List<Point3d> planePts = new List<Point3d>();
            foreach (Point3d pt in samples)
            {
                if (Math.Abs((pt - bestPoint) * bestNormal) < band)
                    planePts.Add(pt);
            }
            if (planePts.Count < 3)
                return false;

            Point3d centroid = new Point3d();
            foreach (Point3d pt in planePts)
                centroid += pt;
            centroid /= planePts.Count;

            double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
            foreach (Point3d pt in planePts)
            {
                Vector3d d = pt - centroid;
                xx += d.X * d.X;
                xy += d.X * d.Y;
                xz += d.X * d.Z;
                yy += d.Y * d.Y;
                yz += d.Y * d.Z;
                zz += d.Z * d.Z;
            }
            double[] eigVals;
            Vector3d[] eigVecs;
            Eigen3Solver.Solve(xx, xy, xz, yy, yz, zz, out eigVals, out eigVecs);

            Vector3d normal = eigVecs[0];
            if (normal.Z < 0.0)
                normal = -normal;

            // 保证法向指向点数较多的一侧（向上）
            int up = 0, down = 0;
            foreach (Point3d pt in samples)
            {
                double d = (pt - centroid) * normal;
                if (d > band) up++;
                else if (d < -band) down++;
            }
            if (down > up)
                normal = -normal;
            normal = Normalized(normal);

            tiltDegrees = Math.Acos(Math.Max(-1.0, Math.Min(1.0, normal.Z))) * 180.0 / Math.PI;
            Transform rotation = RotationTo(normal, new Vector3d(0.0, 0.0, 1.0), centroid);

            // XY 居中并把地面放到 Z=0（对齐参考实现）
            BoundingBox transformed = new BoundingBox();
            bool first = true;
            foreach (Point3d pt in samples)
            {
                Point3d tp = rotation * pt;
                if (first)
                {
                    transformed = new BoundingBox(tp, tp);
                    first = false;
                }
                else
                {
                    transformed.Union(tp);
                }
            }
            Point3d rotatedCenter = rotation * centroid;
            Transform translate = Transform.Identity;
            translate.M03 = -(transformed.Min.X + transformed.Max.X) / 2.0;
            translate.M13 = -(transformed.Min.Y + transformed.Max.Y) / 2.0;
            translate.M23 = -rotatedCenter.Z;
            leveling = translate * rotation;
            return true;
        }

        private static List<Point3d> SampleWorld(Splat[] splats, int max)
        {
            int step = Math.Max(1, splats.Length / max);
            List<Point3d> list = new List<Point3d>(Math.Min(splats.Length, max) + 1);
            for (int i = 0; i < splats.Length; i += step)
            {
                Splat s = splats[i];
                if (s.A >= 64 && IsFinite(s.X) && IsFinite(s.Y) && IsFinite(s.Z))
                    list.Add(new Point3d(s.X, s.Y, s.Z));
            }
            return list;
        }

        private static Vector3d Normalized(Vector3d v)
        {
            double len = Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
            if (len <= 1e-300)
                return new Vector3d(0.0, 0.0, 0.0);
            return new Vector3d(v.X / len, v.Y / len, v.Z / len);
        }

        private static Transform RotationTo(Vector3d from, Vector3d to, Point3d center)
        {
            Vector3d cross = Vector3d.CrossProduct(from, to);
            double sinA = cross.Length;
            double cosA = from * to;
            Transform t = Transform.Identity;
            if (sinA < 1e-12)
            {
                if (cosA <= 0.0)
                {
                    // 反平行：绕 X 轴旋转 180°
                    t.M11 = -1.0;
                    t.M22 = -1.0;
                }
            }
            else
            {
                Vector3d k = cross / sinA;
                double k1 = 1.0 - cosA;
                t.M00 = cosA + k.X * k.X * k1;
                t.M01 = k.X * k.Y * k1 - k.Z * sinA;
                t.M02 = k.X * k.Z * k1 + k.Y * sinA;
                t.M10 = k.Y * k.X * k1 + k.Z * sinA;
                t.M11 = cosA + k.Y * k.Y * k1;
                t.M12 = k.Y * k.Z * k1 - k.X * sinA;
                t.M20 = k.Z * k.X * k1 - k.Y * sinA;
                t.M21 = k.Z * k.Y * k1 + k.X * sinA;
                t.M22 = cosA + k.Z * k.Z * k1;
            }
            Point3d rotated = t * center;
            t.M03 = center.X - rotated.X;
            t.M13 = center.Y - rotated.Y;
            t.M23 = center.Z - rotated.Z;
            return t;
        }

        private static bool IsFinite(float v)
        {
            return !float.IsNaN(v) && !float.IsInfinity(v);
        }
    }

    /// <summary>3x3 对称矩阵特征分解（Jacobi 旋转，特征值升序，移植自参考实现 Eigen3）。</summary>
    internal static class Eigen3Solver
    {
        public static void Solve(double a00, double a01, double a02, double a11, double a12, double a22,
            out double[] values, out Vector3d[] vectors)
        {
            double[,] a = new double[3, 3] { { a00, a01, a02 }, { a01, a11, a12 }, { a02, a12, a22 } };
            double[,] v = new double[3, 3] { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };
            for (int sweep = 0; sweep < 12; sweep++)
            {
                if (Math.Abs(a[0, 1]) + Math.Abs(a[0, 2]) + Math.Abs(a[1, 2]) < 1e-30)
                    break;
                for (int k = 0; k < 2; k++)
                {
                    for (int m = k + 1; m < 3; m++)
                    {
                        if (Math.Abs(a[k, m]) >= 1e-300)
                        {
                            double theta = (a[m, m] - a[k, k]) / (2.0 * a[k, m]);
                            double t = theta == 0.0 ? 1.0 : Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1.0));
                            double c = 1.0 / Math.Sqrt(t * t + 1.0);
                            double s = t * c;
                            for (int i = 0; i < 3; i++)
                            {
                                double aik = a[i, k];
                                double aim = a[i, m];
                                a[i, k] = c * aik - s * aim;
                                a[i, m] = s * aik + c * aim;
                            }
                            for (int i = 0; i < 3; i++)
                            {
                                double aki = a[k, i];
                                double ami = a[m, i];
                                a[k, i] = c * aki - s * ami;
                                a[m, i] = s * aki + c * ami;
                            }
                            for (int i = 0; i < 3; i++)
                            {
                                double vki = v[i, k];
                                double vmi = v[i, m];
                                v[i, k] = c * vki - s * vmi;
                                v[i, m] = s * vki + c * vmi;
                            }
                        }
                    }
                }
            }

            int[] order = new int[3] { 0, 1, 2 };
            Array.Sort(order, delegate (int x, int y) { return a[x, x].CompareTo(a[y, y]); });
            values = new double[3];
            vectors = new Vector3d[3];
            for (int j = 0; j < 3; j++)
            {
                int col = order[j];
                values[j] = a[col, col];
                vectors[j] = new Vector3d(v[0, col], v[1, col], v[2, col]);
            }
        }
    }
}
