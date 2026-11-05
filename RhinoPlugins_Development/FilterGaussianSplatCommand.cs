using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;

namespace RhinoPlugins_Development
{
    /// <summary>
    /// FilterGaussianSplat：点选一个高斯椭球，在窗口中查看其颜色/透明度等参数，
    /// 并设置颜色变化阈值、搜索放大倍率与最低透明度进行递归过滤：
    /// 从选中椭球出发，将其按"搜索放大倍率"放大后与周围椭球做相交测试，
    /// 颜色变化与透明度满足条件的椭球被保留并作为新的扩散源继续向外搜索（BFS），
    /// 最终仅显示过滤后的所有椭球（注册撤销，窗口内可一键恢复全部）。
    /// </summary>
    public class FilterGaussianSplatCommand : Command
    {
        public override string EnglishName => "FilterGaussianSplat";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            // ========== 1. 选择 3dgs 模型 ==========
            SplatInstance instance = SplatDisplayConduit.GetTargetInstance(doc, "选择 3dgs 模型（代理点云）");
            if (instance == null)
            {
                RhinoApp.WriteLine("未找到 splat 模型，请先运行 ImportGaussianSplat 导入并显示模型");
                return Result.Failure;
            }

            Splat[] splats = instance.Splats;
            if (splats == null || splats.Length == 0)
            {
                RhinoApp.WriteLine("所选模型没有 splat 数据");
                return Result.Failure;
            }

            // ========== 2. 点选一个高斯椭球（点击其中心附近） ==========
            var gp = new GetPoint();
            gp.SetCommandPrompt("点选一个高斯椭球（点击其中心附近）");
            if (gp.Get() != GetResult.Point)
                return Result.Cancel;

            // ========== 3. 拾取点 → 最近 splat（超出局部范围视为无效拾取） ==========
            double pickDistance;
            int seedIndex = FindNearestSplat(splats, gp.Point(), out pickDistance);
            if (seedIndex < 0)
            {
                RhinoApp.WriteLine("拾取位置附近没有找到高斯椭球，请点击模型上椭球的中心附近");
                return Result.Failure;
            }

            Splat seed = splats[seedIndex];
            RhinoApp.WriteLine(string.Format(
                "已选中椭球 #{0}: RGB({1},{2},{3}) 透明度 {4:F3}",
                seedIndex, seed.R, seed.G, seed.B, seed.A / 255.0));

            // ========== 4. 打开过滤窗口（非模态：Rhino 视口可继续操作；可反复应用，可恢复） ==========
            var window = new FilterGaussianSplatWindow(doc, instance, splats, seedIndex);
            var helper = new System.Windows.Interop.WindowInteropHelper(window);
            helper.Owner = RhinoApp.MainWindowHandle();
            window.Show();

            return Result.Success;
        }

        /// <summary>
        /// 拾取点 → 最近 splat 索引。用空间网格加速最近邻查找；
        /// 拾取点离最近椭球中心超过 8×平均半轴视为无效（返回 -1）。
        /// </summary>
        public static int FindNearestSplat(Splat[] splats, Point3d target, out double distance)
        {
            distance = 0.0;
            int n = splats.Length;
            if (n == 0)
                return -1;

            // 平均最大半轴（抽样估计），决定网格尺寸
            double avgR = 0.0;
            int stride = Math.Max(1, n / 4096);
            int sampled = 0;
            for (int i = 0; i < n; i += stride)
            {
                Splat s = splats[i];
                avgR += Math.Max(s.ScaleX, Math.Max(s.ScaleY, s.ScaleZ));
                sampled++;
            }
            avgR = sampled > 0 ? avgR / sampled : 1.0;
            if (avgR < 1e-9)
                avgR = 1e-3;
            double cell = 2.0 * avgR;

            var grid = new Dictionary<long, List<int>>();
            for (int i = 0; i < n; i++)
            {
                Splat s = splats[i];
                long key = CellKey(
                    Math.Floor(s.X / cell),
                    Math.Floor(s.Y / cell),
                    Math.Floor(s.Z / cell));
                List<int> list;
                if (!grid.TryGetValue(key, out list))
                {
                    list = new List<int>(4);
                    grid.Add(key, list);
                }
                list.Add(i);
            }

            double bestDist2 = double.MaxValue;
            int bestIdx = -1;
            int tcx = (int)Math.Floor(target.X / cell);
            int tcy = (int)Math.Floor(target.Y / cell);
            int tcz = (int)Math.Floor(target.Z / cell);

            // 扩环搜索：一旦最短距离小于内环边界即不可能更近，提前结束
            for (int ring = 0; ring <= 6; ring++)
            {
                for (int dx = -ring; dx <= ring; dx++)
                {
                    for (int dy = -ring; dy <= ring; dy++)
                    {
                        for (int dz = -ring; dz <= ring; dz++)
                        {
                            if (Math.Max(Math.Abs(dx), Math.Max(Math.Abs(dy), Math.Abs(dz))) != ring)
                                continue; // 只扫当前环的壳

                            long key = CellKey(tcx + dx, tcy + dy, tcz + dz);
                            List<int> list;
                            if (!grid.TryGetValue(key, out list))
                                continue;

                            foreach (int i in list)
                            {
                                Splat s = splats[i];
                                double ddx = s.X - target.X;
                                double ddy = s.Y - target.Y;
                                double ddz = s.Z - target.Z;
                                double dist2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                if (dist2 < bestDist2)
                                {
                                    bestDist2 = dist2;
                                    bestIdx = i;
                                }
                            }
                        }
                    }
                }

                if (bestIdx >= 0 && bestDist2 <= (double)ring * ring * cell * cell)
                    break; // 更远处不可能更近
            }

            if (bestIdx < 0)
                return -1;
            distance = Math.Sqrt(bestDist2);
            if (distance > 8.0 * avgR)
                return -1; // 拾取点离任何椭球都太远
            return bestIdx;
        }

        /// <summary>
        /// 从种子椭球递归扩散（BFS）构建过滤后的椭球集合：
        /// 当前椭球按 searchMultiplier 放大后，与空间网格邻域内的椭球做近似相交测试
        /// （沿中心连线方向的支撑半径之和），且候选椭球须满足：
        /// 与种子颜色的 RGB 欧氏距离 ≤ colorThreshold、不透明度 ≥ minOpacity。
        /// 返回按原始顺序排列的过滤后数组。
        /// </summary>
        public static Splat[] BuildFilteredSplats(Splat[] splats, int seedIndex,
            double colorThreshold, double searchMultiplier, double minOpacity, out int acceptedCount)
        {
            acceptedCount = 0;
            int n = splats.Length;
            if (n == 0 || seedIndex < 0 || seedIndex >= n)
                return splats;

            float splatScale = SplatDisplayConduit.SplatScale;
            double m = Math.Max(1.0, searchMultiplier);

            // 全局最大半轴与平均半轴（网格尺寸与搜索上界用）
            double maxR = 0.0, sumR = 0.0;
            for (int i = 0; i < n; i++)
            {
                Splat s = splats[i];
                double r = Math.Max(s.ScaleX, Math.Max(s.ScaleY, s.ScaleZ)) * splatScale;
                sumR += r;
                if (r > maxR)
                    maxR = r;
            }
            double avgR = sumR / n;
            double cell = Math.Max(2.0 * avgR * m, 1e-12);
            int maxRange = 48;

            // 空间哈希网格（键为网格坐标哈希，碰撞候选靠距离检查过滤）
            var grid = new Dictionary<long, List<int>>();
            for (int i = 0; i < n; i++)
            {
                Splat s = splats[i];
                long key = CellKey(
                    Math.Floor(s.X / cell),
                    Math.Floor(s.Y / cell),
                    Math.Floor(s.Z / cell));
                List<int> list;
                if (!grid.TryGetValue(key, out list))
                {
                    list = new List<int>(4);
                    grid.Add(key, list);
                }
                list.Add(i);
            }

            // 状态：0=未访问 1=已排除（颜色/透明度，确定性拒绝） 2=已接受
            var state = new byte[n];
            var queue = new Queue<int>();
            state[seedIndex] = 2;
            queue.Enqueue(seedIndex);
            int count = 1;

            Splat seed = splats[seedIndex];
            double thr2 = colorThreshold * colorThreshold;
            double minA = minOpacity * 255.0;

            while (queue.Count > 0)
            {
                int i = queue.Dequeue();
                Splat si = splats[i];

                // 当前椭球的旋转矩阵（归一化四元数，与渲染管线一致）
                double qw = si.RotW, qx = si.RotX, qy = si.RotY, qz = si.RotZ;
                double qn = Math.Sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
                if (qn < 1e-10) { qw = 1.0; qn = 1.0; }
                qw /= qn; qx /= qn; qy /= qn; qz /= qn;
                double a00 = 1 - 2 * (qy * qy + qz * qz), a01 = 2 * (qx * qy - qw * qz), a02 = 2 * (qx * qz + qw * qy);
                double a10 = 2 * (qx * qy + qw * qz), a11 = 1 - 2 * (qx * qx + qz * qz), a12 = 2 * (qy * qz - qw * qx);
                double a20 = 2 * (qx * qz - qw * qy), a21 = 2 * (qy * qz + qw * qx), a22 = 1 - 2 * (qx * qx + qy * qy);

                // 放大后的半轴与支撑半径上界（方向半径 ≤ √(s1²+s2²+s3²)）
                double s1 = si.ScaleX * splatScale * m;
                double s2 = si.ScaleY * splatScale * m;
                double s3 = si.ScaleZ * splatScale * m;
                double reach = Math.Sqrt(s1 * s1 + s2 * s2 + s3 * s3) + maxR;
                int range = Math.Min(maxRange, (int)Math.Ceiling(reach / cell));
                int ccx = (int)Math.Floor(si.X / cell);
                int ccy = (int)Math.Floor(si.Y / cell);
                int ccz = (int)Math.Floor(si.Z / cell);

                for (int dx = -range; dx <= range; dx++)
                {
                    for (int dy = -range; dy <= range; dy++)
                    {
                        for (int dz = -range; dz <= range; dz++)
                        {
                            long key = CellKey(ccx + dx, ccy + dy, ccz + dz);
                            List<int> list;
                            if (!grid.TryGetValue(key, out list))
                                continue;

                            foreach (int j in list)
                            {
                                if (state[j] != 0)
                                    continue;

                                Splat sj = splats[j];

                                // 最低透明度（确定性排除）
                                if (sj.A < minA)
                                {
                                    state[j] = 1;
                                    continue;
                                }

                                // 颜色变化阈值：与种子颜色的 RGB 欧氏距离（确定性排除）
                                double dr = sj.R - seed.R;
                                double dg = sj.G - seed.G;
                                double db = sj.B - seed.B;
                                if (dr * dr + dg * dg + db * db > thr2)
                                {
                                    state[j] = 1;
                                    continue;
                                }

                                // 距离粗筛
                                double ddx = sj.X - si.X;
                                double ddy = sj.Y - si.Y;
                                double ddz = sj.Z - si.Z;
                                double dist2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                if (dist2 > reach * reach)
                                    continue;

                                // 近似相交测试：沿中心连线方向，放大椭球 i 的支撑半径
                                // 与椭球 j 的支撑半径之和 ≥ 中心距离
                                double dist = Math.Sqrt(dist2);
                                double ux = 1.0, uy = 0.0, uz = 0.0;
                                if (dist > 1e-12)
                                {
                                    ux = ddx / dist; uy = ddy / dist; uz = ddz / dist;
                                }
                                double ri = SupportRadius(a00, a01, a02, a10, a11, a12, a20, a21, a22,
                                    s1, s2, s3, ux, uy, uz);

                                double rj;
                                if (dist > 1e-12)
                                {
                                    double jw = sj.RotW, jx = sj.RotX, jy = sj.RotY, jz = sj.RotZ;
                                    double jn = Math.Sqrt(jw * jw + jx * jx + jy * jy + jz * jz);
                                    if (jn < 1e-10) { jw = 1.0; jn = 1.0; }
                                    jw /= jn; jx /= jn; jy /= jn; jz /= jn;
                                    double b00 = 1 - 2 * (jy * jy + jz * jz), b01 = 2 * (jx * jy - jw * jz), b02 = 2 * (jx * jz + jw * jy);
                                    double b10 = 2 * (jx * jy + jw * jz), b11 = 1 - 2 * (jx * jx + jz * jz), b12 = 2 * (jy * jz - jw * jx);
                                    double b20 = 2 * (jx * jz - jw * jy), b21 = 2 * (jy * jz + jw * jx), b22 = 1 - 2 * (jx * jx + jy * jy);
                                    rj = SupportRadius(b00, b01, b02, b10, b11, b12, b20, b21, b22,
                                        sj.ScaleX * splatScale, sj.ScaleY * splatScale, sj.ScaleZ * splatScale,
                                        ux, uy, uz);
                                }
                                else
                                {
                                    rj = 0.0;
                                }

                                // 不相交：保持未访问（后续可能经其他椭球再次发现）
                                if (dist > ri + rj)
                                    continue;

                                state[j] = 2;
                                count++;
                                queue.Enqueue(j);
                            }
                        }
                    }
                }
            }

            acceptedCount = count;
            var result = new List<Splat>(count);
            for (int i = 0; i < n; i++)
            {
                if (state[i] == 2)
                    result.Add(splats[i]);
            }
            return result.ToArray();
        }

        /// <summary>椭球沿单位方向 d 的支撑半径：√(Σ (sₖ·(d·eₖ))²)，eₖ 为旋转矩阵列。</summary>
        private static double SupportRadius(
            double a00, double a01, double a02,
            double a10, double a11, double a12,
            double a20, double a21, double a22,
            double s1, double s2, double s3,
            double ux, double uy, double uz)
        {
            double d1 = ux * a00 + uy * a10 + uz * a20;
            double d2 = ux * a01 + uy * a11 + uz * a21;
            double d3 = ux * a02 + uy * a12 + uz * a22;
            return Math.Sqrt(s1 * s1 * d1 * d1 + s2 * s2 * d2 * d2 + s3 * s3 * d3 * d3);
        }

        private static long CellKey(double cx, double cy, double cz)
        {
            long ix = (long)cx, iy = (long)cy, iz = (long)cz;
            return ix * 73856093L ^ iy * 19349663L ^ iz * 83492791L;
        }
    }
}
