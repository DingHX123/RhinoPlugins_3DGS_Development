using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Commands;
using Rhino.Display;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;

namespace RhinoPlugins_Development
{
    /// <summary>
    /// EllipseOuterHull：选取若干封闭椭圆曲线，生成它们的外接曲线。三种模式：
    ///
    /// Auto（默认）：自动识别——同时做真矩形与椭圆两种收缩拟合，
    ///   依据覆盖率判定：覆盖率 = 外接曲线内部的所选椭圆面积 / 外接曲线面积
    ///   =（椭圆总面积 − loss）/ 外接曲线面积，自动选用覆盖率更高（椭圆填充更满）的形状。
    ///
    /// Polygon：收缩拟合为真矩形（四角恒为直角）。以包住凸包的 PCA 对齐外接矩形为初始
    ///   （loss = 0），逐边沿法向二分内移、整体旋转/平移，全部在矩形形状约束内进行；
    ///   内移按比例分步多轮收敛，椭圆密度小（支撑松弛大）的一侧自然收缩更多。
    ///
    /// Ellipse：收缩拟合为单个椭圆。以加权质心 + 协方差主轴放大到全包含的外接椭圆为初始，
    ///   pattern-search 缩轴 / 移心 / 转角，逐步向单个椭圆简化。
    ///
    /// loss 定义：所选椭圆"联合面积"落在拟合区域外的部分——各椭圆内部拟均匀采样点
    ///   按重叠重数加权（被 m 个椭圆覆盖的点权重 1/m，重叠只计一次），
    ///   避免椭圆相互重叠导致面积重复计算、收缩预算与覆盖率被虚高。
    ///   每次收缩的幅度（二分上界）与方向（先收缩哪条边/哪个参数）均由
    ///   "loss ≤ LossTolerance × 椭圆联合面积" 判定；
    ///   Auto 的覆盖率 = （椭圆联合面积 − loss）/ 外接曲线面积。
    /// 基础外包轮廓由支撑函数凸包得到：每条椭圆表示为 p(t) = C + U·cos t + V·sin t，
    /// 在 N 个方向上取支撑值 h(θ) = d·C + √((d·U)² + (d·V)²) 的最大值。
    /// 非椭圆的封闭曲线退化为密集采样点参与外包计算（无面积贡献，不参与 loss）。
    /// </summary>
    public class EllipseOuterHullCommand : Command
    {
        public override string EnglishName => "EllipseOuterHull";

        private const double TwoPi = 2.0 * Math.PI;

        /// <summary>方向采样数（决定外包轮廓与初始外接多边形的精度）。</summary>
        private const int DirectionCount = 1440;

        /// <summary>非椭圆封闭曲线的采样点数。</summary>
        private const int FallbackSampleCount = 256;

        /// <summary>每个椭圆内部用于 loss 计算的拟均匀（sunflower）采样点数。</summary>
        private const int LatticeCount = 384;

        /// <summary>sunflower 采样的黄金角。</summary>
        private static readonly double GoldenAngle = Math.PI * (3.0 - Math.Sqrt(5.0));

        /// <summary>2D 凸体：p(t) = C + U·cos t + V·sin t；U、V 均为零向量时退化为点。</summary>
        private struct HullBody
        {
            public double Cx, Cy;
            public double Ux, Uy;
            public double Vx, Vy;
        }

        /// <summary>参与 loss 计算的采样体：单个椭圆内部的采样点、逐点面积权重与重叠重数。</summary>
        private sealed class SampleBody
        {
            public double[] X;
            public double[] Y;
            public double[] Mult;         // 每个采样点被多少个椭圆包含（≥1）
            public double WeightPerPoint; // Area / LatticeCount
            public double Area;
        }

        /// <summary>收缩拟合结果：单个椭圆（中心 + 半轴 + 旋转角）。</summary>
        private sealed class EllipseFit
        {
            public double Cx, Cy, A, B, Phi;
        }

        /// <summary>单位圆盘内 sunflower 采样点（缓存），[x0,y0,x1,y1,...]。</summary>
        private static double[] _unitDiskSamples;

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            // ========== 1. 选取封闭曲线 ==========
            var go = new GetObject();
            go.SetCommandPrompt("选取若干封闭椭圆曲线（至少两条）");
            go.GeometryFilter = ObjectType.Curve;
            go.GeometryAttributeFilter = GeometryAttributeFilter.ClosedCurve;
            go.GroupSelect = true;
            go.GetMultiple(2, 0);
            if (go.CommandResult() != Result.Success)
                return go.CommandResult();

            var curves = new List<Curve>(go.ObjectCount);
            for (int i = 0; i < go.ObjectCount; i++)
            {
                Curve crv = go.Object(i).Curve();
                if (crv != null)
                    curves.Add(crv);
            }
            if (curves.Count < 2)
            {
                RhinoApp.WriteLine("有效曲线不足两条，无法计算外接曲线");
                return Result.Failure;
            }

            double tol = doc.ModelAbsoluteTolerance;

            // ========== 2. 提取椭圆（解析）与非椭圆（采样点） ==========
            var ellipsePlanes = new List<Plane>();
            var ellipseRadius1 = new List<double>();
            var ellipseRadius2 = new List<double>();
            var fallbackPoints = new List<Point3d>();

            foreach (Curve crv in curves)
            {
                Ellipse ell;
                if (crv.TryGetEllipse(out ell, tol))
                {
                    ellipsePlanes.Add(ell.Plane);
                    ellipseRadius1.Add(ell.Radius1);
                    ellipseRadius2.Add(ell.Radius2);
                    continue;
                }

                double[] tParams = crv.DivideByCount(FallbackSampleCount, true);
                if (tParams == null || tParams.Length == 0)
                {
                    RhinoApp.WriteLine("一条封闭曲线无法采样，已跳过");
                    continue;
                }
                foreach (double tp in tParams)
                    fallbackPoints.Add(crv.PointAt(tp));
            }

            if (ellipsePlanes.Count + fallbackPoints.Count < 2)
            {
                RhinoApp.WriteLine("有效凸体不足两个");
                return Result.Failure;
            }

            // ========== 3. 模式与收缩参数 ==========
            int modeIndex = 0; // 0=Auto 1=Polygon 2=Ellipse
            double lossTolerance = 0.05;

            var getOption = new GetOption();
            getOption.SetCommandPrompt("设置外接模式与收缩参数，按 Enter 继续");
            getOption.AcceptNothing(true);
            int modeListIndex = getOption.AddOptionList("Mode", new[] { "Auto", "Polygon", "Ellipse" }, 0);
            OptionDouble lossOption = new OptionDouble(lossTolerance, 0.0, 0.5);
            getOption.AddOptionDouble("LossTolerance", ref lossOption, "允许的椭圆未覆盖面积比例（收缩停止条件）");

            GetResult optionResult;
            do
            {
                optionResult = getOption.Get();
                if (optionResult == GetResult.Cancel)
                    return Result.Cancel;
                if (optionResult == GetResult.Option && getOption.Option() != null
                    && getOption.Option().Index == modeListIndex)
                {
                    modeIndex = getOption.Option().CurrentListOptionIndex;
                }
            } while (optionResult == GetResult.Option);

            lossTolerance = lossOption.CurrentValue;

            // ========== 4. 公共平面：法线取椭圆法线平均（自动统一朝向），无椭圆时用当前 CPlane ==========
            Vector3d normal;
            Point3d origin;
            if (ellipsePlanes.Count > 0)
            {
                Vector3d refNormal = ellipsePlanes[0].Normal;
                Vector3d normalSum = Vector3d.Zero;
                Vector3d centerSum = Vector3d.Zero;
                double maxDeviation = 0.0;
                for (int i = 0; i < ellipsePlanes.Count; i++)
                {
                    Vector3d n = ellipsePlanes[i].Normal;
                    if (n * refNormal < 0.0)
                        n = -n;
                    normalSum += n;
                    centerSum += (Vector3d)ellipsePlanes[i].Origin;
                    maxDeviation = Math.Max(maxDeviation, 1.0 - Math.Abs(n * refNormal));
                }
                normal = normalSum.Length > 1e-12 ? normalSum : refNormal;
                normal.Unitize();
                origin = (Point3d)(centerSum / ellipsePlanes.Count);

                if (fallbackPoints.Count > 0)
                {
                    Vector3d ptSum = Vector3d.Zero;
                    foreach (Point3d p in fallbackPoints)
                        ptSum += (Vector3d)p;
                    origin = (Point3d)((centerSum + ptSum) / (ellipsePlanes.Count + fallbackPoints.Count));
                }

                if (maxDeviation > 1e-6)
                    RhinoApp.WriteLine("注意：所选椭圆不完全共面，已投影到平均平面计算外接曲线");
            }
            else
            {
                RhinoView view = doc.Views.ActiveView;
                RhinoViewport viewport = view != null ? view.ActiveViewport : null;
                Plane cplane = viewport != null ? viewport.ConstructionPlane() : Plane.WorldXY;
                normal = cplane.Normal;

                Vector3d ptSum2 = Vector3d.Zero;
                foreach (Point3d p in fallbackPoints)
                    ptSum2 += (Vector3d)p;
                origin = (Point3d)(ptSum2 / fallbackPoints.Count);
            }

            Vector3d aux = Math.Abs(normal.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis;
            Vector3d xAxis = aux - normal * (aux * normal);
            xAxis.Unitize();
            Vector3d yAxis = Vector3d.CrossProduct(normal, xAxis);
            Plane plane = new Plane(origin, xAxis, yAxis);

            // ========== 5. 世界 → 平面 2D 凸体 ==========
            var bodies = new List<HullBody>(ellipsePlanes.Count + fallbackPoints.Count);
            for (int i = 0; i < ellipsePlanes.Count; i++)
            {
                Plane ep = ellipsePlanes[i];
                Vector3d dc = ep.Origin - origin;
                HullBody b = new HullBody
                {
                    Cx = dc * xAxis,
                    Cy = dc * yAxis,
                    Ux = ellipseRadius1[i] * (ep.XAxis * xAxis),
                    Uy = ellipseRadius1[i] * (ep.XAxis * yAxis),
                    Vx = ellipseRadius2[i] * (ep.YAxis * xAxis),
                    Vy = ellipseRadius2[i] * (ep.YAxis * yAxis)
                };
                if (b.Ux * b.Ux + b.Uy * b.Uy < 1e-24 && b.Vx * b.Vx + b.Vy * b.Vy < 1e-24)
                {
                    b.Ux = 0.0; b.Uy = 0.0; b.Vx = 0.0; b.Vy = 0.0;
                }
                bodies.Add(b);
            }
            foreach (Point3d p in fallbackPoints)
            {
                Vector3d dc = p - origin;
                HullBody b = new HullBody();
                b.Cx = dc * xAxis;
                b.Cy = dc * yAxis;
                bodies.Add(b);
            }

            // ========== 6. 支撑函数外包轮廓 ==========
            int bodyCount = bodies.Count;
            double step = TwoPi / DirectionCount;
            double[] supportH = new double[DirectionCount];
            for (int k = 0; k < DirectionCount; k++)
            {
                double dx = Math.Cos(k * step);
                double dy = Math.Sin(k * step);
                double bestH = SupportValue(bodies[0], dx, dy);
                for (int i = 1; i < bodyCount; i++)
                {
                    double h = SupportValue(bodies[i], dx, dy);
                    if (h > bestH)
                        bestH = h;
                }
                supportH[k] = bestH;
            }

            // ========== 7. 收缩拟合（loss = 椭圆联合面积未覆盖部分，重叠只计一次） ==========
            var samples = BuildSampleBodies(bodies, out double unionArea);
            if (unionArea <= 1e-18)
            {
                RhinoApp.WriteLine("所选曲线没有可度量的面积，无法进行外接拟合");
                return Result.Failure;
            }
            double budget = lossTolerance * unionArea;

            Curve resultCurve = null;
            string report;

            if (modeIndex == 1)
            {
                double finalArea;
                resultCurve = FitShrunkPolygon(bodies, samples, budget, unionArea, lossTolerance, supportH,
                    out finalArea, out _, out report);
            }
            else if (modeIndex == 2)
            {
                double finalArea;
                resultCurve = FitShrunkEllipse(samples, budget, unionArea, lossTolerance,
                    out finalArea, out _, out report);
            }
            else
            {
                // Auto：矩形与椭圆各拟合一轮，按覆盖率（区域内椭圆联合面积/外接面积）选用更高者
                double polyArea, polyLoss;
                string polyReport;
                Curve polyCurve = FitShrunkPolygon(bodies, samples, budget, unionArea, lossTolerance, supportH, out polyArea, out polyLoss, out polyReport);

                double ellArea, ellLoss;
                string ellReport;
                Curve ellCurve = FitShrunkEllipse(samples, budget, unionArea, lossTolerance, out ellArea, out ellLoss, out ellReport);

                if (polyCurve == null && ellCurve == null)
                {
                    RhinoApp.WriteLine("收缩拟合适配失败（" + polyReport + "；" + ellReport + "）");
                    return Result.Failure;
                }

                double polyRatio = polyCurve != null && polyArea > 1e-12
                    ? (unionArea - polyLoss) / polyArea : -1.0;
                double ellRatio = ellCurve != null && ellArea > 1e-12
                    ? (unionArea - ellLoss) / ellArea : -1.0;

                bool usePolygon = polyRatio >= ellRatio;
                if (usePolygon)
                {
                    resultCurve = polyCurve;
                    report = "Auto 判定为矩形" + string.Format("（覆盖率：矩形 {0:P1} ≥ 椭圆 {1:P1}）", Math.Max(polyRatio, 0.0), Math.Max(ellRatio, 0.0))
                        + "：" + polyReport;
                }
                else
                {
                    resultCurve = ellCurve;
                    report = "Auto 判定为椭圆" + string.Format("（覆盖率：椭圆 {0:P1} > 矩形 {1:P1}）", ellRatio, Math.Max(polyRatio, 0.0))
                        + "：" + ellReport;
                }
            }

            if (resultCurve == null)
            {
                RhinoApp.WriteLine("收缩拟合失败（" + report + "）");
                return Result.Failure;
            }

            Transform toWorld = Transform.PlaneToPlane(Plane.WorldXY, plane);
            if (!resultCurve.Transform(toWorld))
            {
                RhinoApp.WriteLine("坐标变换失败");
                return Result.Failure;
            }

            // ========== 8. 写入文档 ==========
            uint undoRecord = doc.BeginUndoRecord("EllipseOuterHull");
            var attributes = new ObjectAttributes { Name = "OuterHull" };
            Guid id = doc.Objects.AddCurve(resultCurve, attributes);
            if (id != Guid.Empty)
                doc.Objects.Select(id);
            doc.EndUndoRecord(undoRecord);
            doc.Views.Redraw();

            string closedText = resultCurve.IsClosed ? "封闭" : "未封闭";
            RhinoApp.WriteLine(string.Format("EllipseOuterHull 完成：输入 {0} 条曲线，{1}，{2}",
                curves.Count, report, closedText));
            return Result.Success;
        }

        // ==================== 基础几何工具 ====================

        /// <summary>凸体在方向 (dx,dy) 上的支撑值 h = d·C + √((d·U)² + (d·V)²)。</summary>
        private static double SupportValue(HullBody b, double dx, double dy)
        {
            double du = dx * b.Ux + dy * b.Uy;
            double dv = dx * b.Vx + dy * b.Vy;
            return dx * b.Cx + dy * b.Cy + Math.Sqrt(du * du + dv * dv);
        }

        /// <summary>
        /// 生成 2D 全椭圆 Nurbs 曲线（参数 t 对应 p(t) = C + U·cos t + V·sin t，域 [0, 2π]）：
        /// 单位圆 Nurbs 经仿射变换 (x→U, y→V, 原点→C) 得到。
        /// </summary>
        private static NurbsCurve MakeFullEllipseNurbs(HullBody b)
        {
            NurbsCurve unit = new Circle(Plane.WorldXY, 1.0).ToNurbsCurve();
            if (unit == null)
                return null;

            // RhinoCommon 行向量约定 p' = p·M：x 基向量映射到 (M00, M10, M20)
            Transform xf = Transform.Identity;
            xf.M00 = b.Ux; xf.M10 = b.Uy;
            xf.M01 = b.Vx; xf.M11 = b.Vy;
            xf.M03 = b.Cx; xf.M13 = b.Cy;
            return unit.Transform(xf) ? unit : null;
        }

        // ==================== loss 采样 ====================

        /// <summary>单位圆盘内拟均匀（sunflower）采样点，缓存复用。</summary>
        private static double[] UnitDiskSamples()
        {
            if (_unitDiskSamples != null)
                return _unitDiskSamples;
            double[] pts = new double[2 * LatticeCount];
            for (int i = 0; i < LatticeCount; i++)
            {
                double r = Math.Sqrt((i + 0.5) / LatticeCount);
                double a = i * GoldenAngle;
                pts[2 * i] = r * Math.Cos(a);
                pts[2 * i + 1] = r * Math.Sin(a);
            }
            _unitDiskSamples = pts;
            return pts;
        }

        /// <summary>
        /// 把每个有面积的凸体转为内部采样点集，并计算每个采样点的重叠重数
        /// （被多少个椭圆包含，空间哈希网格加速）。
        /// 联合面积 = Σ 采样点面积权重 / 重数——重叠部分只计一次。
        /// </summary>
        private static List<SampleBody> BuildSampleBodies(List<HullBody> bodies, out double unionArea)
        {
            unionArea = 0.0;
            double[] disk = UnitDiskSamples();

            var valid = new List<HullBody>(bodies.Count);
            foreach (HullBody b in bodies)
            {
                double area = Math.PI * Math.Abs(b.Ux * b.Vy - b.Uy * b.Vx);
                if (area >= 1e-18)
                    valid.Add(b); // 点/退化体无面积贡献
            }
            if (valid.Count == 0)
                return new List<SampleBody>();

            // 空间哈希：单元格尺寸取最大椭圆外接半展的 2 倍；
            // 每个椭圆按其 AABB 覆盖的单元格登记，查询采样点所在单元格即可找到所有候选椭圆
            double maxHalf = 1e-12;
            foreach (HullBody b in valid)
            {
                double hx = Math.Sqrt(b.Ux * b.Ux + b.Vx * b.Vx);
                double hy = Math.Sqrt(b.Uy * b.Uy + b.Vy * b.Vy);
                if (hx > maxHalf) maxHalf = hx;
                if (hy > maxHalf) maxHalf = hy;
            }
            double cellSize = 2.0 * maxHalf;
            var grid = new Dictionary<long, List<int>>();
            for (int j = 0; j < valid.Count; j++)
            {
                HullBody b = valid[j];
                double hx = Math.Sqrt(b.Ux * b.Ux + b.Vx * b.Vx);
                double hy = Math.Sqrt(b.Uy * b.Uy + b.Vy * b.Vy);
                int cx0 = (int)Math.Floor((b.Cx - hx) / cellSize);
                int cx1 = (int)Math.Floor((b.Cx + hx) / cellSize);
                int cy0 = (int)Math.Floor((b.Cy - hy) / cellSize);
                int cy1 = (int)Math.Floor((b.Cy + hy) / cellSize);
                for (int cx = cx0; cx <= cx1; cx++)
                {
                    for (int cy = cy0; cy <= cy1; cy++)
                    {
                        long key = ((long)cx << 32) | (long)(uint)cy;
                        List<int> cell;
                        if (!grid.TryGetValue(key, out cell))
                        {
                            cell = new List<int>(4);
                            grid.Add(key, cell);
                        }
                        cell.Add(j);
                    }
                }
            }

            var result = new List<SampleBody>(valid.Count);
            for (int i = 0; i < valid.Count; i++)
            {
                HullBody b = valid[i];
                SampleBody sb = new SampleBody
                {
                    X = new double[LatticeCount],
                    Y = new double[LatticeCount],
                    Mult = new double[LatticeCount],
                    Area = Math.PI * Math.Abs(b.Ux * b.Vy - b.Uy * b.Vx)
                };
                sb.WeightPerPoint = sb.Area / LatticeCount;

                for (int m = 0; m < LatticeCount; m++)
                {
                    double x = b.Cx + b.Ux * disk[2 * m] + b.Vx * disk[2 * m + 1];
                    double y = b.Cy + b.Uy * disk[2 * m] + b.Vy * disk[2 * m + 1];
                    sb.X[m] = x;
                    sb.Y[m] = y;

                    // 重数：查询采样点所在单元格中的候选椭圆，逐个做包含测试
                    long key = ((long)Math.Floor(x / cellSize) << 32) | (long)(uint)Math.Floor(y / cellSize);
                    int mult = 0;
                    List<int> cell;
                    if (grid.TryGetValue(key, out cell))
                    {
                        foreach (int j in cell)
                        {
                            HullBody c = valid[j];
                            double dx = x - c.Cx;
                            double dy = y - c.Cy;
                            // 解 p − C = U·s + V·t，s² + t² ≤ 1 即在椭圆内
                            double det = c.Ux * c.Vy - c.Uy * c.Vx;
                            if (Math.Abs(det) > 1e-300)
                            {
                                double s = (c.Vy * dx - c.Vx * dy) / det;
                                double t = (c.Ux * dy - c.Uy * dx) / det;
                                if (s * s + t * t <= 1.0 + 1e-9)
                                    mult++;
                            }
                            else
                            {
                                mult++; // 退化线状体：无面积权重，计数无影响
                            }
                        }
                    }
                    if (mult == 0)
                        mult = 1; // 至少包含自身
                    sb.Mult[m] = mult;

                    unionArea += sb.WeightPerPoint / mult;
                }
                result.Add(sb);
            }
            return result;
        }

        /// <summary>
        /// loss：椭圆联合面积落在半平面交集区域外的部分（采样点按重叠重数 1/m 加权，重叠只计一次）。
        /// 半平面约束：DirX[j]·x + DirY[j]·y ≤ Off[j]。
        /// </summary>
        private static double PolygonLoss(List<SampleBody> samples, double[] dirX, double[] dirY, double[] off)
        {
            double loss = 0.0;
            int k = dirX.Length;
            foreach (SampleBody sb in samples)
            {
                double[] xs = sb.X;
                double[] ys = sb.Y;
                double[] mult = sb.Mult;
                for (int m = 0; m < xs.Length; m++)
                {
                    double px = xs[m];
                    double py = ys[m];
                    for (int j = 0; j < k; j++)
                    {
                        if (dirX[j] * px + dirY[j] * py > off[j])
                        {
                            loss += sb.WeightPerPoint / mult[m];
                            break;
                        }
                    }
                }
            }
            return loss;
        }

        /// <summary>loss（以方向角 + 偏移表示的多边形区域）。</summary>
        private static double PolygonLossFromSides(List<SampleBody> samples, double[] theta, double[] off)
        {
            int k = theta.Length;
            double[] dirX = new double[k];
            double[] dirY = new double[k];
            for (int j = 0; j < k; j++)
            {
                dirX[j] = Math.Cos(theta[j]);
                dirY[j] = Math.Sin(theta[j]);
            }
            return PolygonLoss(samples, dirX, dirY, off);
        }

        /// <summary>loss（第 side 条边偏移临时替换为 newOff，其余不变）。</summary>
        private static double PolygonLossMoved(List<SampleBody> samples, double[] theta, double[] off,
            int side, double newOff)
        {
            int k = theta.Length;
            double[] dirX = new double[k];
            double[] dirY = new double[k];
            double[] off2 = new double[k];
            for (int j = 0; j < k; j++)
            {
                dirX[j] = Math.Cos(theta[j]);
                dirY[j] = Math.Sin(theta[j]);
                off2[j] = off[j];
            }
            off2[side] = newOff;
            return PolygonLoss(samples, dirX, dirY, off2);
        }

        /// <summary>loss：椭圆联合面积落在椭圆区域外的部分（采样点按重叠重数 1/m 加权，重叠只计一次）。</summary>
        private static double EllipseLoss(List<SampleBody> samples, double cx, double cy, double a, double b, double phi)
        {
            double e1x = Math.Cos(phi), e1y = Math.Sin(phi);
            double e2x = -e1y, e2y = e1x;
            double invA = 1.0 / a;
            double invB = 1.0 / b;
            double loss = 0.0;
            foreach (SampleBody sb in samples)
            {
                double[] xs = sb.X;
                double[] ys = sb.Y;
                double[] mult = sb.Mult;
                for (int m = 0; m < xs.Length; m++)
                {
                    double dx = xs[m] - cx;
                    double dy = ys[m] - cy;
                    double u = (dx * e1x + dy * e1y) * invA;
                    double v = (dx * e2x + dy * e2y) * invB;
                    if (u * u + v * v > 1.0)
                        loss += sb.WeightPerPoint / mult[m];
                }
            }
            return loss;
        }

        // ==================== Polygon（矩形）收缩拟合 ====================

        /// <summary>半平面交集多边形的鞋带面积；退化返回 -1。</summary>
        private static double PolygonAreaFromSides(double[] theta, double[] off)
        {
            int k = theta.Length;
            double area2 = 0.0;
            double prevX = 0.0, prevY = 0.0, firstX = 0.0, firstY = 0.0;
            for (int j = 0; j < k; j++)
            {
                int j2 = (j + 1) % k;
                double a1 = Math.Cos(theta[j]), b1 = Math.Sin(theta[j]);
                double a2 = Math.Cos(theta[j2]), b2 = Math.Sin(theta[j2]);
                double det = a1 * b2 - a2 * b1;
                if (Math.Abs(det) < 1e-12)
                    return -1.0;
                double x = (off[j] * b2 - off[j2] * b1) / det;
                double y = (a1 * off[j2] - a2 * off[j]) / det;
                if (j == 0)
                {
                    firstX = x;
                    firstY = y;
                }
                else
                {
                    area2 += prevX * y - x * prevY;
                }
                prevX = x;
                prevY = y;
            }
            area2 += prevX * firstY - firstX * prevY;
            return Math.Abs(area2) * 0.5;
        }

        /// <summary>半平面交集多边形转封闭曲线（顶点顺序连线）。</summary>
        private static Curve BuildPolygonCurve(double[] theta, double[] off)
        {
            int k = theta.Length;
            var pts = new List<Point3d>(k + 1);
            for (int j = 0; j < k; j++)
            {
                int j2 = (j + 1) % k;
                double a1 = Math.Cos(theta[j]), b1 = Math.Sin(theta[j]);
                double a2 = Math.Cos(theta[j2]), b2 = Math.Sin(theta[j2]);
                double det = a1 * b2 - a2 * b1;
                if (Math.Abs(det) < 1e-12)
                    return null;
                double x = (off[j] * b2 - off[j2] * b1) / det;
                double y = (a1 * off[j2] - a2 * off[j]) / det;
                pts.Add(new Point3d(x, y, 0.0));
            }
            pts.Add(pts[0]);
            return new PolylineCurve(pts);
        }

        /// <summary>采样点（按联合面积权重 1/重数）的加权 PCA 主轴方向角。</summary>
        private static double PrincipalAngle(List<SampleBody> samples)
        {
            double totalW = 0.0, sx = 0.0, sy = 0.0;
            foreach (SampleBody sb in samples)
            {
                for (int m = 0; m < sb.X.Length; m++)
                {
                    double w = sb.WeightPerPoint / sb.Mult[m];
                    totalW += w;
                    sx += w * sb.X[m];
                    sy += w * sb.Y[m];
                }
            }
            if (totalW < 1e-18)
                return 0.0;
            double cx = sx / totalW;
            double cy = sy / totalW;

            double cxx = 0.0, cxy = 0.0, cyy = 0.0;
            foreach (SampleBody sb in samples)
            {
                for (int m = 0; m < sb.X.Length; m++)
                {
                    double w = sb.WeightPerPoint / sb.Mult[m];
                    double dx = sb.X[m] - cx;
                    double dy = sb.Y[m] - cy;
                    cxx += w * dx * dx;
                    cxy += w * dx * dy;
                    cyy += w * dy * dy;
                }
            }
            cxx /= totalW;
            cxy /= totalW;
            cyy /= totalW;
            return 0.5 * Math.Atan2(2.0 * cxy, cxx - cyy);
        }

        /// <summary>
        /// 矩形收缩拟合：以包住凸包的 PCA 对齐外接矩形为初始（loss = 0）。
        /// 形状恒为真矩形——四边法向固定成对正交（phi, phi+90°, phi+180°, phi+270°），
        /// 四角始终为直角；只有整体旋转/平移和沿法向内移，不存在逐边独立旋转。
        /// 逐边内移采用比例分步（每轮只收该边理想增量的 40%）并多轮迭代至收敛：
        /// 椭圆密度小（支撑松弛大、边际 loss 低）的一侧自然获得更多收缩，
        /// 密度大的一侧几乎不动——即收缩由密度小的地方开始。
        /// 每步以 "loss ≤ budget" 为约束、以矩形面积减小为目标。
        /// 返回各边方向角 theta 与偏移 off。
        /// </summary>
        private static void OptimizeShrunkRectangle(List<HullBody> bodies, List<SampleBody> samples,
            double budget, double[] supportH, out double[] resultTheta, out double[] resultOff, out double initialArea)
        {
            // 场景尺度（收敛阈值与移动量级参考）
            double scale = 0.0;
            foreach (HullBody b in bodies)
                scale = Math.Max(scale, Math.Abs(b.Cx) + Math.Abs(b.Cy)
                    + Math.Abs(b.Ux) + Math.Abs(b.Uy) + Math.Abs(b.Vx) + Math.Abs(b.Vy));
            if (scale < 1e-12)
                scale = 1.0;
            double eps = 1e-6 * scale;
            double areaEps = eps * eps;

            // 初始：PCA 主轴对齐的外接矩形（四边法向成对正交，四角直角）
            double phi = PrincipalAngle(samples);
            double[] theta = new double[4];
            double[] off = new double[4];
            for (int j = 0; j < 4; j++)
            {
                theta[j] = phi + j * Math.PI / 2.0;
                int idx = (int)Math.Round(theta[j] / TwoPi * supportH.Length) % supportH.Length;
                if (idx < 0)
                    idx += supportH.Length;
                off[j] = supportH[idx];
            }

            initialArea = PolygonAreaFromSides(theta, off);
            double lossEps = 1e-9 * Math.Max(initialArea, 1e-300);
            double gamma = 0.15 * (Math.PI / 2.0);   // 整体旋转步长

            for (int sweep = 0; sweep < 150; sweep++)
            {
                bool improved = false;

                // ---- 逐边内移（比例分步，多轮收敛：密度小的一侧收得多）----
                for (int j = 0; j < 4; j++)
                {
                    double dirX = Math.Cos(theta[j]);
                    double dirY = Math.Sin(theta[j]);
                    double deepest = double.MaxValue;
                    foreach (HullBody b in bodies)
                    {
                        double du = dirX * b.Ux + dirY * b.Uy;
                        double dv = dirX * b.Vx + dirY * b.Vy;
                        double low = dirX * b.Cx + dirY * b.Cy - Math.Sqrt(du * du + dv * dv);
                        if (low < deepest)
                            deepest = low;
                    }
                    double deltaMax = off[j] - deepest;
                    if (deltaMax > eps)
                    {
                        double ideal;
                        if (PolygonLossMoved(samples, theta, off, j, off[j] - deltaMax) <= budget)
                        {
                            ideal = deltaMax;
                        }
                        else
                        {
                            double lo = 0.0;
                            double hi = deltaMax;
                            for (int it = 0; it < 30; it++)
                            {
                                double mid = 0.5 * (lo + hi);
                                if (PolygonLossMoved(samples, theta, off, j, off[j] - mid) <= budget)
                                    lo = mid;
                                else
                                    hi = mid;
                            }
                            ideal = lo;
                        }
                        double step = ideal * 0.4;   // 每轮只收 40%，预算在四边间均衡分配
                        if (step > eps)
                        {
                            off[j] -= step;
                            improved = true;
                        }
                    }
                }

                // ---- 整体同步旋转（保持四角直角）----
                double curArea = PolygonAreaFromSides(theta, off);
                for (int sign = -1; sign <= 1; sign += 2)
                {
                    double[] trialTheta = (double[])theta.Clone();
                    for (int j = 0; j < 4; j++)
                        trialTheta[j] += sign * gamma;
                    double trialArea = PolygonAreaFromSides(trialTheta, off);
                    if (trialArea >= 0.0 && trialArea < curArea - areaEps
                        && PolygonLossFromSides(samples, trialTheta, off) <= budget)
                    {
                        theta = trialTheta;
                        curArea = trialArea;
                        improved = true;
                    }
                }

                // ---- 整体平移（面积不变，loss 减小才接受——向簇质心靠拢后可继续收缩）----
                double stepT = 0.05 * Math.Min(
                    0.5 * (off[0] + off[2]),
                    0.5 * (off[1] + off[3]));
                if (stepT > eps)
                {
                    for (int axis = 0; axis < 2; axis++)
                    {
                        for (int sign = -1; sign <= 1; sign += 2)
                        {
                            double[] trialOff = (double[])off.Clone();
                            trialOff[axis] += sign * stepT;
                            trialOff[axis + 2] -= sign * stepT;
                            if (PolygonLossFromSides(samples, theta, trialOff)
                                < PolygonLossFromSides(samples, theta, off) - lossEps)
                            {
                                off = trialOff;
                                improved = true;
                            }
                        }
                    }
                }

                if (!improved)
                {
                    gamma *= 0.5;
                    if (gamma < 1e-6)
                        break;
                }
            }

            resultTheta = theta;
            resultOff = off;
        }

        // ==================== Ellipse 收缩拟合 ====================

        /// <summary>
        /// Ellipse 收缩拟合：初始为加权质心 + 协方差主轴、放大到包含全部采样点的外接椭圆
        /// （loss = 0），随后 pattern-search 缩轴 / 移心 / 转角，
        /// 每步以 "loss ≤ budget" 为约束、以椭圆面积 πAB 减小为目标。
        /// </summary>
        private static EllipseFit OptimizeShrunkEllipse(List<SampleBody> samples, double budget, out double initialArea)
        {
            initialArea = 0.0;
            // 加权质心与协方差
            double totalW = 0.0, sx = 0.0, sy = 0.0;
            foreach (SampleBody sb in samples)
            {
                double w = sb.WeightPerPoint;
                for (int m = 0; m < sb.X.Length; m++)
                {
                    totalW += w;
                    sx += w * sb.X[m];
                    sy += w * sb.Y[m];
                }
            }
            if (totalW < 1e-18)
                return null;
            double cx0 = sx / totalW;
            double cy0 = sy / totalW;

            double cxx = 0.0, cxy = 0.0, cyy = 0.0;
            foreach (SampleBody sb in samples)
            {
                double w = sb.WeightPerPoint;
                for (int m = 0; m < sb.X.Length; m++)
                {
                    double dx = sb.X[m] - cx0;
                    double dy = sb.Y[m] - cy0;
                    cxx += w * dx * dx;
                    cxy += w * dx * dy;
                    cyy += w * dy * dy;
                }
            }
            cxx /= totalW;
            cxy /= totalW;
            cyy /= totalW;

            // 2×2 对称矩阵特征分解（稳定形式）
            double mid = 0.5 * (cxx + cyy);
            double diff = 0.5 * (cxx - cyy);
            double root = Math.Sqrt(diff * diff + cxy * cxy);
            double lambda1 = mid + root;
            double lambda2 = mid - root;
            if (lambda1 <= 1e-300 || lambda2 <= 1e-300)
                return null;

            double e1x, e1y;
            if (cxx >= cyy) { e1x = lambda1 - cyy; e1y = cxy; }
            else { e1x = cxy; e1y = lambda1 - cxx; }
            double e1Len = Math.Sqrt(e1x * e1x + e1y * e1y);
            if (e1Len < 1e-300)
            {
                e1x = 1.0; e1y = 0.0;
                e1Len = 1.0;
            }
            e1x /= e1Len;
            e1y /= e1Len;
            double sigma1 = Math.Sqrt(lambda1);
            double sigma2 = Math.Sqrt(lambda2);

            // 放大到包含全部采样点（初始外接椭圆，loss = 0）
            double containScale = 0.0;
            foreach (SampleBody sb in samples)
            {
                for (int m = 0; m < sb.X.Length; m++)
                {
                    double dx = sb.X[m] - cx0;
                    double dy = sb.Y[m] - cy0;
                    double u = (dx * e1x + dy * e1y) / sigma1;
                    double v = (dx * (-e1y) + dy * e1x) / sigma2;
                    double r2 = u * u + v * v;
                    if (r2 > containScale)
                        containScale = r2;
                }
            }
            containScale = Math.Sqrt(containScale) + 1e-12;

            EllipseFit fit = new EllipseFit();
            fit.Cx = cx0;
            fit.Cy = cy0;
            fit.A = sigma1 * containScale;
            fit.B = Math.Max(sigma2 * containScale, 1e-9);
            fit.Phi = Math.Atan2(e1y, e1x);
            initialArea = Math.PI * fit.A * fit.B;

            // pattern search
            double stepRel = 0.12;
            double eps = 1e-9 * (fit.A + fit.B);
            int sweeps = 0;
            while (stepRel > 0.004 && sweeps < 1000)
            {
                sweeps++;
                bool accepted = false;
                double rotStep = 0.05 * stepRel;

                // 缩 A
                EllipseFit trial = CloneFit(fit);
                trial.A *= (1.0 - stepRel);
                if (TrialAcceptable(samples, trial, fit, budget, eps, false)) { fit = trial; accepted = true; }

                // 缩 B
                if (!accepted)
                {
                    trial = CloneFit(fit);
                    trial.B *= (1.0 - stepRel);
                    if (TrialAcceptable(samples, trial, fit, budget, eps, false)) { fit = trial; accepted = true; }
                }

                // 移心 ±（平移不改变面积，以 loss 减小为接受条件——居中后才能继续缩轴）
                if (!accepted)
                {
                    for (int dir = 0; dir < 4 && !accepted; dir++)
                    {
                        trial = CloneFit(fit);
                        if (dir == 0) trial.Cx += stepRel * fit.A;
                        else if (dir == 1) trial.Cx -= stepRel * fit.A;
                        else if (dir == 2) trial.Cy += stepRel * fit.B;
                        else trial.Cy -= stepRel * fit.B;
                        if (TrialAcceptable(samples, trial, fit, budget, eps, true)) { fit = trial; accepted = true; }
                    }
                }

                // 转角 ±
                if (!accepted)
                {
                    for (int sign = -1; sign <= 1 && !accepted; sign += 2)
                    {
                        trial = CloneFit(fit);
                        trial.Phi += sign * rotStep;
                        if (TrialAcceptable(samples, trial, fit, budget, eps, false)) { fit = trial; accepted = true; }
                    }
                }

                if (!accepted)
                    stepRel *= 0.6;
            }

            return fit;
        }

        /// <summary>
        /// 试验解是否可接受：缩轴/转角要求面积减小且 loss 合规；
        /// 移心（centerMove）不改变面积，要求 loss 严格减小（居中后才能继续缩轴）。
        /// </summary>
        private static bool TrialAcceptable(List<SampleBody> samples, EllipseFit trial, EllipseFit current,
            double budget, double eps, bool centerMove)
        {
            if (trial.A <= eps || trial.B <= eps)
                return false;

            if (centerMove)
            {
                double lossEps = 1e-9 * current.A * current.B;
                return EllipseLoss(samples, trial.Cx, trial.Cy, trial.A, trial.B, trial.Phi)
                    < EllipseLoss(samples, current.Cx, current.Cy, current.A, current.B, current.Phi) - lossEps;
            }

            double trialArea = Math.PI * trial.A * trial.B;
            double currentArea = Math.PI * current.A * current.B;
            if (trialArea >= currentArea - eps)
                return false;
            return EllipseLoss(samples, trial.Cx, trial.Cy, trial.A, trial.B, trial.Phi) <= budget;
        }

        private static EllipseFit CloneFit(EllipseFit f)
        {
            return new EllipseFit { Cx = f.Cx, Cy = f.Cy, A = f.A, B = f.B, Phi = f.Phi };
        }

        // ==================== 模式封装 ====================

        /// <summary>Polygon 模式：矩形（4 边）收缩拟合，返回 2D 曲线与统计报告。</summary>
        private static Curve FitShrunkPolygon(List<HullBody> bodies, List<SampleBody> samples, double budget,
            double unionArea, double lossTolerance, double[] supportH,
            out double finalArea, out double finalLoss, out string report)
        {
            finalArea = 0.0;
            finalLoss = 0.0;
            report = null;

            double initialArea;
            double[] theta, off;
            OptimizeShrunkRectangle(bodies, samples, budget, supportH, out theta, out off, out initialArea);

            finalArea = PolygonAreaFromSides(theta, off);
            if (finalArea < 0.0)
            {
                report = "矩形区域退化";
                return null;
            }

            Curve curve = BuildPolygonCurve(theta, off);
            if (curve == null)
            {
                report = "矩形曲线生成失败";
                return null;
            }

            finalLoss = PolygonLossFromSides(samples, theta, off);
            report = string.Format(
                "矩形外接面积 {0:F4} → {1:F4}（缩减 {2:P1}），联合面积损失 {3:F4}（{4:P1}，上限 {5:P1}）",
                initialArea, finalArea,
                initialArea > 1e-12 ? 1.0 - finalArea / initialArea : 0.0,
                finalLoss, unionArea > 1e-12 ? finalLoss / unionArea : 0.0, lossTolerance);
            return curve;
        }

        /// <summary>Ellipse 模式：单个椭圆收缩拟合，返回 2D 曲线与统计报告。</summary>
        private static Curve FitShrunkEllipse(List<SampleBody> samples, double budget,
            double unionArea, double lossTolerance,
            out double finalArea, out double finalLoss, out string report)
        {
            finalArea = 0.0;
            finalLoss = 0.0;
            report = null;

            double initialArea;
            EllipseFit fit = OptimizeShrunkEllipse(samples, budget, out initialArea);
            if (fit == null)
            {
                report = "无法拟合外接椭圆（采样点分布退化）";
                return null;
            }

            HullBody eb = new HullBody();
            eb.Cx = fit.Cx;
            eb.Cy = fit.Cy;
            eb.Ux = fit.A * Math.Cos(fit.Phi);
            eb.Uy = fit.A * Math.Sin(fit.Phi);
            eb.Vx = -fit.B * Math.Sin(fit.Phi);
            eb.Vy = fit.B * Math.Cos(fit.Phi);
            NurbsCurve curve = MakeFullEllipseNurbs(eb);
            if (curve == null)
            {
                report = "生成椭圆曲线失败";
                return null;
            }

            finalArea = Math.PI * fit.A * fit.B;
            finalLoss = EllipseLoss(samples, fit.Cx, fit.Cy, fit.A, fit.B, fit.Phi);
            report = string.Format(
                "外接椭圆面积 {0:F4} → {1:F4}（缩减 {2:P1}），联合面积损失 {3:F4}（{4:P1}，上限 {5:P1}）",
                initialArea, finalArea,
                initialArea > 1e-12 ? 1.0 - finalArea / initialArea : 0.0,
                finalLoss, unionArea > 1e-12 ? finalLoss / unionArea : 0.0, lossTolerance);
            return curve;
        }
    }
}
