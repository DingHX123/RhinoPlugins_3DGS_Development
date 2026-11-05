using Rhino;
using Rhino.Commands;
using Rhino.Display;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading.Tasks;

namespace RhinoPlugins_Development
{
    /// <summary>
    /// SectionSplat：两种剖切方式。
    /// 1) 选取平面：选取已有平面或三点绘制一个平面（原操作不变）；
    /// 2) 正交取样（OrthoSample）：设置 X/Y/Z 三个方向的数量，在模型包围盒内
    ///    自动均匀布置剖面，逐一与 splat 椭球求交。
    /// 解析计算平面与每个 splat 椭球的相交椭圆（等值面与栅格化渲染可见尺寸一致），
    /// 相交线颜色与该椭球的渲染颜色保持一致。
    /// </summary>
    public class SectionSplatCommand : Command
    {
        /// <summary>Sigma=0（自动模式）时按栅格化可见边缘求交：片元 alpha 衰减到该阈值视为不可见。</summary>
        private const double VisibleAlphaThreshold = 0.05;

        /// <summary>自动模式下单轴 σ 系数上限（与渲染 quad 的 3σ 覆盖一致）。</summary>
        private const double MaxSigmaLevel = 3.0;

        private const string SectionLayerName = "SplatSections";

        public override string EnglishName => "SectionSplat";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            RhinoApp.WriteLine("SectionSplat v3：数据源 = 渲染管道 SplatInstance.Splats（与光栅渲染 GPU 上传同一数组）");

            // 渲染一致性：渲染的 splat 尺寸 = σ × 全局 splat 缩放（着色器 u_SplatScale）
            float splatScale = SplatDisplayConduit.SplatScale;

            // 渲染一致性：收集当前视口的裁剪平面（着色器按中心点丢弃其后的 splat）
            List<Plane> clipPlanes = CollectActiveViewportClipPlanes(doc);

            // ========== 1. 剖切方式：选取/绘制平面（原操作不变）或 OrthoSample 正交取样 ==========
            var go = new GetObject();
            go.SetCommandPrompt("选取剖切平面（平面曲面/多重曲面），按 Enter 三点绘制，或点击 OrthoSample 正交取样");
            go.GeometryFilter = ObjectType.Surface | ObjectType.Brep | ObjectType.Extrusion;
            go.AcceptNothing(true);
            int orthoSampleIndex = go.AddOption("OrthoSample");

            GetResult planeResult = go.Get();
            if (planeResult == GetResult.Cancel)
                return Result.Cancel;

            bool orthoMode = planeResult == GetResult.Option
                && go.Option() != null
                && go.Option().Index == orthoSampleIndex;

            // ========== 2. 选取平面模式：获取剖切平面 ==========
            Plane sectionPlane = Plane.Unset;
            if (!orthoMode)
            {
                if (planeResult == GetResult.Object)
                {
                    GeometryBase geometry = go.Object(0).Geometry();
                    if (!TryGetPlaneFromGeometry(geometry, out sectionPlane))
                    {
                        RhinoApp.WriteLine("所选对象不是平面，请选择平面曲面或平面多重曲面");
                        return Result.Cancel;
                    }
                }
                else if (planeResult == GetResult.Nothing)
                {
                    if (!TryGetThreePointPlane(out sectionPlane))
                        return Result.Cancel;
                }
                else
                {
                    return Result.Cancel;
                }
            }

            // ========== 3. 选择 3dgs 模型 ==========
            SplatInstance targetInstance = SplatDisplayConduit.GetTargetInstance(
                doc, "选择 3dgs 模型（代理点云）");
            if (targetInstance == null)
            {
                RhinoApp.WriteLine("未选择 3dgs 模型，请先运行 ImportGaussianSplat 导入并显示模型");
                return Result.Failure;
            }

            Splat[] splats = targetInstance.Splats;
            if (splats == null || splats.Length == 0)
            {
                RhinoApp.WriteLine("所选模型没有 splat 数据");
                return Result.Failure;
            }

            // ========== 4. 过滤参数（正交取样模式追加三个方向的数量） ==========
            double minOpacity = 0.1;
            double minRadius = 0.0;
            double sigma = 0.0; // 0 = 自动：按栅格化可见边缘（α 等值面）逐 splat 求交
            double maxAspect = 100.0;
            int xCount = 5, yCount = 5, zCount = 5;

            OptionDouble opacityOption = new OptionDouble(minOpacity, 0.0, 1.0);
            OptionDouble radiusOption = new OptionDouble(minRadius, 0.0, double.MaxValue);
            OptionDouble sigmaOption = new OptionDouble(sigma, 0.0, 5.0);
            OptionDouble aspectOption = new OptionDouble(maxAspect, 0.0, double.MaxValue);
            OptionInteger xCountOption = new OptionInteger(xCount, 1, 500);
            OptionInteger yCountOption = new OptionInteger(yCount, 1, 500);
            OptionInteger zCountOption = new OptionInteger(zCount, 1, 500);

            var getOption = new GetOption();
            getOption.SetCommandPrompt(orthoMode
                ? "设置正交取样与过滤参数，按 Enter 继续"
                : "设置过滤参数，按 Enter 继续");
            getOption.AcceptNothing(true);
            if (orthoMode)
            {
                getOption.AddOptionInteger("XCount", ref xCountOption, "包围盒 X 方向剖面数量");
                getOption.AddOptionInteger("YCount", ref yCountOption, "包围盒 Y 方向剖面数量");
                getOption.AddOptionInteger("ZCount", ref zCountOption, "包围盒 Z 方向剖面数量");
            }
            getOption.AddOptionDouble("MinOpacity", ref opacityOption, "不透明度低于该值的 splat 不绘制");
            getOption.AddOptionDouble("MinRadius", ref radiusOption, "长半轴小于该值(模型单位)的椭圆不绘制");
            getOption.AddOptionDouble("Sigma", ref sigmaOption, "0=自动匹配栅格化可见边缘, >0=固定σ系数(3=渲染椭球等值面)");
            getOption.AddOptionDouble("MaxAspect", ref aspectOption, "长短半轴比超过该值的针状退化椭圆不绘制, 0=不限制");

            GetResult optionResult;
            do
            {
                optionResult = getOption.Get();
                if (optionResult == GetResult.Cancel)
                    return Result.Cancel;
            }
            while (optionResult == GetResult.Option);

            minOpacity = opacityOption.CurrentValue;
            minRadius = radiusOption.CurrentValue;
            sigma = sigmaOption.CurrentValue;
            maxAspect = aspectOption.CurrentValue;
            if (orthoMode)
            {
                xCount = xCountOption.CurrentValue;
                yCount = yCountOption.CurrentValue;
                zCount = zCountOption.CurrentValue;
            }

            if (Math.Abs(splatScale - 1.0) > 1e-4)
                RhinoApp.WriteLine($"已应用全局 splat 缩放 SplatScale = {splatScale:0.###}（与渲染一致）");
            if (clipPlanes.Count > 0)
                RhinoApp.WriteLine($"已排除当前视口 {clipPlanes.Count} 个裁剪平面后方的隐藏 splat（与渲染一致）");
            if (sigma <= 0.0)
                RhinoApp.WriteLine($"Sigma=自动：按每个 splat 的栅格化可见边缘求交（alpha 衰减到 {VisibleAlphaThreshold:0.##} 的等值面）");

            // ========== 5. 计算平面-椭球相交椭圆 ==========
            var allSections = new List<SectionEllipse>(4096);
            long totalCandidates = 0;
            int totalPlanes = 0;

            if (orthoMode)
            {
                // 由 splat 中心点计算模型包围盒
                double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue;
                double maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;
                for (int i = 0; i < splats.Length; i++)
                {
                    Splat s = splats[i];
                    if (s.X < minX) minX = s.X;
                    if (s.X > maxX) maxX = s.X;
                    if (s.Y < minY) minY = s.Y;
                    if (s.Y > maxY) maxY = s.Y;
                    if (s.Z < minZ) minZ = s.Z;
                    if (s.Z > maxZ) maxZ = s.Z;
                }
                var bboxCenter = new Point3d(0.5 * (minX + maxX), 0.5 * (minY + maxY), 0.5 * (minZ + maxZ));

                RhinoApp.WriteLine($"模型包围盒：X[{minX:F3},{maxX:F3}] Y[{minY:F3},{maxY:F3}] Z[{minZ:F3},{maxZ:F3}]");

                totalCandidates += SampleOrthoAxis(splats, Vector3d.XAxis, minX, maxX, bboxCenter, xCount,
                    minOpacity, minRadius, sigma, splatScale, clipPlanes, maxAspect, allSections);
                totalCandidates += SampleOrthoAxis(splats, Vector3d.YAxis, minY, maxY, bboxCenter, yCount,
                    minOpacity, minRadius, sigma, splatScale, clipPlanes, maxAspect, allSections);
                totalCandidates += SampleOrthoAxis(splats, Vector3d.ZAxis, minZ, maxZ, bboxCenter, zCount,
                    minOpacity, minRadius, sigma, splatScale, clipPlanes, maxAspect, allSections);
                totalPlanes = xCount + yCount + zCount;
            }
            else
            {
                totalCandidates += ComputeSectionsForPlane(splats, sectionPlane,
                    minOpacity, minRadius, sigma, splatScale, clipPlanes, maxAspect, allSections);
                totalPlanes = 1;
            }

            if (totalCandidates > 500000)
                RhinoApp.WriteLine($"候选 splat 较多（{totalCandidates:N0}），如生成过慢可提高 MinOpacity 或 MinRadius");

            if (allSections.Count == 0)
            {
                RhinoApp.WriteLine("没有生成相交线：平面附近没有满足条件的 splat（可尝试降低 MinOpacity 或提高剖面数量）");
                return Result.Success;
            }

            // ========== 6. 添加曲线（统一图层，颜色取椭球颜色） ==========
            int layerIndex = doc.Layers.Find(SectionLayerName, true);
            if (layerIndex < 0)
            {
                var layer = new Layer { Name = SectionLayerName, Color = Color.LightGray };
                layerIndex = doc.Layers.Add(layer);
            }

            uint undoRecord = doc.BeginUndoRecord("SectionSplat");
            int addedCount = 0;
            int skippedCount = 0;

            foreach (SectionEllipse ellipse in allSections)
            {
                var ellipsePlane = new Plane(ellipse.Center, ellipse.Dir1, ellipse.Dir2);
                var ellipseCurve = new Ellipse(ellipsePlane, ellipse.Radius1, ellipse.Radius2);
                NurbsCurve curve = ellipseCurve.ToNurbsCurve();
                if (curve == null)
                {
                    skippedCount++;
                    continue;
                }

                var attributes = new ObjectAttributes
                {
                    LayerIndex = layerIndex,
                    ObjectColor = ellipse.Color,
                    ColorSource = ObjectColorSource.ColorFromObject,
                    Name = "SplatSection"
                };

                if (doc.Objects.AddCurve(curve, attributes) != Guid.Empty)
                    addedCount++;
                else
                    skippedCount++;
            }

            doc.EndUndoRecord(undoRecord);
            doc.Views.Redraw();

            long skippedTotal = totalCandidates - addedCount + skippedCount;
            string modeText = orthoMode
                ? $"正交取样 X/Y/Z = {xCount}/{yCount}/{zCount}（{totalPlanes} 个剖面）"
                : "选取平面模式";
            RhinoApp.WriteLine($"SectionSplat 完成：{modeText}，候选 {totalCandidates:N0}，绘制 {addedCount:N0} 条相交线，跳过 {skippedTotal:N0} 个（曲线位于图层 {SectionLayerName}）");
            return Result.Success;
        }

        /// <summary>收集当前活动视口生效的裁剪平面（与渲染着色器同一剔除规则）。</summary>
        private static List<Plane> CollectActiveViewportClipPlanes(RhinoDoc doc)
        {
            var clipPlanes = new List<Plane>();
            RhinoView activeView = doc.Views.ActiveView;
            RhinoViewport activeViewport = activeView != null ? activeView.ActiveViewport : null;
            if (activeViewport == null)
                return clipPlanes;

            Guid activeViewportId = activeViewport.Id;
            RhinoObject[] clipObjects = doc.Objects.FindByObjectType(ObjectType.ClipPlane);
            if (clipObjects == null)
                return clipPlanes;

            foreach (RhinoObject clipObj in clipObjects)
            {
                var clipPlaneObj = clipObj as ClippingPlaneObject;
                if (clipPlaneObj == null || clipObj.IsDeleted)
                    continue;
                ClippingPlaneSurface surface = clipPlaneObj.ClippingPlaneGeometry;
                if (surface == null)
                    continue;
                foreach (Guid vpId in surface.ViewportIds())
                {
                    if (vpId == activeViewportId)
                    {
                        clipPlanes.Add(surface.Plane);
                        break;
                    }
                }
            }

            return clipPlanes;
        }

        /// <summary>用三个点绘制平面（带橡皮筋线引导）。</summary>
        private static bool TryGetThreePointPlane(out Plane plane)
        {
            var gp = new GetPoint();

            gp.SetCommandPrompt("平面上第一点");
            if (gp.Get() != GetResult.Point)
            {
                plane = Plane.Unset;
                return false;
            }
            Point3d p1 = gp.Point();

            gp.SetCommandPrompt("平面上第二点");
            gp.SetBasePoint(p1, false);
            gp.DrawLineFromPoint(p1, true);
            if (gp.Get() != GetResult.Point)
            {
                plane = Plane.Unset;
                return false;
            }
            Point3d p2 = gp.Point();

            gp.SetCommandPrompt("平面上第三点");
            gp.SetBasePoint(p2, false);
            gp.DrawLineFromPoint(p2, true);
            if (gp.Get() != GetResult.Point)
            {
                plane = Plane.Unset;
                return false;
            }
            Point3d p3 = gp.Point();

            plane = new Plane(p1, p2, p3);
            if (!plane.IsValid)
            {
                RhinoApp.WriteLine("三点共线或重合，无法确定平面");
                return false;
            }
            return true;
        }

        /// <summary>从曲面 / 多重曲面 / 挤出体中提取平面（要求对象为平面）。</summary>
        private static bool TryGetPlaneFromGeometry(GeometryBase geometry, out Plane plane)
        {
            plane = Plane.Unset;
            if (geometry == null)
                return false;

            Surface surface = geometry as Surface;
            if (surface != null)
                return surface.TryGetPlane(out plane);

            Brep brep = geometry as Brep;
            if (brep != null)
            {
                foreach (BrepFace face in brep.Faces)
                {
                    if (face.TryGetPlane(out plane))
                        return true;
                }
                return false;
            }

            Extrusion extrusion = geometry as Extrusion;
            if (extrusion != null)
            {
                Brep extrusionBrep = extrusion.ToBrep();
                if (extrusionBrep != null)
                {
                    foreach (BrepFace face in extrusionBrep.Faces)
                    {
                        if (face.TryGetPlane(out plane))
                            return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// 正交取样一个坐标轴方向：在 [minPos, maxPos] 内部均匀布置 count 个剖面
        /// （不含包围盒边界面），逐面与椭球求交并把结果追加到 output。
        /// 返回该方向所有剖面的候选 splat 总数。
        /// </summary>
        private static long SampleOrthoAxis(Splat[] splats, Vector3d axisNormal, double minPos, double maxPos,
            Point3d bboxCenter, int count, double minOpacity, double minRadius, double sigma, float splatScale,
            List<Plane> clipPlanes, double maxAspect, List<SectionEllipse> output)
        {
            double size = maxPos - minPos;
            if (count <= 0 || size <= 1e-12)
            {
                RhinoApp.WriteLine($"警告：{axisNormal} 方向包围盒厚度为 0 或数量无效，跳过该方向");
                return 0;
            }

            double centerProj = axisNormal.X * bboxCenter.X + axisNormal.Y * bboxCenter.Y + axisNormal.Z * bboxCenter.Z;
            long totalCandidates = 0;

            for (int i = 0; i < count; i++)
            {
                double pos = minPos + (i + 1) * size / (count + 1);
                Point3d origin = bboxCenter + axisNormal * (pos - centerProj);
                var plane = new Plane(origin, axisNormal);

                totalCandidates += ComputeSectionsForPlane(splats, plane,
                    minOpacity, minRadius, sigma, splatScale, clipPlanes, maxAspect, output);
            }

            return totalCandidates;
        }

        /// <summary>
        /// 单个剖面的完整计算：预筛候选 splat（不透明度 + 裁剪平面 + 到平面距离上界），
        /// 并行解析求交，把有效椭圆追加到 output。返回该剖面的候选 splat 数量。
        /// </summary>
        private static long ComputeSectionsForPlane(Splat[] splats, Plane sectionPlane,
            double minOpacity, double minRadius, double sigma, float splatScale,
            List<Plane> clipPlanes, double maxAspect, List<SectionEllipse> output)
        {
            Vector3d planeNormal = sectionPlane.Normal;
            planeNormal.Unitize();
            Point3d planeOrigin = sectionPlane.Origin;

            // 自动模式下可见边缘最多到 3σ，预筛按上限 3σ 展开
            double reachSigma = sigma > 0.0 ? sigma : MaxSigmaLevel;

            var candidates = new List<int>(1024);
            for (int i = 0; i < splats.Length; i++)
            {
                Splat s = splats[i];
                if (s.A / 255.0 < minOpacity)
                    continue;

                // 与渲染着色器一致：中心点在裁剪平面后方的 splat 被丢弃
                bool hiddenByClip = false;
                foreach (Plane clipPlane in clipPlanes)
                {
                    Vector3d cn = clipPlane.Normal;
                    double clipSigned = (s.X - clipPlane.Origin.X) * cn.X
                        + (s.Y - clipPlane.Origin.Y) * cn.Y
                        + (s.Z - clipPlane.Origin.Z) * cn.Z;
                    if (clipSigned < 0.0)
                    {
                        hiddenByClip = true;
                        break;
                    }
                }
                if (hiddenByClip)
                    continue;

                double maxScale = Math.Max(s.ScaleX, Math.Max(s.ScaleY, s.ScaleZ)) * splatScale;
                if (maxScale <= 1e-12)
                    continue;

                // 到平面距离超过 reachSigma×σ 最大半轴的椭球必然不相交，直接排除
                double dx = s.X - planeOrigin.X;
                double dy = s.Y - planeOrigin.Y;
                double dz = s.Z - planeOrigin.Z;
                double signedDistance = dx * planeNormal.X + dy * planeNormal.Y + dz * planeNormal.Z;
                if (Math.Abs(signedDistance) > reachSigma * maxScale)
                    continue;

                candidates.Add(i);
            }

            if (candidates.Count == 0)
                return 0;

            SectionEllipse[] results = new SectionEllipse[candidates.Count];
            Parallel.For(0, candidates.Count, k =>
            {
                results[k] = ComputeSectionEllipse(splats[candidates[k]], planeOrigin, planeNormal,
                    sigma, splatScale, minRadius, maxAspect);
            });

            foreach (SectionEllipse ellipse in results)
            {
                if (ellipse.Valid)
                    output.Add(ellipse);
            }

            return candidates.Count;
        }

        /// <summary>
        /// 解析计算平面与单个 splat 椭球的相交椭圆，等值面与栅格化渲染的可见尺寸一致：
        /// sigmaMultiplier > 0 时为固定 σ×splatScale×sigma 等值面；
        /// sigmaMultiplier ≤ 0 时为自动模式，按片元 alpha 衰减到 VisibleAlphaThreshold 的
        /// 可见边缘 σ_vis = min(3, sqrt(2·ln(opacity/阈值))) 逐 splat 确定，渲染中不可见的 splat 不绘制。
        /// 椭球：(p-C)ᵀ A (p-C) = 1，A = R·diag(1/s²)·Rᵀ；在平面参数坐标系 (u,v) 下
        /// 化为二次曲线 a·u² + 2b·uv + c·v² + 2g·u + 2h·v + f = 0，
        /// 平移到中心后由 2×2 对称矩阵特征分解得到长短轴与方向。
        /// </summary>
        private static SectionEllipse ComputeSectionEllipse(Splat s, Point3d planeOrigin, Vector3d planeNormal,
            double sigmaMultiplier, float splatScale, double minRadius, double maxAspect)
        {
            var result = new SectionEllipse { Valid = false };

            // 确定等值面 σ 系数：固定值或按栅格化可见边缘自动计算
            double sigmaLevel;
            if (sigmaMultiplier > 0.0)
            {
                sigmaLevel = sigmaMultiplier;
            }
            else
            {
                // 片元 alpha = opacity · exp(-d²/2σ²)，衰减到阈值处的 σ 系数
                double opacity = s.A / 255.0;
                if (opacity <= VisibleAlphaThreshold)
                    return result; // 渲染中不可见，不绘制
                sigmaLevel = Math.Min(MaxSigmaLevel, Math.Sqrt(2.0 * Math.Log(opacity / VisibleAlphaThreshold)));
                if (sigmaLevel < 0.5)
                    sigmaLevel = 0.5;
            }

            // 四元数归一化并转旋转矩阵（与 Create3dgsellipsoid 一致）
            double w = s.RotW, x = s.RotX, y = s.RotY, z = s.RotZ;
            double qNorm = Math.Sqrt(w * w + x * x + y * y + z * z);
            if (qNorm < 1e-10)
                return result;
            w /= qNorm; x /= qNorm; y /= qNorm; z /= qNorm;

            double r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - w * z), r02 = 2 * (x * z + w * y);
            double r10 = 2 * (x * y + w * z), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - w * x);
            double r20 = 2 * (x * z - w * y), r21 = 2 * (y * z + w * x), r22 = 1 - 2 * (x * x + y * y);

            // σ 等值面半轴：σ × 全局 splat 缩放 × sigma 系数（与栅格化渲染可见尺寸一致）
            double sx = s.ScaleX * sigmaLevel * splatScale;
            double sy = s.ScaleY * sigmaLevel * splatScale;
            double sz = s.ScaleZ * sigmaLevel * splatScale;
            if (sx <= 1e-12 || sy <= 1e-12 || sz <= 1e-12)
                return result;
            double ix = 1.0 / (sx * sx), iy = 1.0 / (sy * sy), iz = 1.0 / (sz * sz);

            // 平面内正交基 U、V（与法线互相垂直）
            Vector3d aux = Math.Abs(planeNormal.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis;
            Vector3d uAxis = Vector3d.CrossProduct(planeNormal, aux);
            uAxis.Unitize();
            Vector3d vAxis = Vector3d.CrossProduct(planeNormal, uAxis);

            // 世界向量转到椭球局部坐标系：local = Rᵀ · world
            double uRx = r00 * uAxis.X + r10 * uAxis.Y + r20 * uAxis.Z;
            double uRy = r01 * uAxis.X + r11 * uAxis.Y + r21 * uAxis.Z;
            double uRz = r02 * uAxis.X + r12 * uAxis.Y + r22 * uAxis.Z;
            double vRx = r00 * vAxis.X + r10 * vAxis.Y + r20 * vAxis.Z;
            double vRy = r01 * vAxis.X + r11 * vAxis.Y + r21 * vAxis.Z;
            double vRz = r02 * vAxis.X + r12 * vAxis.Y + r22 * vAxis.Z;
            double qx = s.X - planeOrigin.X;
            double qy = s.Y - planeOrigin.Y;
            double qz = s.Z - planeOrigin.Z;
            double qRx = r00 * qx + r10 * qy + r20 * qz;
            double qRy = r01 * qx + r11 * qy + r21 * qz;
            double qRz = r02 * qx + r12 * qy + r22 * qz;

            // 二次曲线系数（u 沿 U、v 沿 V）
            double a = uRx * uRx * ix + uRy * uRy * iy + uRz * uRz * iz;
            double b = uRx * vRx * ix + uRy * vRy * iy + uRz * vRz * iz;
            double c = vRx * vRx * ix + vRy * vRy * iy + vRz * vRz * iz;
            double g = -(uRx * qRx * ix + uRy * qRy * iy + uRz * qRz * iz);
            double h = -(vRx * qRx * ix + vRy * qRy * iy + vRz * qRz * iz);
            double f = qRx * qRx * ix + qRy * qRy * iy + qRz * qRz * iz - 1.0;

            // 椭圆中心：[[a,b],[b,c]]·[u0,v0]ᵀ = -[g,h]ᵀ
            double det = a * c - b * b;
            if (Math.Abs(det) < 1e-30)
                return result;
            double u0 = (b * h - c * g) / det;
            double v0 = (b * g - a * h) / det;

            // 中心处二次曲线取值：< 0 为椭圆（平面真切开椭球），>= 0 无交或相切
            double f0 = f + g * u0 + h * v0;
            if (f0 >= 0.0)
                return result;

            // 2×2 对称矩阵特征分解（无抵消稳定形式）
            double mid = 0.5 * (a + c);
            double diff = 0.5 * (a - c);
            double root = Math.Sqrt(diff * diff + b * b);
            double lambda1 = mid + root;
            double lambda2 = mid - root;
            if (lambda1 <= 0.0 || lambda2 <= 0.0)
                return result;

            double e1x, e1y;
            if (a >= c) { e1x = lambda1 - c; e1y = b; }
            else { e1x = b; e1y = lambda1 - a; }
            double e1Len = Math.Sqrt(e1x * e1x + e1y * e1y);
            if (e1Len < 1e-300)
            {
                e1x = 1.0; e1y = 0.0; e1Len = 1.0;
            }
            e1x /= e1Len;
            e1y /= e1Len;
            double e2x = -e1y, e2y = e1x;

            // 半长轴：sqrt(-f0/λ)；λ1 较大对应短轴
            double radius1 = Math.Sqrt(-f0 / lambda1);
            double radius2 = Math.Sqrt(-f0 / lambda2);
            if (radius2 < minRadius)
                return result;

            double maxRadius = Math.Max(radius1, radius2);
            if (double.IsNaN(maxRadius) || maxRadius <= 0.0)
                return result;

            // 针状退化 splat（一个轴 σ 巨大、其余近 0，渲染中仅为发丝级条纹伪影）：
            // 其剖切"椭圆"退化为极细长的直线状细条，铺满视口毫无意义，按长短轴比排除
            if (maxAspect > 0.0 && radius1 > 1e-300 && radius2 / radius1 > maxAspect)
                return result;

            result.Valid = true;
            result.Center = planeOrigin + uAxis * u0 + vAxis * v0;
            result.Dir1 = uAxis * e1x + vAxis * e1y;
            result.Dir2 = uAxis * e2x + vAxis * e2y;
            result.Radius1 = radius1;
            result.Radius2 = radius2;
            result.Color = Color.FromArgb(255, s.R, s.G, s.B);
            return result;
        }

        private struct SectionEllipse
        {
            public bool Valid;
            public Point3d Center;
            public Vector3d Dir1;   // Radius1 对应的半轴方向
            public Vector3d Dir2;   // Radius2 对应的半轴方向
            public double Radius1;
            public double Radius2;
            public Color Color;
        }
    }
}
