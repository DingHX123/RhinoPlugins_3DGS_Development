using Rhino.Commands;
using Rhino.Input.Custom;
using Rhino.Input;
using Rhino;
using System;
using System.Collections.Generic;
using System.Linq;
using Rhino.DocObjects;
using System.IO;
using Rhino.Geometry;

namespace RhinoPlugins_Development
{
    public class Create3dgsellipsoid : Command
    {
        public override string EnglishName => "Create3dgsellipsoid";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            RhinoApp.WriteLine("请选择选取方式：");
            RhinoApp.WriteLine("  1 = 单点选取（点击查找最近点）");
            RhinoApp.WriteLine("  2 = 笔刷选取（涂抹方式选取点）");
            RhinoApp.WriteLine("  3 = 可见选取（只对当前显示的 3dgs 模型，视口可见点创建椭球）");

            var getString = new GetString();
            getString.SetCommandPrompt("输入选取方式编号 (1/2/3)");
            getString.AcceptNothing(false);
            if (getString.Get() != GetResult.String)
                return Result.Cancel;

            string modeStr = getString.StringResult();
            int selectionMode;
            if (!int.TryParse(modeStr, out selectionMode) || selectionMode < 1 || selectionMode > 3)
            {
                RhinoApp.WriteLine("无效的选取方式，请输入1、2或3");
                return Result.Failure;
            }

            PointCloud pointCloud = null;
            Guid cloudObjectId = Guid.Empty;
            List<int> selectedIndices = null;

            switch (selectionMode)
            {
                case 1:
                case 2:
                    {
                        // 单点/笔刷选取作用于用户指定（预选或提示选择）的点云
                        var selectedObjects = doc.Objects.GetSelectedObjects(false, false);
                        foreach (var rhinoObj in selectedObjects)
                        {
                            if (rhinoObj.ObjectType == ObjectType.PointSet)
                            {
                                pointCloud = rhinoObj.Geometry as PointCloud;
                                cloudObjectId = rhinoObj.Id;
                                break;
                            }
                        }

                        if (pointCloud == null)
                        {
                            var go = new GetObject();
                            go.SetCommandPrompt("请选择一个点云对象");
                            go.GeometryFilter = ObjectType.PointSet;
                            if (go.Get() != GetResult.Object)
                                return Result.Cancel;

                            pointCloud = go.Object(0).PointCloud();
                            cloudObjectId = go.Object(0).ObjectId;
                        }

                        if (pointCloud == null || pointCloud.Count == 0)
                        {
                            RhinoApp.WriteLine("未获取到有效的点云");
                            return Result.Failure;
                        }

                        selectedIndices = selectionMode == 1
                            ? SelectSinglePoint(pointCloud)
                            : SelectByBrush(doc, pointCloud, cloudObjectId);
                        break;
                    }
                case 3:
                    {
                        // 可见选取只对当前显示的 3dgs 模型进行：取渲染管线目标实例的代理点云，
                        // 忽略用户预选的其他点云；后续数据源查找按 cloudObjectId 命中渲染实例，
                        // 不会落入硬编码 PLY 回退路径
                        SplatInstance displayInstance = SplatDisplayConduit.GetTargetInstance(
                            doc, "选择当前显示的 3dgs 模型");
                        if (displayInstance == null || displayInstance.ProxyObjectId == Guid.Empty)
                        {
                            RhinoApp.WriteLine("当前没有显示的 3dgs 模型，无法进行可见选取创建椭球");
                            return Result.Failure;
                        }

                        RhinoObject proxyObj = doc.Objects.FindId(displayInstance.ProxyObjectId);
                        pointCloud = proxyObj != null ? proxyObj.Geometry as PointCloud : null;
                        if (pointCloud == null || pointCloud.Count == 0)
                        {
                            RhinoApp.WriteLine("未找到当前显示 3dgs 模型的代理点云");
                            return Result.Failure;
                        }

                        cloudObjectId = displayInstance.ProxyObjectId;
                        selectedIndices = SelectVisiblePoints(doc, pointCloud);
                        break;
                    }
            }

            if (selectedIndices == null || selectedIndices.Count == 0)
            {
                RhinoApp.WriteLine("未选中任何点");
                return Result.Cancel;
            }

            RhinoApp.WriteLine($"共选中 {selectedIndices.Count} 个点，开始创建椭球...");

            // 数据源：优先使用 ImportGaussianSplat 渲染管线中的 splat 数据
            // （已经过 UpAxis/AutoCrop/LevelFloor 处理，与 GPU 光栅渲染的椭球严格一致）
            SplatInstance renderInstance = null;
            int sampleStep = 1;
            if (SplatDisplayConduit.Instance != null && cloudObjectId != Guid.Empty)
            {
                renderInstance = SplatDisplayConduit.Instance.Instances.Find(x => x.ProxyObjectId == cloudObjectId);
                if (renderInstance != null && renderInstance.Splats != null && renderInstance.Splats.Length > 0)
                {
                    // 代理点云是按固定步长抽样的：点云第 k 点对应 splat 索引 k * step
                    sampleStep = Math.Max(1, (int)Math.Ceiling((double)renderInstance.Splats.Length / Math.Max(1, pointCloud.Count)));
                }
                else
                {
                    renderInstance = null;
                }
            }

            List<GaussianPoint> gaussianPoints = null;
            bool useRhinoCenter = false;
            double sigmaMultiplier = 3.0; // 与渲染着色器一致（3σ 等值面）

            if (renderInstance != null)
            {
                RhinoApp.WriteLine($"数据源：渲染管线 splat（总数 {renderInstance.Splats.Length:N0}，代理抽样步长 {sampleStep}）");
            }
            else
            {
                // 回退：点云不是当前渲染的代理点云，解析 PLY 并按默认 YDown 上轴约定对齐朝向
                string plyPath = @"E:\Users\Windows11\Desktop\models\kitchen\point_cloud\iteration_30000\point_cloud.ply";
                if (!File.Exists(plyPath))
                {
                    RhinoApp.WriteLine("点云不是当前渲染的代理点云，且 PLY 文件不存在，请修改代码中的路径");
                    return Result.Failure;
                }

                gaussianPoints = Get3dgsPointEllipsoid.Parse3dgsPlyBinary(plyPath);

                Point3d testPos = pointCloud.PointAt(0);
                double testDiff = testPos.DistanceTo(new Point3d(gaussianPoints[0].X, gaussianPoints[0].Y, gaussianPoints[0].Z));
                useRhinoCenter = testDiff > 0.01;

                if (useRhinoCenter)
                {
                    RhinoApp.WriteLine("检测到坐标系不一致，将使用Rhino点云位置并按 YDown 上轴约定对齐朝向");
                }
            }

            int createdCount = 0;
            int skippedCount = 0;

            foreach (int idx in selectedIndices)
            {
                GaussianPoint gp;
                Point3d center;

                if (renderInstance != null)
                {
                    // 渲染管线数据：代理点云第 k 点对应 splat 索引 k * step
                    int splatIdx = idx * sampleStep;
                    if (splatIdx < 0 || splatIdx >= renderInstance.Splats.Length)
                    {
                        skippedCount++;
                        continue;
                    }
                    Splat s = renderInstance.Splats[splatIdx];
                    gp = SplatToGaussianPoint(s);
                    center = new Point3d(s.X, s.Y, s.Z);
                }
                else
                {
                    if (idx < 0 || idx >= gaussianPoints.Count)
                    {
                        skippedCount++;
                        continue;
                    }
                    gp = gaussianPoints[idx];
                    center = pointCloud.PointAt(idx);
                }

                Brep ellipsoidBrep = CreateEllipsoidBrep(gp, center, sigmaMultiplier, useRhinoCenter);
                if (ellipsoidBrep == null)
                {
                    skippedCount++;
                    continue;
                }

                var attributes = new ObjectAttributes();
                attributes.Name = $"3DGS_Ellipsoid_{idx}";

                double r = 0.5, g = 0.5, b = 0.5;
                double shCoeff = 0.28209479177387814;
                if (gp.AllProperties.ContainsKey("f_dc_0"))
                    r = 0.5 + gp.AllProperties["f_dc_0"] * shCoeff;
                if (gp.AllProperties.ContainsKey("f_dc_1"))
                    g = 0.5 + gp.AllProperties["f_dc_1"] * shCoeff;
                if (gp.AllProperties.ContainsKey("f_dc_2"))
                    b = 0.5 + gp.AllProperties["f_dc_2"] * shCoeff;
                r = MathUtil.Clamp01(r); g = MathUtil.Clamp01(g); b = MathUtil.Clamp01(b);

                var displayColor = System.Drawing.Color.FromArgb(255, (int)(r * 255), (int)(g * 255), (int)(b * 255));
                attributes.ObjectColor = displayColor;
                attributes.ColorSource = ObjectColorSource.ColorFromObject;

                double opacity = 1.0;
                if (gp.AllProperties.ContainsKey("opacity"))
                {
                    double logit = gp.AllProperties["opacity"];
                    opacity = 1.0 / (1.0 + Math.Exp(-logit));
                }
                opacity = MathUtil.Clamp01(opacity);

                Guid objectId = doc.Objects.AddBrep(ellipsoidBrep, attributes);

                if (objectId != Guid.Empty)
                {
                    int materialIndex = doc.Materials.Add();
                    var material = doc.Materials[materialIndex];
                    material.DiffuseColor = displayColor;
                    material.Transparency = 1.0 - opacity;
                    material.CommitChanges();

                    var rhinoObj = doc.Objects.FindId(objectId);
                    if (rhinoObj != null)
                    {
                        rhinoObj.Attributes.MaterialSource = ObjectMaterialSource.MaterialFromObject;
                        rhinoObj.Attributes.MaterialIndex = materialIndex;
                        rhinoObj.CommitChanges();
                    }
                }

                createdCount++;
            }

            doc.Views.Redraw();
            RhinoApp.WriteLine($"创建完成：成功 {createdCount} 个，跳过 {skippedCount} 个");

            return Result.Success;
        }

        private List<int> SelectSinglePoint(PointCloud pointCloud)
        {
            var gpo = new GetPoint();
            gpo.SetCommandPrompt("点击目标位置，自动查找最近顶点");
            if (gpo.Get() != GetResult.Point)
                return null;

            Point3d clickPosition = gpo.Point();
            int pointIndex = pointCloud.ClosestPoint(clickPosition);
            if (pointIndex < 0)
                return null;

            RhinoApp.WriteLine($"单点选取：点索引 {pointIndex}");
            return new List<int> { pointIndex };
        }

        private List<int> SelectByBrush(RhinoDoc doc, PointCloud pointCloud, Guid cloudObjectId)
        {
            double brushRadius = 0.5;

            using (var getNum = new GetNumber())
            {
                getNum.SetCommandPrompt("输入笔刷半径（模型单位）");
                getNum.SetDefaultNumber(brushRadius);
                getNum.SetLowerLimit(0.001, true);
                if (getNum.Get() == GetResult.Number)
                    brushRadius = getNum.Number();
            }

            RhinoApp.WriteLine($"笔刷半径：{brushRadius:F4}");
            RhinoApp.WriteLine("在点云上点击涂抹选取，按 Enter 完成选取");

            var selectedSet = new HashSet<int>();
            var brushGp = new BrushGetPoint(brushRadius);

            while (true)
            {
                brushGp.SetCommandPrompt($"点击涂抹选取（已选 {selectedSet.Count} 个点），按 Enter 完成");
                brushGp.AcceptNothing(true);

                GetResult result = brushGp.Get();

                if (result == GetResult.Nothing)
                    break;

                if (result == GetResult.Point)
                {
                    Point3d clickPos = brushGp.Point();
                    List<int> nearby = FindPointsInRadius(pointCloud, clickPos, brushRadius);

                    int addedThisClick = 0;
                    foreach (int idx in nearby)
                    {
                        if (selectedSet.Add(idx))
                            addedThisClick++;
                    }

                    if (addedThisClick > 0)
                    {
                        HighlightPoints(doc, cloudObjectId, selectedSet.ToList());
                        RhinoApp.WriteLine($"  本次新增 {addedThisClick} 个点，累计 {selectedSet.Count} 个");
                    }
                }
                else
                {
                    break;
                }
            }

            if (selectedSet.Count == 0)
                return null;

            RhinoApp.WriteLine($"笔刷选取完成：共 {selectedSet.Count} 个点");
            return selectedSet.ToList();
        }

        private List<int> FindPointsInRadius(PointCloud pointCloud, Point3d center, double radius)
        {
            var indices = new List<int>();
            double rSq = radius * radius;

            for (int i = 0; i < pointCloud.Count; i++)
            {
                Point3d pt = pointCloud.PointAt(i);
                double dx = pt.X - center.X;
                double dy = pt.Y - center.Y;
                double dz = pt.Z - center.Z;
                double distSq = dx * dx + dy * dy + dz * dz;

                if (distSq <= rSq)
                    indices.Add(i);
            }

            return indices;
        }

        private List<int> SelectVisiblePoints(RhinoDoc doc, PointCloud pointCloud)
        {
            var view = doc.Views.ActiveView;
            if (view == null)
            {
                RhinoApp.WriteLine("无法获取活动视口，回退为全部选取");
                return SelectAllPointsFallback(pointCloud);
            }

            var viewport = view.ActiveViewport;
            var clientRect = view.ClientRectangle;

            bool isPerspective = viewport.IsPerspectiveProjection;
            Point3d cameraPos = viewport.CameraLocation;
            Vector3d cameraDir = viewport.CameraDirection;

            var indices = new List<int>();
            for (int i = 0; i < pointCloud.Count; i++)
            {
                Point3d pt = pointCloud.PointAt(i);

                if (isPerspective)
                {
                    Vector3d toPoint = pt - cameraPos;
                    if (toPoint * cameraDir <= 0)
                        continue;
                }

                Point2d screenPt = viewport.WorldToClient(pt);
                if (screenPt.X >= clientRect.Left && screenPt.X <= clientRect.Right &&
                    screenPt.Y >= clientRect.Top && screenPt.Y <= clientRect.Bottom)
                {
                    indices.Add(i);
                }
            }

            RhinoApp.WriteLine($"可见选取：视口内共 {indices.Count} 个点（总 {pointCloud.Count} 个）");
            return indices;
        }

        private List<int> SelectAllPointsFallback(PointCloud pointCloud)
        {
            var indices = new List<int>(pointCloud.Count);
            for (int i = 0; i < pointCloud.Count; i++)
            {
                indices.Add(i);
            }

            RhinoApp.WriteLine($"全部选取：共 {indices.Count} 个点");
            return indices;
        }

        private void HighlightPoints(RhinoDoc doc, Guid cloudObjectId, List<int> indices)
        {
            doc.Objects.UnselectAll(false);

            var rhinoObj = doc.Objects.FindId(cloudObjectId);
            if (rhinoObj != null)
            {
                foreach (int idx in indices)
                {
                    var ci = new ComponentIndex(ComponentIndexType.PointCloudPoint, idx);
                    rhinoObj.SelectSubObject(ci, true, true);
                }
            }

            doc.Views.Redraw();
        }

        /// <summary>
        /// 把渲染管线中的 Splat（已经过 UpAxis/AutoCrop/LevelFloor 处理，与光栅渲染一致）
        /// 转为椭球参数；AllProperties 反算回 PLY 原始空间以便属性窗口展示。
        /// </summary>
        internal static GaussianPoint SplatToGaussianPoint(Splat s)
        {
            var gp = new GaussianPoint
            {
                X = s.X,
                Y = s.Y,
                Z = s.Z,
                ScaleX = s.ScaleX,
                ScaleY = s.ScaleY,
                ScaleZ = s.ScaleZ,
                RotW = s.RotW,
                RotX = s.RotX,
                RotY = s.RotY,
                RotZ = s.RotZ
            };
            gp.AllProperties["x"] = s.X;
            gp.AllProperties["y"] = s.Y;
            gp.AllProperties["z"] = s.Z;
            gp.AllProperties["scale_0"] = Math.Log(Math.Max(s.ScaleX, 1e-8));
            gp.AllProperties["scale_1"] = Math.Log(Math.Max(s.ScaleY, 1e-8));
            gp.AllProperties["scale_2"] = Math.Log(Math.Max(s.ScaleZ, 1e-8));
            gp.AllProperties["rot_0"] = s.RotW;
            gp.AllProperties["rot_1"] = s.RotX;
            gp.AllProperties["rot_2"] = s.RotY;
            gp.AllProperties["rot_3"] = s.RotZ;

            double shCoeff = 0.28209479177387814;
            gp.AllProperties["f_dc_0"] = (s.R / 255.0 - 0.5) / shCoeff;
            gp.AllProperties["f_dc_1"] = (s.G / 255.0 - 0.5) / shCoeff;
            gp.AllProperties["f_dc_2"] = (s.B / 255.0 - 0.5) / shCoeff;
            double a = MathUtil.Clamp(s.A / 255.0, 1e-4, 1.0 - 1e-4);
            gp.AllProperties["opacity"] = Math.Log(a / (1.0 - a));
            return gp;
        }

        internal static Brep CreateEllipsoidBrep(GaussianPoint gp, Point3d center, double sigmaMultiplier, bool useRhinoCenter)
        {
            var sphere = new Sphere(Point3d.Origin, 1.0);
            Brep brep = sphere.ToBrep();
            if (brep == null)
                return null;

            double sx = gp.ScaleX * sigmaMultiplier;
            double sy = gp.ScaleY * sigmaMultiplier;
            double sz = gp.ScaleZ * sigmaMultiplier;

            var scaleTransform = Transform.Scale(Plane.WorldXY, sx, sy, sz);
            brep.Transform(scaleTransform);

            double w = gp.RotW, x = gp.RotX, y = gp.RotY, z = gp.RotZ;
            double qNorm = Math.Sqrt(w * w + x * x + y * y + z * z);
            if (qNorm > 1e-10)
            {
                w /= qNorm;
                x /= qNorm;
                y /= qNorm;
                z /= qNorm;
            }

            if (useRhinoCenter)
            {
                // 与 ImportGaussianSplat 默认导入一致：YDown 上轴约定 = 绕世界 X 轴 -90°。
                // 渲染管线对位置与朝向四元数应用同一旋转，这里对四元数左乘该旋转
                // q_rot = (cos(-45°), sin(-45°), 0, 0)；位置由调用方以 Rhino 点云点覆盖。
                double c = 0.7071067811865476;
                double nW = w * c + x * c;
                double nX = x * c - w * c;
                double nY = y * c + z * c;
                double nZ = z * c - y * c;
                double nLen = Math.Sqrt(nW * nW + nX * nX + nY * nY + nZ * nZ);
                if (nLen > 1e-10)
                {
                    w = nW / nLen;
                    x = nX / nLen;
                    y = nY / nLen;
                    z = nZ / nLen;
                }
            }

            var rotationTransform = Transform.Identity;
            rotationTransform[0, 0] = 1 - 2 * (y * y + z * z);
            rotationTransform[0, 1] = 2 * (x * y - w * z);
            rotationTransform[0, 2] = 2 * (x * z + w * y);
            rotationTransform[1, 0] = 2 * (x * y + w * z);
            rotationTransform[1, 1] = 1 - 2 * (x * x + z * z);
            rotationTransform[1, 2] = 2 * (y * z - w * x);
            rotationTransform[2, 0] = 2 * (x * z - w * y);
            rotationTransform[2, 1] = 2 * (y * z + w * x);
            rotationTransform[2, 2] = 1 - 2 * (x * x + y * y);
            brep.Transform(rotationTransform);

            Point3d translationTarget = useRhinoCenter ? center : new Point3d(gp.X, gp.Y, gp.Z);
            var translationTransform = Transform.Translation(translationTarget - Point3d.Origin);
            brep.Transform(translationTransform);

            return brep;
        }
    }

    public class BrushGetPoint : GetPoint
    {
        private readonly double _brushRadius;

        public BrushGetPoint(double brushRadius)
        {
            _brushRadius = brushRadius;

            DynamicDraw += (sender, e) =>
            {
                Point3d current = e.CurrentPoint;
                var brushColor = System.Drawing.Color.FromArgb(180, 0, 120, 215);
                var circle = new Circle(new Plane(current, Vector3d.ZAxis), _brushRadius);
                e.Display.DrawCircle(circle, brushColor, 2);
            };
        }
    }

    internal static class MathUtil
    {
        internal static double Clamp01(double v)
        {
            if (v < 0) return 0;
            if (v > 1) return 1;
            return v;
        }

        internal static double Clamp(double v, double min, double max)
        {
            if (v < min) return min;
            if (v > max) return max;
            return v;
        }
    }
}