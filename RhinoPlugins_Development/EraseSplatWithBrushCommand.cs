using Rhino;
using Rhino.Commands;
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
    /// <summary>球形笔刷擦除：视口中点击擦除笔刷范围内的 splat，可循环操作，结束后统一注册撤销。</summary>
    public class EraseSplatWithBrushCommand : Command
    {
        public override string EnglishName => "EraseSplatWithBrush";

        /// <summary>笔刷取点：动态绘制红色圆形轮廓和半透明红色球体指示笔刷范围。</summary>
        private class BrushGetPoint : GetPoint
        {
            private readonly double _radius;

            public BrushGetPoint(double radius)
            {
                _radius = radius;
            }

            protected override void OnDynamicDraw(GetPointDrawEventArgs e)
            {
                base.OnDynamicDraw(e);
                Plane plane = e.Viewport.ConstructionPlane();
                plane.Origin = e.CurrentPoint;
                Circle circle = new Circle(plane, _radius);
                e.Display.DrawCircle(circle, System.Drawing.Color.Red, 2);
                e.Display.DrawSphere(new Sphere(e.CurrentPoint, _radius), System.Drawing.Color.FromArgb(40, 255, 0, 0));
            }
        }

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (SplatDisplayConduit.Instance == null || !SplatDisplayConduit.Instance.Enabled ||
                SplatDisplayConduit.Instance.Instances.Count == 0)
            {
                RhinoApp.WriteLine("Gaussian Splatting is not active. Please load a splat model first.");
                return Result.Failure;
            }

            SplatInstance targetInstance = SplatDisplayConduit.GetTargetInstance(
                doc, "Select splat proxy point cloud to erase");
            if (targetInstance == null)
            {
                RhinoApp.WriteLine("No target splat dataset selected.");
                return Result.Failure;
            }

            Splat[] originalSplats = targetInstance.Splats;
            if (originalSplats == null || originalSplats.Length == 0)
            {
                RhinoApp.WriteLine("No splats loaded to edit.");
                return Result.Failure;
            }

            // 笔刷半径选项
            double radius = 1.0;
            OptionDouble radiusOption = new OptionDouble(radius, 0.01, 1000.0);
            GetOption option = new GetOption();
            option.SetCommandPrompt("Adjust eraser brush options");
            option.AcceptNothing(true);
            option.AddOptionDouble("Radius", ref radiusOption, "Eraser brush radius");

            GetResult result;
            do
            {
                result = option.Get();
                if (result == GetResult.Cancel)
                    return Result.Cancel;
            }
            while (result == GetResult.Option);

            radius = radiusOption.CurrentValue;
            double radiusSquared = radius * radius;
            Splat[] activeSplats = (Splat[])originalSplats.Clone();
            bool modified = false;

            // 循环擦除
            while (true)
            {
                BrushGetPoint brushPoint = new BrushGetPoint(radius);
                brushPoint.SetCommandPrompt("Click in the viewport to erase splats under the brush (Press Enter or Escape when done)");
                brushPoint.AcceptNothing(true);

                if (brushPoint.Get() != GetResult.Point)
                    break;

                Point3d clickPoint = brushPoint.Point();
                List<Splat> keptList = new List<Splat>();
                object lockObj = new object();

                Parallel.For(0, activeSplats.Length,
                    () => new List<Splat>(),
                    (i, loopState, localList) =>
                    {
                        Splat splat = activeSplats[i];
                        double dx = splat.X - clickPoint.X;
                        double dy = splat.Y - clickPoint.Y;
                        double dz = splat.Z - clickPoint.Z;
                        if (dx * dx + dy * dy + dz * dz > radiusSquared)
                            localList.Add(splat);
                        return localList;
                    },
                    localList =>
                    {
                        lock (lockObj)
                        {
                            keptList.AddRange(localList);
                        }
                    });

                if (keptList.Count < activeSplats.Length)
                {
                    int erasedCount = activeSplats.Length - keptList.Count;
                    activeSplats = keptList.ToArray();
                    modified = true;
                    targetInstance.SetSplats(activeSplats);
                    doc.Views.Redraw();
                    RhinoApp.WriteLine(string.Format(
                        "Erased {0:N0} splats. Current remaining: {1:N0}",
                        erasedCount, activeSplats.Length));
                }
            }

            if (!modified)
                return Result.Failure;

            // 注册撤销 + 写修改缓存 + 更新代理点云
            string previousFilePath = doc.Objects.FindId(targetInstance.ProxyObjectId)?
                .Attributes.GetUserString("SplatFilePath");
            SplatDisplayConduit.RegisterSplatsModifiedUndo(
                doc, "Erase Gaussian Splats with Brush", targetInstance.ProxyObjectId,
                originalSplats, activeSplats, previousFilePath);

            string cachePath = SplatDisplayConduit.SaveModifiedSplatCache(targetInstance.ProxyObjectId, activeSplats);

            Point3d p0 = Point3d.Unset, p1 = Point3d.Unset, p2 = Point3d.Unset;
            if (targetInstance.ProxyObjectId != Guid.Empty)
            {
                try
                {
                    PointCloud cloud = new PointCloud();
                    int step = Math.Max(1, activeSplats.Length / 50000);
                    for (int i = 0; i < activeSplats.Length; i += step)
                    {
                        Splat splat = activeSplats[i];
                        cloud.Add(new Point3d(splat.X, splat.Y, splat.Z), Color.FromArgb(splat.R, splat.G, splat.B));
                    }
                    while (cloud.Count < 3)
                        cloud.Add(new Point3d(0, 0, 0), Color.Black);

                    p0 = cloud[0].Location;
                    p1 = cloud[1].Location;
                    p2 = cloud[2].Location;

                    doc.Objects.Replace(targetInstance.ProxyObjectId, cloud);
                    RhinoObject proxyObj = doc.Objects.FindId(targetInstance.ProxyObjectId);
                    if (proxyObj != null)
                    {
                        if (!string.IsNullOrEmpty(cachePath))
                        {
                            proxyObj.Attributes.SetUserString("SplatFilePath", cachePath);
                            proxyObj.Attributes.SetUserString("IsModifiedSplat", "True");
                        }
                        proxyObj.Attributes.SetUserString("OrigP0", SplatDisplayConduit.FormatPoint(p0));
                        proxyObj.Attributes.SetUserString("OrigP1", SplatDisplayConduit.FormatPoint(p1));
                        proxyObj.Attributes.SetUserString("OrigP2", SplatDisplayConduit.FormatPoint(p2));
                        proxyObj.CommitChanges();
                    }
                }
                catch (Exception ex)
                {
                    RhinoApp.WriteLine("Error updating proxy point cloud: " + ex.Message);
                }
            }

            targetInstance.SetSplats(activeSplats);
            if (p0 != Point3d.Unset)
            {
                targetInstance.OrigP0 = p0;
                targetInstance.OrigP1 = p1;
                targetInstance.OrigP2 = p2;
            }

            RhinoApp.WriteLine(string.Format("Brushing finished. Remaining splats: {0:N0}.", activeSplats.Length));
            doc.Views.Redraw();
            return Result.Success;
        }
    }
}
