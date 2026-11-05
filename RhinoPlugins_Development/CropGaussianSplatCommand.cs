using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;

namespace RhinoPlugins_Development
{
    /// <summary>用封闭几何体（Box / 封闭体）裁剪高斯泼溅模型，保留内部或外部。</summary>
    public class CropGaussianSplatCommand : Command
    {
        public override string EnglishName => "CropGaussianSplat";

        private static Mesh ConvertBrepToMesh(Brep brep)
        {
            if (brep == null)
                return null;

            Mesh[] pieces = Mesh.CreateFromBrep(brep, MeshingParameters.FastRenderMesh);
            if (pieces == null || pieces.Length == 0)
                return null;

            Mesh mesh = new Mesh();
            foreach (Mesh piece in pieces)
                mesh.Append(piece);
            return mesh;
        }

        /// <summary>把封闭平面曲线沿法线方向挤出成封闭网格体（用于曲线裁剪边界）。</summary>
        private static Mesh ConvertCurveToMesh(Curve curve, BoundingBox splatsBBox)
        {
            Plane plane;
            if (curve == null || !curve.IsClosed)
                return null;

            if (!curve.TryGetPlane(out plane))
                plane = Plane.WorldXY;

            // 求点云包围盒在曲线平面法线方向上的投影范围
            double minDistance = double.MaxValue;
            double maxDistance = double.MinValue;
            Point3d[] corners = splatsBBox.GetCorners();
            foreach (Point3d corner in corners)
            {
                double distance = (corner - plane.Origin) * plane.Normal;
                if (distance < minDistance) minDistance = distance;
                if (distance > maxDistance) maxDistance = distance;
            }

            double margin = (maxDistance - minDistance) * 0.1;
            if (margin < 10.0)
                margin = 100.0;
            minDistance -= margin;
            maxDistance += margin;

            // 将曲线平移到范围一端，再向另一端挤出
            Plane farPlane = plane;
            farPlane.Translate(plane.Normal * minDistance);
            Curve moved = curve.DuplicateCurve();
            moved.Transform(Transform.PlaneToPlane(plane, farPlane));

            double height = maxDistance - minDistance;
            if (height < 0.001)
                height = 1.0;

            Extrusion extrusion = Extrusion.Create(moved, height, true);
            if (extrusion == null)
                return null;

            Brep brep = extrusion.ToBrep();
            if (brep == null)
                return null;

            return ConvertBrepToMesh(brep);
        }

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (SplatDisplayConduit.Instance == null || !SplatDisplayConduit.Instance.Enabled ||
                SplatDisplayConduit.Instance.Instances.Count == 0)
            {
                RhinoApp.WriteLine("Gaussian Splatting is not active. Please import a splat model first using ImportGaussianSplat.");
                return Result.Failure;
            }

            SplatInstance targetInstance = SplatDisplayConduit.GetTargetInstance(
                doc, "Select splat proxy point cloud to crop");
            if (targetInstance == null)
            {
                RhinoApp.WriteLine("No target splat dataset selected.");
                return Result.Failure;
            }

            Splat[] currentSplats = targetInstance.Splats;
            if (currentSplats == null || currentSplats.Length == 0)
            {
                RhinoApp.WriteLine("No splats loaded to crop.");
                return Result.Failure;
            }

            // 选择裁剪模式
            int modeIndex = 0;
            GetObject go = new GetObject();
            go.GeometryFilter = ObjectType.Brep | ObjectType.Mesh | ObjectType.Curve | ObjectType.Extrusion;
            go.AcceptNothing(true);
            int modeListIndex = go.AddOptionList("Mode", new string[] { "KeepInside", "KeepOutside" }, modeIndex);

            GeometryBase boundaryGeometry = null;
            Box selectedBox = Box.Empty;
            bool isBox = false;

            while (true)
            {
                go.SetCommandPrompt("Select closed solid, closed mesh, closed curve, or extrusion to crop (press Enter to draw a box)");
                GetResult result = go.Get();
                if (result == GetResult.Nothing)      // Enter：手动绘制 Box
                    break;
                if (result == GetResult.Option)
                {
                    CommandLineOption opt = go.Option();
                    if (opt != null && opt.Index == modeListIndex)
                        modeIndex = opt.CurrentListOptionIndex;
                    continue;
                }
                if (result == GetResult.Object)
                {
                    RhinoObject rhObj = go.Object(0).Object();
                    if (rhObj != null)
                        boundaryGeometry = rhObj.Geometry;
                    isBox = false;
                    break;
                }
                return Result.Cancel;
            }

            if (isBox == false && boundaryGeometry == null)
            {
                // Enter 分支：画 Box
                Box box;
                if (RhinoGet.GetBox(out box) != Result.Success)
                    return Result.Cancel;
                selectedBox = box;
                isBox = true;
            }

            Mesh cropMesh = null;
            BoundingBox cropBBox = BoundingBox.Empty;

            if (!isBox)
            {
                cropMesh = boundaryGeometry as Mesh;
                if (cropMesh == null)
                {
                    Brep brep = boundaryGeometry as Brep;
                    if (brep != null)
                        cropMesh = ConvertBrepToMesh(brep);
                    else
                    {
                        Extrusion extrusion = boundaryGeometry as Extrusion;
                        if (extrusion != null)
                            cropMesh = ConvertBrepToMesh(extrusion.ToBrep());
                        else
                        {
                            Curve curve = boundaryGeometry as Curve;
                            if (curve != null)
                                cropMesh = ConvertCurveToMesh(curve, targetInstance.BBox);
                        }
                    }
                }

                if (cropMesh == null || !cropMesh.IsClosed)
                {
                    RhinoApp.WriteLine("Cropping boundary geometry must form a closed solid volume.");
                    return Result.Failure;
                }
                cropBBox = cropMesh.GetBoundingBox(true);
            }
            else
            {
                cropBBox = selectedBox.BoundingBox;
            }

            // 并行过滤
            RhinoApp.WriteLine("Filtering splats...");
            Stopwatch stopwatch = Stopwatch.StartNew();
            bool keepInside = modeIndex == 0;
            List<Splat> keptSplats = new List<Splat>();
            object lockObj = new object();

            Parallel.For(0, currentSplats.Length,
                () => new List<Splat>(),
                (i, loopState, localList) =>
                {
                    Splat splat = currentSplats[i];
                    Point3d point = new Point3d(splat.X, splat.Y, splat.Z);
                    bool inside = false;
                    if (cropBBox.Contains(point))
                    {
                        inside = isBox
                            ? selectedBox.Contains(point)
                            : cropMesh.IsPointInside(point, 1e-06, true);
                    }
                    if (inside == keepInside)
                        localList.Add(splat);
                    return localList;
                },
                localList =>
                {
                    lock (lockObj)
                    {
                        keptSplats.AddRange(localList);
                    }
                });
            stopwatch.Stop();

            Splat[] afterSplats = keptSplats.ToArray();
            if (afterSplats.Length == 0)
            {
                RhinoApp.WriteLine("Cropping operation would discard all splats. Cancelled.");
                return Result.Cancel;
            }

            // 注册撤销 + 写修改缓存 + 更新代理点云
            string previousFilePath = doc.Objects.FindId(targetInstance.ProxyObjectId)?
                .Attributes.GetUserString("SplatFilePath");
            SplatDisplayConduit.RegisterSplatsModifiedUndo(
                doc, "Crop Gaussian Splat", targetInstance.ProxyObjectId,
                currentSplats, afterSplats, previousFilePath);

            string cachePath = SplatDisplayConduit.SaveModifiedSplatCache(targetInstance.ProxyObjectId, afterSplats);

            Point3d p0 = Point3d.Unset, p1 = Point3d.Unset, p2 = Point3d.Unset;
            if (targetInstance.ProxyObjectId != Guid.Empty)
            {
                try
                {
                    PointCloud cloud = new PointCloud();
                    int step = Math.Max(1, afterSplats.Length / 50000);
                    for (int i = 0; i < afterSplats.Length; i += step)
                    {
                        Splat splat = afterSplats[i];
                        cloud.Add(new Point3d(splat.X, splat.Y, splat.Z), System.Drawing.Color.FromArgb(splat.R, splat.G, splat.B));
                    }
                    while (cloud.Count < 3)
                        cloud.Add(new Point3d(0, 0, 0), System.Drawing.Color.Black);

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

            targetInstance.SetSplats(afterSplats);
            if (p0 != Point3d.Unset)
            {
                targetInstance.OrigP0 = p0;
                targetInstance.OrigP1 = p1;
                targetInstance.OrigP2 = p2;
            }

            RhinoApp.WriteLine(string.Format(
                "Cropped {0:N0} splats to {1:N0} splats (removed {2:N0}) in {3:F2} seconds.",
                currentSplats.Length, afterSplats.Length,
                currentSplats.Length - afterSplats.Length,
                stopwatch.ElapsedMilliseconds / 1000.0));

            doc.Views.Redraw();
            return Result.Success;
        }
    }
}
