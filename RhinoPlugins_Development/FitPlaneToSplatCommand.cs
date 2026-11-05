using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Rhino.Input;
using Rhino.Input.Custom;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;

namespace RhinoPlugins_Development
{
    /// <summary>在边界区域内对 splat 中心点做 RANSAC 基元拟合（平面 / 圆柱 / 球）。</summary>
    public class FitPlaneToSplatCommand : Command
    {
        public override string EnglishName => "FitPlaneToSplat";

        // ==================== 边界几何转换 ====================

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

        private static Mesh ConvertCurveToMesh(Curve curve, BoundingBox splatsBBox)
        {
            Plane plane;
            if (curve == null || !curve.IsClosed)
                return null;

            if (!curve.TryGetPlane(out plane))
                plane = Plane.WorldXY;

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

            Plane nearPlane = plane;
            nearPlane.Translate(plane.Normal * minDistance);
            Curve moved = curve.DuplicateCurve();
            moved.Transform(Transform.PlaneToPlane(plane, nearPlane));

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

        // ==================== 法线估计 ====================

        /// <summary>基于体素网格邻域的协方差分析估计点法线（主成分最小特征方向，用叉积近似）。</summary>
        private static Vector3d[] EstimateNormals(List<Point3d> points)
        {
            int count = points.Count;
            Vector3d[] normals = new Vector3d[count];
            if (count < 4)
                return normals;

            BoundingBox box = BoundingBox.Empty;
            foreach (Point3d p in points)
                box.Union(p);
            double voxelSize = Math.Max(0.01, box.Diagonal.Length / 35.0);

            // 体素哈希网格
            Dictionary<long, List<int>> grid = new Dictionary<long, List<int>>();
            for (int i = 0; i < count; i++)
            {
                Point3d p = points[i];
                long vx = (long)Math.Floor(p.X / voxelSize);
                long vy = (long)Math.Floor(p.Y / voxelSize);
                long vz = (long)Math.Floor(p.Z / voxelSize);
                long hash = (((vx * 0x466f45dL) ^ (vy * 0x127409fL)) ^ (vz * 0x4f9ffb7L));
                List<int> bucket;
                if (!grid.TryGetValue(hash, out bucket))
                {
                    bucket = new List<int>();
                    grid[hash] = bucket;
                }
                bucket.Add(i);
            }

            Parallel.For(0, count, index =>
            {
                Point3d p = points[index];
                long vx = (long)Math.Floor(p.X / voxelSize);
                long vy = (long)Math.Floor(p.Y / voxelSize);
                long vz = (long)Math.Floor(p.Z / voxelSize);

                // 收集 27 邻域内的点
                List<Point3d> neighbors = new List<Point3d>();
                for (long dx = -1; dx <= 1; dx++)
                {
                    for (long dy = -1; dy <= 1; dy++)
                    {
                        for (long dz = -1; dz <= 1; dz++)
                        {
                            long hash = (((vx + dx) * 0x466f45dL) ^ ((vy + dy) * 0x127409fL)) ^ ((vz + dz) * 0x4f9ffb7L);
                            List<int> bucket;
                            if (!grid.TryGetValue(hash, out bucket))
                                continue;
                            foreach (int neighborIndex in bucket)
                                neighbors.Add(points[neighborIndex]);
                        }
                    }
                }

                if (neighbors.Count < 3)
                {
                    normals[index] = Vector3d.ZAxis;
                    return;
                }

                // 协方差矩阵（对称，取两个行向量叉积近似最小方向）
                double meanX = 0, meanY = 0, meanZ = 0;
                foreach (Point3d n in neighbors)
                {
                    meanX += n.X; meanY += n.Y; meanZ += n.Z;
                }
                meanX /= neighbors.Count;
                meanY /= neighbors.Count;
                meanZ /= neighbors.Count;

                double covXX = 0, covXY = 0, covXZ = 0, covYY = 0, covYZ = 0, covZZ = 0;
                foreach (Point3d n in neighbors)
                {
                    double dx = n.X - meanX;
                    double dy = n.Y - meanY;
                    double dz = n.Z - meanZ;
                    covXX += dx * dx;
                    covXY += dx * dy;
                    covXZ += dx * dz;
                    covYY += dy * dy;
                    covYZ += dy * dz;
                    covZZ += dz * dz;
                }

                Vector3d rowA = new Vector3d(covXX, covXY, covXZ);
                Vector3d rowB = new Vector3d(covXY, covYY, covYZ);
                Vector3d rowC = new Vector3d(covXZ, covYZ, covZZ);

                Vector3d normal = Vector3d.CrossProduct(rowA, rowB);
                if (normal.Length < 1e-06)
                    normal = Vector3d.CrossProduct(rowA, rowC);
                if (normal.Length < 1e-06)
                    normal = Vector3d.ZAxis;

                normal.Unitize();
                normals[index] = normal;
            });

            return normals;
        }

        // ==================== RANSAC 拟合 ====================

        private static Plane FitPlaneRansac(List<Point3d> points, double threshold, int maxIterations,
            out List<Point3d> bestInliers)
        {
            bestInliers = new List<Point3d>();
            if (points.Count < 3)
                return Plane.Unset;

            Random random = new Random();
            int bestCount = 0;
            Plane bestPlane = Plane.Unset;

            for (int i = 0; i < maxIterations; i++)
            {
                int a = random.Next(points.Count);
                int b = random.Next(points.Count);
                int c = random.Next(points.Count);
                if (a == b || b == c || a == c)
                    continue;

                Plane candidate = new Plane(points[a], points[b], points[c]);
                if (!candidate.IsValid)
                    continue;

                List<Point3d> inliers = new List<Point3d>();
                foreach (Point3d p in points)
                {
                    if (Math.Abs(candidate.DistanceTo(p)) < threshold)
                        inliers.Add(p);
                }

                if (inliers.Count > bestCount)
                {
                    bestCount = inliers.Count;
                    bestInliers = inliers;
                    bestPlane = candidate;
                }
            }

            // 用内点做最小二乘精化
            Plane refined;
            if (bestInliers.Count >= 3 && Plane.FitPlaneToPoints(bestInliers, out refined) == PlaneFitResult.Success)
                return refined;
            return bestPlane;
        }

        private static Cylinder FitCylinderRansac(List<Point3d> points, Vector3d[] normals, double threshold,
            int maxIterations, out List<Point3d> bestInliers)
        {
            bestInliers = new List<Point3d>();
            int count = points.Count;
            if (count < 4)
                return Cylinder.Unset;

            Random random = new Random();
            int bestCount = 0;
            Line bestAxis = Line.Unset;

            for (int i = 0; i < maxIterations; i++)
            {
                int a = random.Next(count);
                int b = random.Next(count);
                if (a == b)
                    continue;

                Point3d pa = points[a];
                Vector3d na = normals[a];
                Point3d pb = points[b];
                Vector3d nb = normals[b];

                // 两法线叉积即轴向
                Vector3d axis = Vector3d.CrossProduct(na, nb);
                if (axis.Length < 0.001)
                    continue;
                axis.Unitize();

                Line rayA = new Line(pa, pa + (na * 100.0));
                Line rayB = new Line(pb, pb + (nb * 100.0));
                double tA, tB;
                if (!Intersection.LineLine(rayA, rayB, out tA, out tB, 0.0001, false))
                    continue;

                // 轴上中心点：两视线交点取平均
                Point3d center = (rayA.PointAt(tA) + rayB.PointAt(tB)) * 0.5;
                double radius = (pa.DistanceTo(center) + pb.DistanceTo(center)) * 0.5;
                if (radius < 0.001 || radius > 1000.0)
                    continue;

                Line axisLine = new Line(center - (axis * 1000.0), center + (axis * 1000.0));

                // 内点：到轴的距离与半径之差小于阈值
                List<Point3d> inliers = new List<Point3d>();
                for (int j = 0; j < count; j++)
                {
                    if (Math.Abs(axisLine.DistanceTo(points[j], false) - radius) < threshold)
                        inliers.Add(points[j]);
                }

                if (inliers.Count > bestCount)
                {
                    bestCount = inliers.Count;
                    bestInliers = inliers;
                    bestAxis = axisLine;
                }
            }

            if (bestInliers.Count < 4)
                return Cylinder.Unset;

            // 用内点精化半径与高度范围
            double minProjection = double.MaxValue;
            double maxProjection = double.MinValue;
            double radiusSum = 0.0;
            Vector3d direction = bestAxis.Direction;
            direction.Unitize();
            Point3d origin = bestAxis.From;

            foreach (Point3d p in bestInliers)
            {
                double projection = (p - origin) * direction;
                if (projection < minProjection) minProjection = projection;
                if (projection > maxProjection) maxProjection = projection;
                radiusSum += bestAxis.DistanceTo(p, false);
            }
            double refinedRadius = radiusSum / bestInliers.Count;

            Point3d baseCenter = origin + (direction * minProjection);
            double height = maxProjection > minProjection ? maxProjection - minProjection : 1.0;
            return new Cylinder(new Circle(new Plane(baseCenter, direction), refinedRadius), height);
        }

        private static Sphere FitSphereRansac(List<Point3d> points, Vector3d[] normals, double threshold,
            int maxIterations, out List<Point3d> bestInliers)
        {
            bestInliers = new List<Point3d>();
            int count = points.Count;
            if (count < 4)
                return Sphere.Unset;

            Random random = new Random();
            int bestCount = 0;
            Point3d bestCenter = Point3d.Unset;

            for (int i = 0; i < maxIterations; i++)
            {
                int a = random.Next(count);
                int b = random.Next(count);
                if (a == b)
                    continue;

                Point3d pa = points[a];
                Vector3d na = normals[a];
                Point3d pb = points[b];
                Vector3d nb = normals[b];

                // 两法线射线的交点即球心候选
                Line rayA = new Line(pa, pa + (na * 100.0));
                Line rayB = new Line(pb, pb + (nb * 100.0));
                double tA, tB;
                if (!Intersection.LineLine(rayA, rayB, out tA, out tB, 0.0001, false))
                    continue;

                Point3d center = (rayA.PointAt(tA) + rayB.PointAt(tB)) * 0.5;
                double radius = (pa.DistanceTo(center) + pb.DistanceTo(center)) * 0.5;
                if (radius < 0.001 || radius > 1000.0)
                    continue;

                List<Point3d> inliers = new List<Point3d>();
                for (int j = 0; j < count; j++)
                {
                    if (Math.Abs(points[j].DistanceTo(center) - radius) < threshold)
                        inliers.Add(points[j]);
                }

                if (inliers.Count > bestCount)
                {
                    bestCount = inliers.Count;
                    bestInliers = inliers;
                    bestCenter = center;
                }
            }

            if (bestInliers.Count < 4)
                return Sphere.Unset;

            // 平均半径精化
            double radiusSum = 0.0;
            foreach (Point3d p in bestInliers)
                radiusSum += p.DistanceTo(bestCenter);
            return new Sphere(bestCenter, radiusSum / bestInliers.Count);
        }

        // ==================== 命令主流程 ====================

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (SplatDisplayConduit.Instance == null || !SplatDisplayConduit.Instance.Enabled ||
                SplatDisplayConduit.Instance.Instances.Count == 0)
            {
                RhinoApp.WriteLine("Gaussian Splatting is not active. Please load a splat model first.");
                return Result.Failure;
            }

            SplatInstance targetInstance = SplatDisplayConduit.GetTargetInstance(
                doc, "Select splat proxy point cloud to fit shape to");
            if (targetInstance == null)
            {
                RhinoApp.WriteLine("No target splat dataset selected.");
                return Result.Cancel;
            }

            Splat[] splats = targetInstance.Splats;
            if (splats == null || splats.Length == 0)
            {
                RhinoApp.WriteLine("No splats loaded to fit geometry against.");
                return Result.Failure;
            }

            // 边界与拟合形状选项
            int shapeIndex = 0;
            int boundaryModeIndex = 0;
            GetObject go = new GetObject();
            go.GeometryFilter = ObjectType.Brep | ObjectType.Mesh | ObjectType.Curve | ObjectType.Extrusion;
            go.AcceptNothing(true);
            int shapeListIndex = go.AddOptionList("ShapeToFit", new string[] { "Plane", "Cylinder", "Sphere" }, shapeIndex);
            int boundaryListIndex = go.AddOptionList("BoundaryMode", new string[] { "DrawBox", "SelectObject" }, boundaryModeIndex);

            GeometryBase boundaryGeometry = null;
            Box boundaryBox = Box.Empty;
            bool useBox = false;

            while (true)
            {
                go.SetCommandPrompt("Select shape boundary object (or press Enter / choose DrawBox for aligned 2-point box)");
                GetResult result = go.Get();
                if (result == GetResult.Object)
                {
                    RhinoObject rhObj = go.Object(0).Object();
                    if (rhObj != null)
                        boundaryGeometry = rhObj.Geometry;
                    break;
                }
                if (result == GetResult.Option)
                {
                    CommandLineOption opt = go.Option();
                    if (opt != null)
                    {
                        if (opt.Index == shapeListIndex)
                            shapeIndex = opt.CurrentListOptionIndex;
                        else if (opt.Index == boundaryListIndex)
                            boundaryModeIndex = opt.CurrentListOptionIndex;
                    }
                    continue;
                }
                if (result == GetResult.Nothing || (result == GetResult.Cancel && boundaryModeIndex == 0))
                {
                    RhinoApp.WriteLine("Pick first corner, second corner (width/depth), and height for aligned boundary box:");
                    Box box;
                    Result boxResult = RhinoGet.GetBox(out box);
                    if (boxResult != Result.Success)
                        return boxResult;
                    boundaryBox = box;
                    useBox = true;
                    break;
                }
                return Result.Cancel;
            }

            // 转换边界为封闭网格
            Mesh boundaryMesh = null;
            BoundingBox boundaryBBox = BoundingBox.Empty;
            if (!useBox)
            {
                boundaryMesh = boundaryGeometry as Mesh;
                if (boundaryMesh == null)
                {
                    Brep brep = boundaryGeometry as Brep;
                    if (brep != null)
                        boundaryMesh = ConvertBrepToMesh(brep);
                    else
                    {
                        Extrusion extrusion = boundaryGeometry as Extrusion;
                        if (extrusion != null)
                            boundaryMesh = ConvertBrepToMesh(extrusion.ToBrep());
                        else
                        {
                            Curve curve = boundaryGeometry as Curve;
                            if (curve != null && curve.IsClosed)
                                boundaryMesh = ConvertCurveToMesh(curve, targetInstance.BBox);
                        }
                    }
                }
                if (boundaryMesh == null)
                {
                    RhinoApp.WriteLine("Failed to convert selection to a valid closed boundary mesh.");
                    return Result.Failure;
                }
                boundaryBBox = boundaryMesh.GetBoundingBox(true);
            }

            // 收集边界内的 splat 中心点
            List<Point3d> regionPoints = new List<Point3d>();
            for (int i = 0; i < splats.Length; i++)
            {
                Splat splat = splats[i];
                Point3d point = new Point3d(splat.X, splat.Y, splat.Z);
                bool inside = false;
                if (useBox)
                {
                    inside = boundaryBox.Contains(point);
                }
                else if (boundaryBBox.Contains(point))
                {
                    inside = boundaryMesh.IsPointInside(point, 1e-05, true);
                }
                if (inside)
                    regionPoints.Add(point);
            }

            if (regionPoints.Count < 4)
            {
                RhinoApp.WriteLine("Selection region does not contain enough splats (minimum 4 required).");
                return Result.Failure;
            }

            // RANSAC 参数
            double threshold = 0.05;
            int maxIterations = 200;
            OptionDouble thresholdOption = new OptionDouble(threshold, 0.001, 10.0);
            OptionInteger iterationsOption = new OptionInteger(maxIterations, 10, 2000);

            GetOption paramOption = new GetOption();
            paramOption.SetCommandPrompt("RANSAC Primitive Fitting Parameters");
            paramOption.AcceptNothing(true);
            paramOption.AddOptionDouble("Threshold", ref thresholdOption, "Inlier distance tolerance threshold");
            paramOption.AddOptionInteger("Iterations", ref iterationsOption, "Number of RANSAC trials");

            GetResult paramResult;
            do
            {
                paramResult = paramOption.Get();
                if (paramResult == GetResult.Cancel)
                    return Result.Cancel;
            }
            while (paramResult == GetResult.Option);

            threshold = thresholdOption.CurrentValue;
            maxIterations = iterationsOption.CurrentValue;

            string shapeName = shapeIndex == 0 ? "Plane" : (shapeIndex == 1 ? "Cylinder" : "Sphere");
            RhinoApp.WriteLine(string.Format(
                "Running RANSAC {0} fit on {1:N0} splats...", shapeName, regionPoints.Count));
            Stopwatch stopwatch = Stopwatch.StartNew();
            Guid addedObjectId = Guid.Empty;

            if (shapeIndex == 0)
            {
                // 平面拟合
                List<Point3d> inliers;
                Plane plane = FitPlaneRansac(regionPoints, threshold, maxIterations, out inliers);
                stopwatch.Stop();
                if (plane.IsValid && inliers.Count >= 3)
                {
                    // 求内点在平面参数域的范围
                    double minU = double.MaxValue, maxU = double.MinValue;
                    double minV = double.MaxValue, maxV = double.MinValue;
                    foreach (Point3d p in inliers)
                    {
                        double u, v;
                        plane.ClosestParameter(p, out u, out v);
                        if (u < minU) minU = u;
                        if (u > maxU) maxU = u;
                        if (v < minV) minV = v;
                        if (v > maxV) maxV = v;
                    }

                    PlaneSurface surface = new PlaneSurface(plane, new Interval(minU, maxU), new Interval(minV, maxV));
                    if (surface.IsValid)
                    {
                        addedObjectId = doc.Objects.AddSurface(surface);
                        RhinoApp.WriteLine(string.Format(
                            "Plane fitted in {0} ms. Matched {1} inliers of {2}.",
                            stopwatch.ElapsedMilliseconds, inliers.Count, regionPoints.Count));
                    }
                }
            }
            else
            {
                Vector3d[] normals = EstimateNormals(regionPoints);
                if (shapeIndex == 1)
                {
                    // 圆柱拟合
                    List<Point3d> inliers;
                    Cylinder cylinder = FitCylinderRansac(regionPoints, normals, threshold, maxIterations, out inliers);
                    stopwatch.Stop();
                    if (cylinder.IsValid && inliers.Count >= 4)
                    {
                        Brep cylinderBrep = cylinder.ToBrep(true, true);
                        if (cylinderBrep != null && cylinderBrep.IsValid)
                        {
                            addedObjectId = doc.Objects.AddBrep(cylinderBrep);
                            RhinoApp.WriteLine(string.Format(
                                "Cylinder fitted in {0} ms. Radius: {1:F3}, Height: {2:F3}. Matched {3} inliers.",
                                stopwatch.ElapsedMilliseconds, cylinder.Radius,
                                cylinder.Height1, inliers.Count));
                        }
                    }
                }
                else
                {
                    // 球体拟合
                    List<Point3d> inliers;
                    Sphere sphere = FitSphereRansac(regionPoints, normals, threshold, maxIterations, out inliers);
                    stopwatch.Stop();
                    if (sphere.IsValid && inliers.Count >= 4)
                    {
                        addedObjectId = doc.Objects.AddSphere(sphere);
                        RhinoApp.WriteLine(string.Format(
                            "Sphere fitted in {0} ms. Radius: {1:F3}. Matched {2} inliers.",
                            stopwatch.ElapsedMilliseconds, sphere.Radius, inliers.Count));
                    }
                }
            }

            if (addedObjectId != Guid.Empty)
            {
                doc.Objects.Select(addedObjectId);
                doc.Views.Redraw();
                return Result.Success;
            }

            RhinoApp.WriteLine("Failed to fit a valid " + shapeName + ". Points may be too sparse or noisy.");
            return Result.Failure;
        }
    }
}
