using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Threading.Tasks;

namespace RhinoPlugins_Development
{
    /// <summary>统计离群点去除：基于体素网格哈希 + 27 邻域邻居计数，删除孤立 splat。</summary>
    public class CleanSplatNoiseCommand : Command
    {
        public override string EnglishName => "CleanSplatNoise";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (SplatDisplayConduit.Instance == null || !SplatDisplayConduit.Instance.Enabled ||
                SplatDisplayConduit.Instance.Instances.Count == 0)
            {
                RhinoApp.WriteLine("Gaussian Splatting is not active. Please load a splat model first.");
                return Result.Failure;
            }

            SplatInstance targetInstance = SplatDisplayConduit.GetTargetInstance(
                doc, "Select splat proxy point cloud to clean noise from");
            if (targetInstance == null)
            {
                RhinoApp.WriteLine("No target splat dataset selected.");
                return Result.Failure;
            }

            Splat[] currentSplats = targetInstance.Splats;
            if (currentSplats == null || currentSplats.Length == 0)
            {
                RhinoApp.WriteLine("Active splat dataset is empty.");
                return Result.Failure;
            }

            // 参数选项循环
            double searchRadius = 0.5;
            int minNeighbors = 6;
            OptionDouble radiusOption = new OptionDouble(searchRadius, 0.001, 100.0);
            OptionInteger neighborsOption = new OptionInteger(minNeighbors, 1, 1000);

            GetResult result;
            do
            {
                GetOption option = new GetOption();
                option.SetCommandPrompt("Splat Noise Cleaning Options");
                option.AcceptNothing(true);
                option.AddOptionDouble("SearchRadius", ref radiusOption, "Search boundary radius for neighbor sampling");
                option.AddOptionInteger("MinNeighbors", ref neighborsOption, "Minimum nearby neighbors to retain a splat");
                result = option.Get();
                if (result == GetResult.Cancel)
                    return Result.Cancel;
            }
            while (result == GetResult.Option);

            searchRadius = radiusOption.CurrentValue;
            minNeighbors = neighborsOption.CurrentValue;

            RhinoApp.WriteLine(string.Format("Running statistical outlier removal on {0:N0} splats...", currentSplats.Length));
            Stopwatch stopwatch = Stopwatch.StartNew();

            // 构建体素网格哈希
            double voxelSize = searchRadius;
            if (voxelSize <= 0.0)
                voxelSize = 0.1;

            Dictionary<long, List<int>> grid = new Dictionary<long, List<int>>();
            for (int i = 0; i < currentSplats.Length; i++)
            {
                Splat splat = currentSplats[i];
                long vx = (long)Math.Floor(splat.X / voxelSize);
                long vy = (long)Math.Floor(splat.Y / voxelSize);
                long vz = (long)Math.Floor(splat.Z / voxelSize);
                long hash = vx;
                hash = (hash * 0x18dL) ^ vy;
                hash = (hash * 0x18dL) ^ vz;
                List<int> bucket;
                if (!grid.TryGetValue(hash, out bucket))
                {
                    bucket = new List<int>();
                    grid[hash] = bucket;
                }
                bucket.Add(i);
            }

            // 并行统计 27 邻域内的邻居数
            bool[] keepFlags = new bool[currentSplats.Length];
            double radiusSquared = searchRadius * searchRadius;

            Parallel.For(0, currentSplats.Length, index =>
            {
                Splat splat = currentSplats[index];
                long vx = (long)Math.Floor(splat.X / voxelSize);
                long vy = (long)Math.Floor(splat.Y / voxelSize);
                long vz = (long)Math.Floor(splat.Z / voxelSize);

                int neighborCount = 0;
                for (long dx = -1; dx <= 1; dx++)
                {
                    for (long dy = -1; dy <= 1; dy++)
                    {
                        for (long dz = -1; dz <= 1; dz++)
                        {
                            long hash = vx + dx;
                            hash = (hash * 0x18dL) ^ (vy + dy);
                            hash = (hash * 0x18dL) ^ (vz + dz);
                            List<int> bucket;
                            if (!grid.TryGetValue(hash, out bucket))
                                continue;

                            foreach (int otherIndex in bucket)
                            {
                                if (otherIndex == index)
                                    continue;
                                Splat other = currentSplats[otherIndex];
                                double diffX = splat.X - other.X;
                                double diffY = splat.Y - other.Y;
                                double diffZ = splat.Z - other.Z;
                                if (diffX * diffX + diffY * diffY + diffZ * diffZ <= radiusSquared)
                                    neighborCount++;
                            }
                        }
                    }
                }
                keepFlags[index] = neighborCount >= minNeighbors;
            });

            List<Splat> keptList = new List<Splat>();
            for (int i = 0; i < currentSplats.Length; i++)
            {
                if (keepFlags[i])
                    keptList.Add(currentSplats[i]);
            }
            stopwatch.Stop();

            Splat[] afterSplats = keptList.ToArray();
            if (afterSplats.Length == 0)
            {
                RhinoApp.WriteLine("Outlier removal would delete all splats. Operation cancelled.");
                return Result.Cancel;
            }

            // 注册撤销 + 写修改缓存 + 更新代理点云
            string previousFilePath = doc.Objects.FindId(targetInstance.ProxyObjectId)?
                .Attributes.GetUserString("SplatFilePath");
            SplatDisplayConduit.RegisterSplatsModifiedUndo(
                doc, "Clean Splat Noise", targetInstance.ProxyObjectId,
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

            targetInstance.SetSplats(afterSplats);
            if (p0 != Point3d.Unset)
            {
                targetInstance.OrigP0 = p0;
                targetInstance.OrigP1 = p1;
                targetInstance.OrigP2 = p2;
            }

            RhinoApp.WriteLine(string.Format(
                "Noise cleaning completed in {0:F2} seconds. Kept {1:N0} splats of {2:N0} (removed {3:N0}).",
                stopwatch.ElapsedMilliseconds / 1000.0,
                afterSplats.Length, currentSplats.Length,
                currentSplats.Length - afterSplats.Length));

            doc.Views.Redraw();
            return Result.Success;
        }
    }
}
