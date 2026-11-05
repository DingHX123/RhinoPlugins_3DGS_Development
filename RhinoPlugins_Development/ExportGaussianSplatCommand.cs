using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Input;
using Rhino.Input.Custom;
using Rhino.UI;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace RhinoPlugins_Development
{
    /// <summary>将激活的高斯泼溅数据导出为 ply / splat / ksplat 文件（支持多实例合并导出）。</summary>
    public class ExportGaussianSplatCommand : Command
    {
        public override string EnglishName => "ExportGaussianSplat";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (SplatDisplayConduit.Instance == null || !SplatDisplayConduit.Instance.Enabled ||
                SplatDisplayConduit.Instance.Instances.Count == 0)
            {
                RhinoApp.WriteLine("No Gaussian Splat dataset is currently loaded or active to export.");
                return Result.Failure;
            }

            // 优先收集当前选中的代理点云对应的实例
            List<SplatInstance> selectedInstances = new List<SplatInstance>();
            foreach (RhinoObject rhinoObj in doc.Objects.GetSelectedObjects(false, false))
            {
                SplatInstance instance = SplatDisplayConduit.Instance.Instances.Find(
                    x => x.ProxyObjectId == rhinoObj.Id);
                if (instance != null && !selectedInstances.Contains(instance))
                    selectedInstances.Add(instance);
            }

            if (selectedInstances.Count == 0)
            {
                if (SplatDisplayConduit.Instance.Instances.Count == 1)
                {
                    selectedInstances.Add(SplatDisplayConduit.Instance.Instances[0]);
                }
                else
                {
                    GetObject go = new GetObject();
                    go.SetCommandPrompt("Select splat proxy point clouds to export (Press Enter to export all active)");
                    go.GeometryFilter = ObjectType.PointSet;
                    go.AcceptNothing(true);
                    go.SetCustomGeometryFilter((rhinoObject, geometry, componentIndex) =>
                        SplatDisplayConduit.Instance.Instances.Exists(x => x.ProxyObjectId == rhinoObject.Id));

                    GetResult getResult = go.GetMultiple(1, 0);
                    if (getResult == GetResult.Nothing)
                    {
                        selectedInstances.AddRange(SplatDisplayConduit.Instance.Instances);
                    }
                    else if (getResult == GetResult.Object)
                    {
                        for (int i = 0; i < go.ObjectCount; i++)
                        {
                            ObjRef objRef = go.Object(i);
                            SplatInstance instance = SplatDisplayConduit.Instance.Instances.Find(
                                x => x.ProxyObjectId == objRef.ObjectId);
                            if (instance != null && !selectedInstances.Contains(instance))
                                selectedInstances.Add(instance);
                        }
                    }
                    else
                    {
                        return Result.Cancel;
                    }
                }
            }

            if (selectedInstances.Count == 0)
            {
                RhinoApp.WriteLine("No target splat datasets selected.");
                return Result.Cancel;
            }

            // 合并所有实例的 splat 数据
            List<Splat> merged = new List<Splat>();
            foreach (SplatInstance instance in selectedInstances)
            {
                if (instance.Splats != null && instance.Splats.Length > 0)
                    merged.AddRange(instance.Splats);
            }
            Splat[] splats = merged.ToArray();
            if (splats.Length == 0)
            {
                RhinoApp.WriteLine("Selected splat dataset(s) are empty.");
                return Result.Failure;
            }

            // 保存对话框
            SaveFileDialog dialog = new SaveFileDialog();
            dialog.Filter = "Gaussian Splat PLY (*.ply)|*.ply|Compact Splat (*.splat)|*.splat|Compressed KSplat (*.ksplat)|*.ksplat";
            dialog.Title = "Export Gaussian Splat";
            dialog.FileName = "ExportedSplat.ply";
            dialog.DefaultExt = "ply";
            if (!dialog.ShowSaveDialog())
                return Result.Cancel;

            string path = dialog.FileName;
            string extension = Path.GetExtension(path).ToLowerInvariant();

            RhinoApp.WriteLine(string.Format(
                "Exporting {0:N0} splats to {1}...", splats.Length, Path.GetFileName(path)));
            Stopwatch stopwatch = Stopwatch.StartNew();
            string errorMessage = string.Empty;
            stopwatch.Stop();

            bool success;
            if (extension == ".splat")
                success = SplatLoader.ExportSplat(path, splats, out errorMessage);
            else if (extension == ".ksplat")
                success = SplatLoader.ExportKSplat(path, splats, out errorMessage);
            else
                success = SplatLoader.ExportPly(path, splats, out errorMessage);

            if (!success)
            {
                RhinoApp.WriteLine("Error exporting splats: " + errorMessage);
                return Result.Failure;
            }

            RhinoApp.WriteLine(string.Format(
                "Successfully exported {0:N0} splats in {1:F2} seconds.",
                splats.Length, stopwatch.ElapsedMilliseconds / 1000.0));
            return Result.Success;
        }
    }
}
