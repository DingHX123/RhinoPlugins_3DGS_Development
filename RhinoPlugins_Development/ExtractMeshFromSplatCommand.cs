using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using Rhino.UI;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;

namespace RhinoPlugins_Development
{
    /// <summary>从高斯泼溅数据中提取网格（Surface Nets 等值面提取，后台线程执行并显示进度条）。</summary>
    public class ExtractMeshFromSplatCommand : Command
    {
        private static volatile bool _running;

        public override string EnglishName => "ExtractMeshFromSplat";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (_running)
            {
                RhinoApp.WriteLine("Mesh extraction is already running.");
                return Result.Failure;
            }

            if (SplatDisplayConduit.Instance == null || !SplatDisplayConduit.Instance.Enabled ||
                SplatDisplayConduit.Instance.Instances.Count == 0)
            {
                RhinoApp.WriteLine("Gaussian Splatting is not active. Please load a splat model first.");
                return Result.Failure;
            }

            // 优先收集选中的实例
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
                    go.SetCommandPrompt("Select splat proxy point clouds to extract mesh from (Press Enter to mesh all active)");
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

            // 合并 splat 数据
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

            // 网格化参数
            int resolution = 384;
            int kernel = 5;
            int smooth = 15;
            double levelBias = 0.4;
            int minComponent = 500;
            double minOpacity = 0.02;
            double colorSharpen = 2.0;

            OptionInteger resolutionOption = new OptionInteger(resolution, 64, 768);
            OptionInteger kernelOption = new OptionInteger(kernel, 1, 10);
            OptionInteger smoothOption = new OptionInteger(smooth, 0, 100);
            OptionDouble levelOption = new OptionDouble(levelBias, 0.01, 2.0);
            OptionInteger minCompOption = new OptionInteger(minComponent, 0, 100000);
            OptionDouble minOpacityOption = new OptionDouble(minOpacity, 0.0, 1.0);
            OptionDouble sharpenOption = new OptionDouble(colorSharpen, 1.0, 10.0);

            GetResult result;
            do
            {
                GetOption option = new GetOption();
                option.SetCommandPrompt("Splat Meshing Options (Enter to run)");
                option.AcceptNothing(true);
                option.AddOptionInteger("Resolution", ref resolutionOption);
                option.AddOptionInteger("Kernel", ref kernelOption);
                option.AddOptionInteger("Smooth", ref smoothOption);
                option.AddOptionDouble("LevelBias", ref levelOption);
                option.AddOptionInteger("MinComponent", ref minCompOption);
                option.AddOptionDouble("MinOpacity", ref minOpacityOption);
                option.AddOptionDouble("ColorSharpen", ref sharpenOption);
                result = option.Get();
                if (result == GetResult.Cancel)
                    return Result.Cancel;
            }
            while (result == GetResult.Option);

            resolution = resolutionOption.CurrentValue;
            kernel = kernelOption.CurrentValue;
            smooth = smoothOption.CurrentValue;
            levelBias = levelOption.CurrentValue;
            minComponent = minCompOption.CurrentValue;
            minOpacity = minOpacityOption.CurrentValue;
            colorSharpen = sharpenOption.CurrentValue;

            // 后台执行
            _running = true;
            RhinoApp.WriteLine(string.Format("Starting Surface Nets on {0:N0} splats...", splats.Length));
            StatusBar.ShowProgressMeter(0, 100, "Extracting Mesh...", true, true);
            Stopwatch watch = Stopwatch.StartNew();

            int res = resolution;
            int ker = kernel;
            int smo = smooth;
            int mc = minComponent;
            float lvl = (float)levelBias;
            float mo = (float)minOpacity;
            float cs = (float)colorSharpen;
            Splat[] meshSplats = splats;

            Task.Run(() =>
            {
                Mesh resultMesh = null;
                string error = null;
                try
                {
                    resultMesh = SplatToMesh.Execute(meshSplats, res, ker, smo, lvl, mc, mo, cs,
                        pct => RhinoApp.InvokeOnUiThread((Action)(() =>
                            StatusBar.UpdateProgressMeter((int)(pct * 100f), true))));
                }
                catch (Exception ex)
                {
                    error = ex.ToString();
                }

                RhinoApp.InvokeOnUiThread((Action)(() =>
                {
                    watch.Stop();
                    StatusBar.HideProgressMeter();
                    _running = false;

                    if (error != null)
                    {
                        RhinoApp.WriteLine("Exception during extraction:");
                        RhinoApp.WriteLine(error);
                        return;
                    }

                    if (resultMesh == null)
                    {
                        RhinoApp.WriteLine("SplatToMesh returned null - no iso-surface was found.");
                        RhinoApp.WriteLine("Try: lower LevelBias (e.g. 0.1), raise Kernel (e.g. 7), lower MinOpacity (e.g. 0.01).");
                        return;
                    }

                    RhinoApp.WriteLine(string.Format("Raw mesh: {0:N0} verts, {1:N0} faces",
                        resultMesh.Vertices.Count, resultMesh.Faces.Count));

                    if (resultMesh.Vertices.Count == 0 || resultMesh.Faces.Count == 0)
                    {
                        RhinoApp.WriteLine("Mesh has no geometry.");
                        return;
                    }

                    resultMesh.Vertices.CombineIdentical(true, true);
                    resultMesh.Vertices.CullUnused();
                    resultMesh.Faces.CullDegenerateFaces();
                    resultMesh.RebuildNormals();
                    resultMesh.Compact();

                    if (!resultMesh.IsValid)
                    {
                        string log;
                        resultMesh.IsValidWithLog(out log);
                        RhinoApp.WriteLine("Mesh validation warning: " + log);
                    }

                    RhinoDoc activeDoc = RhinoDoc.ActiveDoc;
                    if (activeDoc == null)
                    {
                        RhinoApp.WriteLine("No active document.");
                        return;
                    }

                    Guid meshId = activeDoc.Objects.AddMesh(resultMesh);
                    if (meshId == Guid.Empty)
                    {
                        BoundingBox bbox = resultMesh.GetBoundingBox(true);
                        RhinoApp.WriteLine("AddMesh failed. Diagnostics:");
                        RhinoApp.WriteLine(string.Format("  BoundingBox valid = {0},  Min={1},  Max={2}",
                            bbox.IsValid, bbox.Min, bbox.Max));
                        RhinoApp.WriteLine(string.Format("  IsValid = {0}", resultMesh.IsValid));
                        RhinoApp.WriteLine(string.Format("  Vertices = {0}, Faces = {1}",
                            resultMesh.Vertices.Count, resultMesh.Faces.Count));
                        RhinoApp.WriteLine(string.Format("  VertexColors = {0}", resultMesh.VertexColors.Count));
                    }
                    else
                    {
                        activeDoc.Objects.Select(meshId);
                        activeDoc.Views.Redraw();
                        RhinoApp.WriteLine(string.Format("Done: {0:N0} verts, {1:N0} faces, {2:F2}s",
                            resultMesh.Vertices.Count, resultMesh.Faces.Count,
                            watch.ElapsedMilliseconds / 1000.0));
                    }
                }));
            });

            return Result.Success;
        }
    }
}
