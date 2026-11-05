using Rhino;
using Rhino.Commands;
using Rhino.Input.Custom;
using System;

namespace RhinoPlugins_Development
{
    /// <summary>清除当前加载的高斯泼溅模型（支持单实例 / 选定实例 / 全部实例）。</summary>
    public class ClearGaussianSplatCommand : Command
    {
        public override string EnglishName => "ClearGaussianSplat";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (SplatDisplayConduit.Instance == null || !SplatDisplayConduit.Instance.Enabled ||
                SplatDisplayConduit.Instance.Instances.Count == 0)
            {
                RhinoApp.WriteLine("Gaussian Splatting is not active.");
                return Result.Success;
            }

            SplatDisplayConduit conduit = SplatDisplayConduit.Instance;
            if (conduit.Instances.Count == 1)
            {
                SplatInstance deletedInstance = conduit.Instances[0];
                SplatDisplayConduit.RegisterDeleteUndo(doc, "Clear Gaussian Splat", deletedInstance);
                conduit.RemoveInstance(deletedInstance, doc);
                RhinoApp.WriteLine("Gaussian Splatting model cleared.");
            }
            else
            {
                GetOption option = new GetOption();
                option.SetCommandPrompt("Gaussian Splatting has multiple active models.");
                int clearSelectedIndex = option.AddOption("ClearSelected");
                int clearAllIndex = option.AddOption("ClearAll");
                option.AddOption("Cancel");
                option.Get();
                if (option.CommandResult() != Result.Success)
                    return Result.Cancel;

                CommandLineOption selected = option.Option();
                if (selected != null)
                {
                    if (selected.Index == clearSelectedIndex)
                    {
                        SplatInstance targetInstance = SplatDisplayConduit.GetTargetInstance(
                            doc, "Select splat proxy point cloud to clear");
                        if (targetInstance == null)
                            return Result.Cancel;

                        SplatDisplayConduit.RegisterDeleteUndo(doc, "Clear Gaussian Splat", targetInstance);
                        conduit.RemoveInstance(targetInstance, doc);
                        RhinoApp.WriteLine("Selected splat model cleared.");
                    }
                    else if (selected.Index == clearAllIndex)
                    {
                        for (int i = conduit.Instances.Count - 1; i >= 0; i--)
                        {
                            SplatDisplayConduit.RegisterDeleteUndo(doc, "Clear Gaussian Splat", conduit.Instances[i]);
                        }
                        SplatDisplayConduit.DisableConduit();
                        RhinoApp.WriteLine("All Gaussian Splatting models cleared.");
                    }
                    else
                    {
                        return Result.Cancel;
                    }
                }
            }

            doc.Views.Redraw();
            return Result.Success;
        }
    }
}
