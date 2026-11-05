using Rhino;
using Rhino.Commands;
using Rhino.Input;
using Rhino.Input.Custom;
using System;

namespace RhinoPlugins_Development
{
    /// <summary>调整高斯泼溅渲染参数：整体缩放倍率与低通滤波半径。</summary>
    public class SplatSettingsCommand : Command
    {
        public override string EnglishName => "SplatSettings";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            GetOption option = new GetOption();
            option.SetCommandPrompt("Gaussian Splatting Settings");
            option.AcceptNothing(true);

            float originalScale = SplatDisplayConduit.SplatScale;
            float originalFilterRadius = SplatDisplayConduit.FilterRadius;

            OptionDouble scaleOption = new OptionDouble(originalScale);
            OptionDouble filterOption = new OptionDouble(originalFilterRadius);
            option.AddOptionDouble("Scale", ref scaleOption, "Splat scale multiplier");
            option.AddOptionDouble("FilterRadius", ref filterOption, "Splat low-pass filter radius");

            while (true)
            {
                GetResult result = option.Get();
                if (result == GetResult.Cancel)
                {
                    // 取消：恢复原始值
                    SplatDisplayConduit.SplatScale = originalScale;
                    SplatDisplayConduit.FilterRadius = originalFilterRadius;
                    doc.Views.Redraw();
                    return Result.Cancel;
                }

                if (result == GetResult.Option)
                {
                    // 校验取值范围
                    if (scaleOption.CurrentValue <= 0.0)
                    {
                        RhinoApp.WriteLine("Scale must be greater than zero.");
                        scaleOption.CurrentValue = originalScale;
                    }
                    if (filterOption.CurrentValue < 0.0)
                    {
                        RhinoApp.WriteLine("FilterRadius cannot be negative.");
                        filterOption.CurrentValue = originalFilterRadius;
                    }

                    SplatDisplayConduit.SplatScale = (float)scaleOption.CurrentValue;
                    SplatDisplayConduit.FilterRadius = (float)filterOption.CurrentValue;
                    doc.Views.Redraw();
                    continue;
                }

                // Enter（AcceptNothing）：应用当前设置
                SplatDisplayConduit.SplatScale = (float)scaleOption.CurrentValue;
                SplatDisplayConduit.FilterRadius = (float)filterOption.CurrentValue;
                doc.Views.Redraw();
                RhinoApp.WriteLine(string.Format(
                    "Gaussian Splat Settings applied. Scale: {0:F3}, FilterRadius: {1:F3}",
                    SplatDisplayConduit.SplatScale, SplatDisplayConduit.FilterRadius));
                return Result.Success;
            }
        }
    }
}
