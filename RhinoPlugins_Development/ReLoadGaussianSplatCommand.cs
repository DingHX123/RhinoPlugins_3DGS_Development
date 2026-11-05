using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using System;
using System.Collections.Generic;

namespace RhinoPlugins_Development
{
    /// <summary>
    /// 手动重载文档中尚未载入 splat 数据的高斯泼溅代理点云。
    /// 适用于保存文档后重新打开时自动恢复未生效的场景（如插件晚于文档加载导致
    /// EndOpenDocument 事件未触发）：扫描带 SplatFilePath / IsConvertedSplat 标记
    /// 且当前未挂接渲染实例的点云，复用 ScanAndRestoreSplats 的确定性恢复链
    /// （文件回退查找 + AutoCrop → UpAxis → LevelFloor 按导入设置复现）。
    /// </summary>
    public class ReLoadGaussianSplatCommand : Command
    {
        public override string EnglishName => "ReLoadGaussianSplat";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (doc == null)
                return Result.Failure;

            // 收集"是 splat 代理但尚未挂接渲染实例"的点云对象
            List<RhinoObject> unloaded = new List<RhinoObject>();
            RhinoObject[] pointClouds = doc.Objects.FindByObjectType(ObjectType.PointSet);
            if (pointClouds != null)
            {
                foreach (RhinoObject obj in pointClouds)
                {
                    if (obj == null || obj.IsDeleted)
                        continue;
                    if (SplatDisplayConduit.Instance != null &&
                        SplatDisplayConduit.Instance.Instances.Exists(x => x.ProxyObjectId == obj.Id))
                        continue; // 已在渲染管道中，跳过

                    string splatFilePath = obj.Attributes.GetUserString("SplatFilePath");
                    string isConverted = obj.Attributes.GetUserString("IsConvertedSplat");
                    if (!string.IsNullOrEmpty(splatFilePath) || isConverted == "True")
                        unloaded.Add(obj);
                }
            }

            if (unloaded.Count == 0)
            {
                RhinoApp.WriteLine("No unloaded Gaussian Splat proxy point clouds found in the document.");
                return Result.Nothing;
            }

            RhinoApp.WriteLine($"Found {unloaded.Count} unloaded Gaussian Splat proxy point cloud(s). Restoring...");

            // ScanAndRestoreSplats 会跳过已载入的代理，只恢复未挂接实例的代理
            int before = SplatDisplayConduit.Instance != null ? SplatDisplayConduit.Instance.Instances.Count : 0;
            SplatDisplayConduit.ScanAndRestoreSplats(doc);
            int after = SplatDisplayConduit.Instance != null ? SplatDisplayConduit.Instance.Instances.Count : 0;
            int restored = after - before;

            if (restored > 0)
            {
                RhinoApp.WriteLine($"ReLoad Gaussian Splat: restored {restored} of {unloaded.Count} splat model(s).");
                doc.Views.Redraw();
                return Result.Success;
            }

            RhinoApp.WriteLine("ReLoad Gaussian Splat: nothing was restored (see messages above).");
            return Result.Failure;
        }
    }
}
