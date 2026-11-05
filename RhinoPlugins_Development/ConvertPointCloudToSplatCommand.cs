using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input.Custom;
using System;
using System.Drawing;

namespace RhinoPlugins_Development
{
    /// <summary>把选中的 Rhino 点云转换为高斯泼溅模型并启用实时渲染（自适应估计 splat 尺度）。</summary>
    public class ConvertPointCloudToSplatCommand : Command
    {
        public override string EnglishName => "ConvertPointCloudToSplat";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            // 若已有激活的 splat，询问处理方式
            if (SplatDisplayConduit.Instance != null && SplatDisplayConduit.Instance.Enabled)
            {
                GetOption option = new GetOption();
                option.SetCommandPrompt("Gaussian Splatting is already active.");
                int clearIndex = option.AddOption("ClearExisting");
                option.AddOption("LoadNew");
                int cancelIndex = option.AddOption("Cancel");
                option.Get();
                if (option.CommandResult() != Result.Success)
                    return Result.Cancel;

                CommandLineOption selected = option.Option();
                if (selected != null)
                {
                    if (selected.Index == clearIndex)
                    {
                        SplatDisplayConduit.DisableConduit();
                        RhinoApp.WriteLine("Gaussian Splatting conduit disabled and cleared.");
                    }
                    else if (selected.Index == cancelIndex)
                    {
                        return Result.Cancel;
                    }
                }
            }

            // 选择点云
            GetObject go = new GetObject();
            go.SetCommandPrompt("Select PointCloud to convert to Gaussian Splats");
            go.GeometryFilter = ObjectType.PointSet;
            go.Get();
            if (go.CommandResult() != Result.Success)
                return go.CommandResult();

            RhinoObject rhinoObj = go.Object(0).Object();
            if (rhinoObj == null)
            {
                RhinoApp.WriteLine("Failed to get Rhino object.");
                return Result.Failure;
            }

            PointCloud cloud = go.Object(0).Geometry() as PointCloud;
            if (cloud == null)
            {
                RhinoApp.WriteLine("Selected object is not a PointCloud geometry.");
                return Result.Failure;
            }

            int pointCount = cloud.Count;
            if (pointCount == 0)
            {
                RhinoApp.WriteLine("Selected PointCloud has 0 points.");
                return Result.Failure;
            }

            RhinoApp.WriteLine(string.Format("Converting {0:N0} points to Gaussian Splats...", pointCount));
            Splat[] splats = new Splat[pointCount];
            for (int i = 0; i < pointCount; i++)
            {
                PointCloudItem item = cloud[i];
                Point3d location = item.Location;
                Color color = item.Color;

                // 无有效颜色的点使用中灰色
                byte r = 180, g = 180, b = 180;
                if (color.A > 0 || color.R > 0 || color.G > 0 || color.B > 0)
                {
                    r = color.R;
                    g = color.G;
                    b = color.B;
                }

                splats[i] = new Splat
                {
                    X = (float)location.X,
                    Y = (float)location.Y,
                    Z = (float)location.Z,
                    ScaleX = 1f,
                    ScaleY = 1f,
                    ScaleZ = 1f,
                    RotW = 1f,
                    RotX = 0f,
                    RotY = 0f,
                    RotZ = 0f,
                    R = r,
                    G = g,
                    B = b,
                    A = 255
                };
            }

            // 自适应尺度
            float adaptiveScale = SplatLoader.EstimatePointCloudScale(splats);
            RhinoApp.WriteLine(string.Format(
                "Adaptive splat scale: {0:F4} (based on density)", adaptiveScale));
            for (int j = 0; j < pointCount; j++)
            {
                splats[j].ScaleX = adaptiveScale;
                splats[j].ScaleY = adaptiveScale;
                splats[j].ScaleZ = adaptiveScale;
            }

            try
            {
                rhinoObj.Attributes.SetUserString("IsConvertedSplat", "True");
                if (cloud.Count >= 3)
                {
                    rhinoObj.Attributes.SetUserString("OrigP0", SplatDisplayConduit.FormatPoint(cloud[0].Location));
                    rhinoObj.Attributes.SetUserString("OrigP1", SplatDisplayConduit.FormatPoint(cloud[1].Location));
                    rhinoObj.Attributes.SetUserString("OrigP2", SplatDisplayConduit.FormatPoint(cloud[2].Location));
                }
                rhinoObj.CommitChanges();

                SplatDisplayConduit.EnableConduit(splats, rhinoObj.Id);
                if (SplatDisplayConduit.Instance != null && SplatDisplayConduit.Instance.Instances.Count > 0)
                {
                    SplatInstance importedInstance =
                        SplatDisplayConduit.Instance.Instances[SplatDisplayConduit.Instance.Instances.Count - 1];
                    SplatDisplayConduit.RegisterImportUndo(doc, "Convert PointCloud to Splat", importedInstance);
                }

                RhinoApp.WriteLine("Splat rendering enabled for selected PointCloud.");
                doc.Views.Redraw();
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine("Error enabling display conduit: " + ex.Message);
                return Result.Failure;
            }

            return Result.Success;
        }
    }
}
