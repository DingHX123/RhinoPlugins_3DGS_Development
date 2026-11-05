using System;
using System.Collections.Generic;
using System.Windows;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace RhinoPlugins_Development
{
    public partial class GaussianPointPropertiesWindow : Window
    {
        private readonly GaussianPoint _gp;
        private readonly int _pointIndex;
        private readonly Point3d _rhinoPos;
        private readonly bool _useRhinoCenter;

        private static readonly Dictionary<string, string> PropertyDescriptions = new Dictionary<string, string>
        {
            { "x", "中心位置 X 坐标" },
            { "y", "中心位置 Y 坐标" },
            { "z", "中心位置 Z 坐标" },
            { "nx", "法向量 X 分量" },
            { "ny", "法向量 Y 分量" },
            { "nz", "法向量 Z 分量" },
            { "f_dc_0", "颜色 DC 分量 0 (R通道)" },
            { "f_dc_1", "颜色 DC 分量 1 (G通道)" },
            { "f_dc_2", "颜色 DC 分量 2 (B通道)" },
            { "opacity", "不透明度 (logit空间)" },
            { "scale_0", "缩放轴 0 (log空间)" },
            { "scale_1", "缩放轴 1 (log空间)" },
            { "scale_2", "缩放轴 2 (log空间)" },
            { "rot_0", "旋转四元数 w 分量" },
            { "rot_1", "旋转四元数 x 分量" },
            { "rot_2", "旋转四元数 y 分量" },
            { "rot_3", "旋转四元数 z 分量" },
        };

        public GaussianPointPropertiesWindow(int pointIndex, GaussianPoint gp, string plyPath)
            : this(pointIndex, gp, plyPath, Point3d.Unset, false)
        {
        }

        public GaussianPointPropertiesWindow(int pointIndex, GaussianPoint gp, string plyPath, Point3d rhinoPos, bool useRhinoCenter)
        {
            InitializeComponent();

            _gp = gp;
            _pointIndex = pointIndex;
            _rhinoPos = rhinoPos;
            _useRhinoCenter = useRhinoCenter;

            PointInfoText.Text = $"点索引：{pointIndex}  |  位置：({gp.X:F4}, {gp.Y:F4}, {gp.Z:F4})";
            FilePathText.Text = $"PLY: {plyPath}";

            var items = new List<PropertyItem>();
            foreach (var kv in gp.AllProperties)
            {
                string name = kv.Key;
                double rawValue = kv.Value;
                string description = GetDescription(name);
                string computedValue = GetComputedValue(name, rawValue);

                items.Add(new PropertyItem
                {
                    Name = name,
                    Description = description,
                    RawValue = rawValue.ToString("F6"),
                    ComputedValue = computedValue
                });
            }

            PropertiesDataGrid.ItemsSource = items;
        }

        private static string GetDescription(string propName)
        {
            if (PropertyDescriptions.TryGetValue(propName, out string desc))
                return desc;

            if (propName.StartsWith("f_rest_"))
                return $"球谐函数高阶分量 {propName.Substring(7)}";

            return propName;
        }

        private static string GetComputedValue(string propName, double rawValue)
        {
            if (propName.StartsWith("scale_"))
                return $"exp = {Math.Exp(rawValue):F6}";

            if (propName == "opacity")
                return $"sigmoid = {1.0 / (1.0 + Math.Exp(-rawValue)):F6}";

            if (propName.StartsWith("f_dc_"))
            {
                double shCoeff = 0.28209479177387814;
                double color = 0.5 + rawValue * shCoeff;
                return $"RGB = {color:F6}";
            }

            return "-";
        }

        private void OnCreateEllipsoid(object sender, RoutedEventArgs e)
        {
            var doc = RhinoDoc.ActiveDoc;
            if (doc == null)
            {
                MessageBox.Show("无法获取Rhino文档", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            Point3d center = _useRhinoCenter && _rhinoPos.IsValid ? _rhinoPos : new Point3d(_gp.X, _gp.Y, _gp.Z);
            double sigmaMultiplier = 3.0;

            Brep ellipsoidBrep = Create3dgsellipsoid.CreateEllipsoidBrep(_gp, center, sigmaMultiplier, _useRhinoCenter);
            if (ellipsoidBrep == null)
            {
                MessageBox.Show("椭球创建失败", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            var attributes = new ObjectAttributes();
            attributes.Name = $"3DGS_Ellipsoid_{_pointIndex}";

            double r = 0.5, g = 0.5, b = 0.5;
            double shCoeff = 0.28209479177387814;
            if (_gp.AllProperties.ContainsKey("f_dc_0"))
                r = 0.5 + _gp.AllProperties["f_dc_0"] * shCoeff;
            if (_gp.AllProperties.ContainsKey("f_dc_1"))
                g = 0.5 + _gp.AllProperties["f_dc_1"] * shCoeff;
            if (_gp.AllProperties.ContainsKey("f_dc_2"))
                b = 0.5 + _gp.AllProperties["f_dc_2"] * shCoeff;

            r = Clamp01(r);
            g = Clamp01(g);
            b = Clamp01(b);

            var displayColor = System.Drawing.Color.FromArgb(
                255,
                (int)(r * 255),
                (int)(g * 255),
                (int)(b * 255));

            attributes.ObjectColor = displayColor;
            attributes.ColorSource = ObjectColorSource.ColorFromObject;

            double opacity = 1.0;
            if (_gp.AllProperties.ContainsKey("opacity"))
            {
                double logit = _gp.AllProperties["opacity"];
                opacity = 1.0 / (1.0 + Math.Exp(-logit));
            }

            opacity = Clamp01(opacity);

            Guid objectId = doc.Objects.AddBrep(ellipsoidBrep, attributes);

            if (objectId != Guid.Empty)
            {
                int materialIndex = doc.Materials.Add();
                var material = doc.Materials[materialIndex];
                material.DiffuseColor = displayColor;
                material.Transparency = 1.0 - opacity;
                material.CommitChanges();

                var rhinoObj = doc.Objects.FindId(objectId);
                if (rhinoObj != null)
                {
                    rhinoObj.Attributes.MaterialSource = ObjectMaterialSource.MaterialFromObject;
                    rhinoObj.Attributes.MaterialIndex = materialIndex;
                    rhinoObj.CommitChanges();
                }

                doc.Views.Redraw();

                string colorHex = $"#{displayColor.R:X2}{displayColor.G:X2}{displayColor.B:X2}";
                RhinoApp.WriteLine($"已创建椭球：索引 {_pointIndex}，颜色 {colorHex}，透明度 {(1.0 - opacity):F3}，3σ半轴 ({_gp.ScaleX * sigmaMultiplier:F4}, {_gp.ScaleY * sigmaMultiplier:F4}, {_gp.ScaleZ * sigmaMultiplier:F4})");
                MessageBox.Show($"椭球已创建到Rhino场景中\n\n颜色：{colorHex}\n透明度：{(1.0 - opacity):F3}\n3σ半轴：({_gp.ScaleX * sigmaMultiplier:F4}, {_gp.ScaleY * sigmaMultiplier:F4}, {_gp.ScaleZ * sigmaMultiplier:F4})", "创建成功", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            else
            {
                MessageBox.Show("椭球添加到文档失败", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private static double Clamp01(double v)
        {
            if (v < 0) return 0;
            if (v > 1) return 1;
            return v;
        }

        private void OnClose(object sender, RoutedEventArgs e)
        {
            Close();
        }
    }

    public class PropertyItem
    {
        public string Name { get; set; }
        public string Description { get; set; }
        public string RawValue { get; set; }
        public string ComputedValue { get; set; }
    }
}