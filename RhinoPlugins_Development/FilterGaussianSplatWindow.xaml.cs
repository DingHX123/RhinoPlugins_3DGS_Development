using System;
using System.Globalization;
using System.Windows;
using System.Windows.Media;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace RhinoPlugins_Development
{
    /// <summary>
    /// FilterGaussianSplat 参数窗口：显示选中高斯椭球的颜色/透明度等参数，
    /// 提供颜色变化阈值、搜索放大倍率与最低透明度设置，应用后仅显示递归过滤
    /// 得到的椭球集合（可撤销，可一键恢复全部）。
    /// </summary>
    public partial class FilterGaussianSplatWindow : Window
    {
        private readonly RhinoDoc _doc;
        private readonly SplatInstance _instance;
        private readonly Splat[] _fullSplats;   // 打开窗口时的完整数据（重复过滤始终基于它）
        private readonly int _seedIndex;

        public FilterGaussianSplatWindow(RhinoDoc doc, SplatInstance instance, Splat[] fullSplats, int seedIndex)
        {
            InitializeComponent();
            _doc = doc;
            _instance = instance;
            _fullSplats = fullSplats;
            _seedIndex = seedIndex;

            Splat s = fullSplats[seedIndex];
            SeedInfoText.Text = string.Format("选中椭球 #{0}（当前共 {1} 个）", seedIndex, fullSplats.Length);
            SeedColorSwatch.Background = new SolidColorBrush(Color.FromRgb(s.R, s.G, s.B));
            SeedColorText.Text = string.Format("RGB ({0}, {1}, {2})    透明度 {3:F3}", s.R, s.G, s.B, s.A / 255.0);
            SeedDetailText.Text = string.Format(
                "位置 ({0:F3}, {1:F3}, {2:F3})\n3σ 半轴 ({3:F4}, {4:F4}, {5:F4})    四元数 ({6:F3}, {7:F3}, {8:F3}, {9:F3})",
                s.X, s.Y, s.Z, s.ScaleX, s.ScaleY, s.ScaleZ, s.RotW, s.RotX, s.RotY, s.RotZ);
        }

        private bool TryReadParams(out double colorThreshold, out double multiplier, out double minOpacity, out string error)
        {
            colorThreshold = 0.0;
            multiplier = 1.0;
            minOpacity = 0.0;
            error = null;

            if (!double.TryParse(ColorThresholdBox.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out colorThreshold)
                || colorThreshold < 0.0)
            {
                error = "颜色变化阈值必须是 0-100 之间的数值";
                return false;
            }
            if (colorThreshold > 100.0)
            {
                colorThreshold = 100.0;   // 超过上限自动钳制为 100
                ColorThresholdBox.Text = "100";
            }
            if (!double.TryParse(SearchMultiplierBox.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out multiplier)
                || multiplier < 1.0)
            {
                error = "搜索放大倍率必须 ≥ 1";
                return false;
            }
            if (!double.TryParse(MinOpacityBox.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out minOpacity)
                || minOpacity < 0.0 || minOpacity > 1.0)
            {
                error = "最低透明度必须是 0-1 之间的数值";
                return false;
            }
            return true;
        }

        private void OnApplyFilter(object sender, RoutedEventArgs e)
        {
            double colorThreshold, multiplier, minOpacity;
            string error;
            if (!TryReadParams(out colorThreshold, out multiplier, out minOpacity, out error))
            {
                MessageBox.Show(error, "参数无效", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            try
            {
                int acceptedCount;
                Splat[] after = FilterGaussianSplatCommand.BuildFilteredSplats(
                    _fullSplats, _seedIndex, colorThreshold, multiplier, minOpacity, out acceptedCount);
                if (after.Length == 0)
                {
                    MessageBox.Show("没有满足条件的椭球（可放宽阈值/降低最低透明度）", "过滤结果为空",
                        MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                ApplySplatSet(after);
                ResultText.Text = string.Format(
                    "过滤后显示 {0} / {1} 个椭球（扩散 {2} 层）。可继续调整参数后再次应用；Ctrl+Z 可撤销。",
                    after.Length, _fullSplats.Length, acceptedCount);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "过滤失败", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void OnRestoreAll(object sender, RoutedEventArgs e)
        {
            Splat[] before = _instance.Splats;
            if (before != null && before.Length == _fullSplats.Length)
            {
                ResultText.Text = "当前已是全部椭球";
                return;
            }

            try
            {
                ApplySplatSet(_fullSplats);
                ResultText.Text = string.Format("已恢复全部 {0} 个椭球", _fullSplats.Length);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "恢复失败", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        /// <summary>用指定椭球集合替换当前显示（注册撤销 + 写缓存 + 同步代理点云，与 Crop 管线一致）。</summary>
        private void ApplySplatSet(Splat[] after)
        {
            Splat[] before = _instance.Splats;
            Guid proxyId = _instance.ProxyObjectId;

            string previousFilePath = _doc.Objects.FindId(proxyId)?.Attributes.GetUserString("SplatFilePath");
            SplatDisplayConduit.RegisterSplatsModifiedUndo(
                _doc, "Filter Gaussian Splat", proxyId, before, after, previousFilePath);

            string cachePath = SplatDisplayConduit.SaveModifiedSplatCache(proxyId, after);

            // 同步代理点云（保留三个基准点用于跟踪用户变换）
            PointCloud cloud = new PointCloud();
            int step = Math.Max(1, after.Length / 50000);
            for (int i = 0; i < after.Length; i += step)
            {
                Splat s = after[i];
                cloud.Add(new Point3d(s.X, s.Y, s.Z), System.Drawing.Color.FromArgb(s.R, s.G, s.B));
            }
            while (cloud.Count < 3)
                cloud.Add(new Point3d(0, 0, 0), System.Drawing.Color.Black);

            Point3d p0 = cloud[0].Location;
            Point3d p1 = cloud[1].Location;
            Point3d p2 = cloud[2].Location;

            _doc.Objects.Replace(proxyId, cloud);
            RhinoObject proxyObj = _doc.Objects.FindId(proxyId);
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

            SplatDisplayConduit.Instance.UpdateInstanceSplats(proxyId, after, p0, p1, p2);
            _doc.Views.Redraw();
        }

        private void OnClose(object sender, RoutedEventArgs e)
        {
            Close();
        }
    }
}
