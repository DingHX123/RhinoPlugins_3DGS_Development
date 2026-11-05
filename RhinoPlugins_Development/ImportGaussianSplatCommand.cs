using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using Rhino.UI;
using System;
using System.Diagnostics;
using System.Drawing;
using System.IO;

namespace RhinoPlugins_Development
{
    /// <summary>3DGS 文件的上轴约定（对齐参考实现 SplatImporter.DefaultUpAxis）。</summary>
    public enum SplatUpAxis
    {
        ZUp,
        YUp,
        YDown
    }

    public class ImportGaussianSplatCommand : Command
    {
        private static Splat[] _loadedSplats;
        private static Guid _proxyObjectId = Guid.Empty;

        public static Splat[] LoadedSplats => _loadedSplats;
        public static Guid ProxyObjectId => _proxyObjectId;

        /// <summary>导入时按 Rhino Z-up 坐标系扶正模型（默认开启，对齐 KIRI 参考实现的绕 X -90° 扶正）。</summary>
        public static bool UprightOnImport { get; set; } = true;

        /// <summary>导入时自动剔除 floaters（AutoCrop），默认开启（对齐参考实现 GaussianSplatPlugIn 默认值）。</summary>
        public static bool AutoCropOnImport { get; set; } = true;

        /// <summary>导入时自动找平地面（LevelFloor），默认开启（对齐参考实现默认值）。</summary>
        public static bool LevelOnImport { get; set; } = true;

        public override string EnglishName => "ImportGaussianSplat";

        /// <summary>
        /// 绕世界 X 轴旋转 splat 数据（仅旋转位置与朝向四元数）。
        /// 缩放是旋转不变量，严禁改动——否则椭球形状与朝向被破坏，渲染出针状伪影。
        /// </summary>
        public static void RotateSplatsAboutX(Splat[] splats, float cosA, float sinA)
        {
            if (splats == null) return;

            // 半角四元数 q_rot = (cos(θ/2), sin(θ/2), 0, 0)，θ = atan2(sinA, cosA)
            float cosHalf = (float)Math.Sqrt(Math.Max(0.0, 0.5 * (1.0 + cosA)));
            float sinHalf = Math.Sign(sinA) * (float)Math.Sqrt(Math.Max(0.0, 0.5 * (1.0 - cosA)));

            for (int i = 0; i < splats.Length; i++)
            {
                Splat s = splats[i];

                float newY = s.Y * cosA - s.Z * sinA;
                float newZ = s.Y * sinA + s.Z * cosA;
                splats[i].Y = newY;
                splats[i].Z = newZ;

                float qw = s.RotW;
                float qx = s.RotX;
                float qy = s.RotY;
                float qz = s.RotZ;

                // 左乘世界系旋转四元数，保证与位置旋转严格对齐；缩放保持不变
                float rQw = qw * cosHalf - qx * sinHalf;
                float rQx = qx * cosHalf + qw * sinHalf;
                float rQy = qy * cosHalf - qz * sinHalf;
                float rQz = qz * cosHalf + qy * sinHalf;

                float norm = rQw * rQw + rQx * rQx + rQy * rQy + rQz * rQz;
                if (norm > 1e-6f)
                {
                    float invNorm = 1f / (float)Math.Sqrt(norm);
                    rQw *= invNorm;
                    rQx *= invNorm;
                    rQy *= invNorm;
                    rQz *= invNorm;
                }
                else
                {
                    rQw = 1f; rQx = 0f; rQy = 0f; rQz = 0f;
                }

                splats[i].RotW = rQw;
                splats[i].RotX = rQx;
                splats[i].RotY = rQy;
                splats[i].RotZ = rQz;
            }
        }

        /// <summary>按文件扩展名给出默认上轴：.spz 为 YUp，其余为 YDown（与参考实现一致）。</summary>
        public static SplatUpAxis DefaultUpAxis(string filename)
        {
            string ext = Path.GetExtension(filename ?? string.Empty).ToLowerInvariant();
            if (ext == ".spz") return SplatUpAxis.YUp;
            return SplatUpAxis.YDown;
        }

        /// <summary>应用上轴约定（ZUp 为恒等变换，即 .las 默认不旋转）。</summary>
        public static void ApplyUpAxis(Splat[] splats, SplatUpAxis axis)
        {
            switch (axis)
            {
                case SplatUpAxis.YDown:
                    RotateSplatsAboutX(splats, 0f, -1f);
                    break;
                case SplatUpAxis.YUp:
                    RotateSplatsAboutX(splats, 0f, 1f);
                    break;
            }
        }

        /// <summary>按 Keep 掩码过滤 splat 数组（掩码为 null 或长度不符时原样返回）。</summary>
        public static Splat[] FilterSplats(Splat[] splats, bool[] keep)
        {
            if (splats == null || keep == null || keep.Length != splats.Length)
                return splats;

            int kept = 0;
            for (int i = 0; i < keep.Length; i++)
            {
                if (keep[i]) kept++;
            }
            if (kept == splats.Length)
                return splats;

            Splat[] result = new Splat[kept];
            int j = 0;
            for (int i = 0; i < splats.Length; i++)
            {
                if (keep[i]) result[j++] = splats[i];
            }
            return result;
        }

        /// <summary>
        /// 对 splat 数据应用仿射变换：位置直接变换，朝向四元数左乘线性部分的旋转后归一化。
        /// 找平变换的线性部分为纯旋转，缩放是旋转不变量保持不变（与 SplatInstance 的矩阵转四元数一致）。
        /// </summary>
        public static void TransformSplats(Splat[] splats, Transform t)
        {
            if (splats == null || splats.Length == 0)
                return;

            // 旋转矩阵 -> 四元数（Shepperd 方法）
            double qw, qx, qy, qz;
            double trace = t.M00 + t.M11 + t.M22;
            if (trace > 0.0)
            {
                double s = Math.Sqrt(trace + 1.0) * 2.0;
                qw = 0.25 * s;
                qx = (t.M21 - t.M12) / s;
                qy = (t.M02 - t.M20) / s;
                qz = (t.M10 - t.M01) / s;
            }
            else if (t.M00 > t.M11 && t.M00 > t.M22)
            {
                double s = Math.Sqrt(1.0 + t.M00 - t.M11 - t.M22) * 2.0;
                qw = (t.M21 - t.M12) / s;
                qx = 0.25 * s;
                qy = (t.M01 + t.M10) / s;
                qz = (t.M02 + t.M20) / s;
            }
            else if (t.M11 > t.M22)
            {
                double s = Math.Sqrt(1.0 + t.M11 - t.M00 - t.M22) * 2.0;
                qw = (t.M02 - t.M20) / s;
                qx = (t.M01 + t.M10) / s;
                qy = 0.25 * s;
                qz = (t.M12 + t.M21) / s;
            }
            else
            {
                double s = Math.Sqrt(1.0 + t.M22 - t.M00 - t.M11) * 2.0;
                qw = (t.M10 - t.M01) / s;
                qx = (t.M02 + t.M20) / s;
                qy = (t.M12 + t.M21) / s;
                qz = 0.25 * s;
            }

            for (int i = 0; i < splats.Length; i++)
            {
                Splat s = splats[i];
                Point3d pt = new Point3d(s.X, s.Y, s.Z);
                pt.Transform(t);

                // 左乘世界系旋转四元数，保证与位置变换严格对齐
                double w = qw, x = qx, y = qy, z = qz;
                double rw = s.RotW, rx = s.RotX, ry = s.RotY, rz = s.RotZ;

                float nQw = (float)(w * rw - x * rx - y * ry - z * rz);
                float nQx = (float)(w * rx + x * rw + y * rz - z * ry);
                float nQy = (float)(w * ry - x * rz + y * rw + z * rx);
                float nQz = (float)(w * rz + x * ry - y * rx + z * rw);

                float norm = nQw * nQw + nQx * nQx + nQy * nQy + nQz * nQz;
                if (norm > 1e-6f)
                {
                    float invNorm = 1f / (float)Math.Sqrt(norm);
                    nQw *= invNorm;
                    nQx *= invNorm;
                    nQy *= invNorm;
                    nQz *= invNorm;
                }
                else
                {
                    nQw = 1f; nQx = 0f; nQy = 0f; nQz = 0f;
                }

                splats[i].X = (float)pt.X;
                splats[i].Y = (float)pt.Y;
                splats[i].Z = (float)pt.Z;
                splats[i].RotW = nQw;
                splats[i].RotX = nQx;
                splats[i].RotY = nQy;
                splats[i].RotZ = nQz;
            }
        }

        public static bool ImportSplatFile(string filename, RhinoDoc doc, bool interactive, SplatUpAxis? upAxis = null)
        {
            if (doc == null || string.IsNullOrEmpty(filename))
                return false;

            if (!File.Exists(filename))
            {
                RhinoApp.WriteLine("File not found: " + filename);
                return false;
            }

            string errorMessage;
            string ext = Path.GetExtension(filename).ToLowerInvariant();
            Splat[] splats = null;

            switch (ext)
            {
                case ".ply":
                    splats = SplatLoader.LoadPly(filename, out errorMessage);
                    break;
                case ".splat":
                    splats = SplatLoader.LoadSplat(filename, out errorMessage);
                    break;
                case ".ksplat":
                    splats = SplatLoader.LoadKSplat(filename, out errorMessage);
                    break;
                case ".spz":
                    splats = SpzLoader.LoadSpz(filename, out errorMessage);
                    break;
                case ".las":
                    splats = LasLoader.LoadLas(filename, out errorMessage);
                    break;
                default:
                    splats = SplatLoader.LoadPly(filename, out errorMessage);
                    break;
            }

            if (splats == null || splats.Length == 0)
            {
                RhinoApp.WriteLine("Failed to load splat file: " + (errorMessage ?? "Unknown error"));
                return false;
            }

            // 是否为真正的 splat 数据（AutoCrop 仅对 splat 有意义；普通点云 PLY 与 .las 跳过）
            bool isSplatData;
            switch (ext)
            {
                case ".splat":
                case ".ksplat":
                case ".spz":
                    isSplatData = true;
                    break;
                case ".las":
                    isSplatData = false;
                    break;
                default:
                    isSplatData = SplatLoader.LastPlyWasSplat;
                    break;
            }

            SplatUpAxis defaultAxis = ext == ".las" ? SplatUpAxis.ZUp : DefaultUpAxis(filename);
            bool upright = UprightOnImport;
            bool autoCrop = AutoCropOnImport;
            bool level = LevelOnImport;

            if (interactive && upAxis == null)
            {
                // 导入前可选功能：Upright（按 Rhino 坐标系扶正）/ AutoCrop / LevelFloor
                GetOption importOptions = new GetOption();
                importOptions.SetCommandPrompt("Import options");
                importOptions.AcceptNothing(true);
                OptionToggle uprightToggle = new OptionToggle(ext != ".las" && upright, "Off", "On");
                OptionToggle autoCropToggle = new OptionToggle(autoCrop, "Off", "On");
                OptionToggle levelToggle = new OptionToggle(level, "Off", "On");
                if (ext != ".las")
                    importOptions.AddOptionToggle("Upright", ref uprightToggle);
                if (isSplatData)
                    importOptions.AddOptionToggle("AutoCrop", ref autoCropToggle);
                importOptions.AddOptionToggle("LevelFloor", ref levelToggle);

                GetResult getResult;
                do
                {
                    getResult = importOptions.Get();
                }
                while (getResult == GetResult.Option);

                // 开关的最终值由 RhinoCommon 通过 OptionToggle 回写
                upright = uprightToggle.CurrentValue;
                autoCrop = autoCropToggle.CurrentValue;
                level = levelToggle.CurrentValue;

                if (importOptions.CommandResult() != Result.Success)
                    return false;
            }

            // 上轴：扶正时按扩展名默认轴（.spz→YUp，其余→YDown）转到 Rhino Z-up；不扶正或 .las 则原样载入
            SplatUpAxis axis = upAxis ?? (upright && ext != ".las" ? defaultAxis : SplatUpAxis.ZUp);

            // 1) AutoCrop：在原始数据上先剔除 floaters（对齐参考实现顺序）
            bool autoCropApplied = false;
            if (autoCrop && isSplatData)
            {
                RhinoApp.WriteLine("Auto-cropping floaters...");
                SplatAutoCropResult crop = SplatAutoCrop.Compute(splats, SplatCropStrength.Medium);
                if (crop != null && crop.Kept > 0)
                {
                    int before = splats.Length;
                    splats = FilterSplats(splats, crop.Keep);
                    autoCropApplied = splats.Length < before;
                    RhinoApp.WriteLine($"AutoCrop kept {splats.Length:N0} of {before:N0} splats ({100.0 * splats.Length / before:F1}%).");
                }
                else
                {
                    RhinoApp.WriteLine("AutoCrop: nothing to remove.");
                }
            }

            // 2) 上轴转换（.las 默认 ZUp，不旋转）
            if (ext != ".las")
                ApplyUpAxis(splats, axis);

            // 3) LevelFloor：RANSAC 找平地面并居中（确定性算法，文档重开时可复现）
            bool leveled = false;
            if (level)
            {
                RhinoApp.WriteLine("Finding floor plane...");
                Transform leveling;
                double tiltDegrees;
                if (SplatFloorLeveler.TryLevel(splats, out leveling, out tiltDegrees))
                {
                    TransformSplats(splats, leveling);
                    leveled = true;
                    RhinoApp.WriteLine($"Leveled floor (tilt {tiltDegrees:F2} deg).");
                }
                else
                {
                    RhinoApp.WriteLine("No floor plane found; skipping leveling.");
                }
            }

            RhinoApp.WriteLine($"Loaded {splats.Length:N0} splats from file.");

            if (interactive && SplatDisplayConduit.Instance != null && SplatDisplayConduit.Instance.Enabled)
            {
                GetOption option = new GetOption();
                option.SetCommandPrompt("Gaussian Splatting is already active.");
                int clearIdx = option.AddOption("ClearExisting");
                option.AddOption("LoadNew");
                int cancelIdx = option.AddOption("Cancel");
                option.Get();

                if (option.CommandResult() != Result.Success)
                    return false;

                CommandLineOption opt = option.Option();
                if (opt != null)
                {
                    if (opt.Index == clearIdx)
                    {
                        ClearLoadedSplats(doc);
                        RhinoApp.WriteLine("Existing splat data cleared.");
                    }
                    else if (opt.Index == cancelIdx)
                    {
                        return false;
                    }
                }
            }

            PointCloud cloud = new PointCloud();
            int step = Math.Max(1, splats.Length / 50000);
            for (int i = 0; i < splats.Length; i += step)
            {
                Splat splat = splats[i];
                cloud.Add(new Point3d(splat.X, splat.Y, splat.Z), Color.FromArgb(splat.R, splat.G, splat.B));
            }

            while (cloud.Count < 3)
            {
                cloud.Add(new Point3d(0, 0, 0), Color.Black);
            }

            Guid proxyId = doc.Objects.AddPointCloud(cloud);
            if (proxyId == Guid.Empty)
            {
                RhinoApp.WriteLine("Failed to add proxy point cloud to document.");
                return false;
            }

            RhinoObject proxyObj = doc.Objects.FindId(proxyId);
            if (proxyObj != null)
            {
                proxyObj.Attributes.SetUserString("SplatFilePath", filename);
                proxyObj.Attributes.SetUserString("SplatCount", splats.Length.ToString());
                // 持久化导入处理设置：文档重开时 ScanAndRestoreSplats 据此复现同样的确定性处理
                proxyObj.Attributes.SetUserString("SplatUpAxis", axis.ToString());
                proxyObj.Attributes.SetUserString("SplatAutoCrop", autoCropApplied ? "True" : "False");
                proxyObj.Attributes.SetUserString("SplatLevelFloor", leveled ? "True" : "False");
                if (cloud.Count >= 3)
                {
                    proxyObj.Attributes.SetUserString("OrigP0", FormatPoint(cloud[0].Location));
                    proxyObj.Attributes.SetUserString("OrigP1", FormatPoint(cloud[1].Location));
                    proxyObj.Attributes.SetUserString("OrigP2", FormatPoint(cloud[2].Location));
                }
                proxyObj.CommitChanges();
            }

            _loadedSplats = splats;
            _proxyObjectId = proxyId;

            // 启用 GPU 泲溅渲染管道（代理点云不再直接绘制，由 splat 渲染替代）
            SplatDisplayConduit.EnableConduit(splats, proxyId);
            if (SplatDisplayConduit.Instance != null && SplatDisplayConduit.Instance.Instances.Count > 0)
            {
                SplatInstance importedInstance = SplatDisplayConduit.Instance.Instances[SplatDisplayConduit.Instance.Instances.Count - 1];
                SplatDisplayConduit.RegisterImportUndo(doc, "Import Gaussian Splat", importedInstance);
            }

            doc.Views.Redraw();
            return true;
        }

        public static void ClearLoadedSplats(RhinoDoc doc)
        {
            // 通过显示管道清理（会同时删除代理点云并释放 GPU 资源）
            SplatDisplayConduit.DisableConduit();
            _loadedSplats = null;
            _proxyObjectId = Guid.Empty;
        }

        private static string FormatPoint(Point3d pt)
        {
            return $"{pt.X:R};{pt.Y:R};{pt.Z:R}";
        }

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (doc == null)
                return Result.Failure;

            if (SplatDisplayConduit.Instance != null && SplatDisplayConduit.Instance.Enabled)
            {
                GetOption option = new GetOption();
                option.SetCommandPrompt("Gaussian Splatting is already active.");
                int clearIdx = option.AddOption("ClearExisting");
                option.AddOption("LoadNew");
                int cancelIdx = option.AddOption("Cancel");
                option.Get();

                if (option.CommandResult() != Result.Success)
                    return Result.Cancel;

                CommandLineOption opt = option.Option();
                if (opt != null)
                {
                    if (opt.Index == clearIdx)
                    {
                        ClearLoadedSplats(doc);
                        RhinoApp.WriteLine("Existing splat data cleared.");
                    }
                    else if (opt.Index == cancelIdx)
                    {
                        return Result.Cancel;
                    }
                }
            }

            OpenFileDialog dialog = new OpenFileDialog();
            dialog.Filter = "Gaussian Splat PLY (*.ply)|*.ply|Compact Splat (*.splat)|*.splat|Compressed KSplat (*.ksplat)|*.ksplat|SPZ Compressed (*.spz)|*.spz|LAS Point Cloud (*.las)|*.las|All Supported|*.ply;*.splat;*.ksplat;*.spz;*.las";
            dialog.Title = "Import Gaussian Splat";

            if (!dialog.ShowOpenDialog())
                return Result.Nothing;

            string filename = dialog.FileName;
            if (string.IsNullOrEmpty(filename) || !File.Exists(filename))
                return Result.Nothing;

            Stopwatch sw = Stopwatch.StartNew();
            bool success = ImportSplatFile(filename, doc, true);
            sw.Stop();

            if (success)
            {
                RhinoApp.WriteLine($"Import completed in {sw.Elapsed.TotalSeconds:F2}s.");
                return Result.Success;
            }

            RhinoApp.WriteLine("Import failed.");
            return Result.Failure;
        }
    }
}