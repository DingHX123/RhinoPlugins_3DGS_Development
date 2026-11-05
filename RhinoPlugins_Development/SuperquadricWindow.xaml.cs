using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;

namespace RhinoPlugins_Development
{
    /// <summary>
    /// 超二次曲面参数（Barr 参数化）：
    /// ((|x/a|^(2/ε2) + |y/b|^(2/ε2))^(ε2/ε1) + |z/c|^(2/ε1))^(ε1/2) = 1
    /// </summary>
    public class SuperquadricParams
    {
        public double AxisA = 1.0;
        public double AxisB = 1.0;
        public double AxisC = 1.0;
        public double Epsilon1 = 1.0;   // 纵向（纬线方向）指数
        public double Epsilon2 = 1.0;   // 横向（经线方向）指数
        public int SegmentsU = 64;      // 经向网格段数
        public int SegmentsV = 32;      // 纬向网格段数
        public double CenterX, CenterY, CenterZ;
        public double RotationXDeg, RotationYDeg, RotationZDeg;

        public static SuperquadricParams Default()
        {
            return new SuperquadricParams();
        }
    }

    /// <summary>
    /// CreateSuperquadric 参数窗口（非模态）：
    /// 任意参数改动后经短延时重建网格，提交给预览 conduit 在 Rhino 视口中实时绘制；
    /// 点「创建」才把当前参数生成的网格正式加入文档，点「关闭」丢弃预览。
    /// </summary>
    public partial class SuperquadricWindow : Window
    {
        /// <summary>预览网格分辨率上限（U/V），避免高分辨率设置导致拖动卡顿。</summary>
        private const int PreviewMaxU = 128;
        private const int PreviewMaxV = 64;

        private readonly RhinoDoc _doc;
        private readonly SuperquadricPreviewConduit _conduit;
        private readonly DispatcherTimer _rebuildTimer;

        /// <summary>滑杆与数值框双向同步期间置位，避免互相触发形成回环。</summary>
        private bool _syncing;

        internal SuperquadricWindow(RhinoDoc doc, SuperquadricPreviewConduit conduit)
        {
            InitializeComponent();

            _doc = doc;
            _conduit = conduit;

            // 输入防抖：连续输入或拖动滑杆时只在停顿后重建一次，避免每次变化都重算网格
            _rebuildTimer = new DispatcherTimer();
            _rebuildTimer.Interval = TimeSpan.FromMilliseconds(60);
            _rebuildTimer.Tick += OnRebuildTimerTick;

            // 滑杆 ↔ 数值框双向绑定（初始化完成后再挂，避免构造期间触发预览）
            Bind(AxisASlider, AxisABox, false);
            Bind(AxisBSlider, AxisBBox, false);
            Bind(AxisCSlider, AxisCBox, false);
            Bind(Eps1Slider, Eps1Box, false);
            Bind(Eps2Slider, Eps2Box, false);
            Bind(SegUSlider, SegUBox, true);
            Bind(SegVSlider, SegVBox, true);
            Bind(CenterXSlider, CenterXBox, false);
            Bind(CenterYSlider, CenterYBox, false);
            Bind(CenterZSlider, CenterZBox, false);
            Bind(RotXSlider, RotXBox, false);
            Bind(RotYSlider, RotYBox, false);
            Bind(RotZSlider, RotZBox, false);

            Closed += OnWindowClosed;

            UpdatePreview(silent: true);
        }

        /// <summary>
        /// 滑杆与数值框双向同步：
        /// 拖动滑杆 → 数值框显示当前值；在数值框输入 → 滑杆跟随（超范围时钳制到滑杆区间，
        /// 但保留用户输入的数值文本，以便仍可用超出滑杆范围的精确值创建）。
        /// </summary>
        private void Bind(Slider slider, TextBox box, bool isInteger)
        {
            slider.ValueChanged += (s, e) =>
            {
                if (_syncing)
                    return;
                _syncing = true;
                box.Text = FormatNumber(slider.Value, isInteger);
                _syncing = false;
                SchedulePreview();
            };

            box.TextChanged += (s, e) =>
            {
                if (_syncing)
                    return;

                double v;
                if (double.TryParse(box.Text.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out v))
                {
                    _syncing = true;
                    slider.Value = Math.Min(Math.Max(v, slider.Minimum), slider.Maximum);
                    _syncing = false;
                }
                SchedulePreview();
            };
        }

        private static string FormatNumber(double v, bool isInteger)
        {
            return isInteger
                ? ((int)Math.Round(v)).ToString(CultureInfo.InvariantCulture)
                : v.ToString("0.###", CultureInfo.InvariantCulture);
        }

        /// <summary>参数变化后延迟重建预览（防抖），避免拖动滑杆时每帧重算网格。</summary>
        private void SchedulePreview()
        {
            _rebuildTimer.Stop();
            _rebuildTimer.Start();
        }

        private void OnRebuildTimerTick(object sender, EventArgs e)
        {
            _rebuildTimer.Stop();
            UpdatePreview(silent: true);
        }

        /// <summary>预设切换：把常见形状指数写入 ε1/ε2 输入框（会连带触发预览刷新）。</summary>
        private void OnPresetChanged(object sender, SelectionChangedEventArgs e)
        {
            if (Eps1Box == null || Eps2Box == null)
                return; // XAML 解析过程中 SelectedIndex 触发时其余控件尚未创建

            switch (PresetBox.SelectedIndex)
            {
                case 1:
                    Eps1Box.Text = "0.15";
                    Eps2Box.Text = "0.15";
                    break;
                case 2:
                    Eps1Box.Text = "2";
                    Eps2Box.Text = "2";
                    break;
                case 3:
                    Eps1Box.Text = "0.5";
                    Eps2Box.Text = "0.5";
                    break;
                default:
                    Eps1Box.Text = "1";
                    Eps2Box.Text = "1";
                    break;
            }
        }

        private static bool TryParseDouble(TextBox box, string name, double min, double max,
            out double value, out string error)
        {
            value = 0.0;
            error = null;
            if (!double.TryParse(box.Text.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out value))
            {
                error = string.Format("{0} 必须是数值（当前输入：\"{1}\"）", name, box.Text);
                return false;
            }
            if (value < min || value > max)
            {
                error = string.Format("{0} 必须在 {1} ~ {2} 之间（当前输入：{3}）", name, min, max, value);
                return false;
            }
            return true;
        }

        /// <summary>读取并校验全部参数；失败时返回 false 并给出中文错误提示。</summary>
        internal bool TryReadParams(out SuperquadricParams p, out string error)
        {
            p = new SuperquadricParams();
            error = null;

            if (!TryParseDouble(AxisABox, "半轴 a", 1e-6, 1e6, out p.AxisA, out error)) return false;
            if (!TryParseDouble(AxisBBox, "半轴 b", 1e-6, 1e6, out p.AxisB, out error)) return false;
            if (!TryParseDouble(AxisCBox, "半轴 c", 1e-6, 1e6, out p.AxisC, out error)) return false;
            if (!TryParseDouble(Eps1Box, "形状指数 ε1", 0.05, 4.0, out p.Epsilon1, out error)) return false;
            if (!TryParseDouble(Eps2Box, "形状指数 ε2", 0.05, 4.0, out p.Epsilon2, out error)) return false;

            int segU, segV;
            if (!int.TryParse(SegUBox.Text.Trim(), out segU) || segU < 8 || segU > 512)
            {
                error = "经向分辨率 U 必须是 8 ~ 512 之间的整数";
                return false;
            }
            if (!int.TryParse(SegVBox.Text.Trim(), out segV) || segV < 8 || segV > 512)
            {
                error = "纬向分辨率 V 必须是 8 ~ 512 之间的整数";
                return false;
            }
            p.SegmentsU = segU;
            p.SegmentsV = segV;

            double cx, cy, cz, rx, ry, rz;
            if (!TryParseDouble(CenterXBox, "中心坐标 X", -1e6, 1e6, out cx, out error)) return false;
            if (!TryParseDouble(CenterYBox, "中心坐标 Y", -1e6, 1e6, out cy, out error)) return false;
            if (!TryParseDouble(CenterZBox, "中心坐标 Z", -1e6, 1e6, out cz, out error)) return false;
            if (!TryParseDouble(RotXBox, "旋转角度 X", -3600, 3600, out rx, out error)) return false;
            if (!TryParseDouble(RotYBox, "旋转角度 Y", -3600, 3600, out ry, out error)) return false;
            if (!TryParseDouble(RotZBox, "旋转角度 Z", -3600, 3600, out rz, out error)) return false;

            p.CenterX = cx;
            p.CenterY = cy;
            p.CenterZ = cz;
            p.RotationXDeg = rx;
            p.RotationYDeg = ry;
            p.RotationZDeg = rz;
            return true;
        }

        /// <summary>
        /// 用当前输入重建预览网格并交给 conduit 绘制。
        /// silent=true 时参数非法（如输入过程产生的空串）只清除预览、不弹窗。
        /// </summary>
        private void UpdatePreview(bool silent)
        {
            SuperquadricParams p;
            string error;
            if (!TryReadParams(out p, out error))
            {
                _conduit.SetMesh(null);
                if (silent)
                    StatusText.Text = "预览暂停：" + error;
                _doc.Views.Redraw();
                return;
            }

            // 预览限制分辨率，保证连续拖动参数时仍流畅
            p.SegmentsU = Math.Min(p.SegmentsU, PreviewMaxU);
            p.SegmentsV = Math.Min(p.SegmentsV, PreviewMaxV);

            Mesh mesh = SuperquadricMeshBuilder.Build(p);
            _conduit.SetMesh(mesh);
            StatusText.Text = string.Format("预览：a={0:F2} b={1:F2} c={2:F2} · ε1={3:F2} ε2={4:F2} · 预览网格 {5}×{6}",
                p.AxisA, p.AxisB, p.AxisC, p.Epsilon1, p.Epsilon2, p.SegmentsU, p.SegmentsV);

            _doc.Views.Redraw();
        }

        private void OnCreate(object sender, RoutedEventArgs e)
        {
            SuperquadricParams p;
            string error;
            if (!TryReadParams(out p, out error))
            {
                MessageBox.Show(error, "参数无效", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            string name = string.Format(
                "Superquadric_a{0:F2}_b{1:F2}_c{2:F2}_e1{3:F2}_e2{4:F2}",
                p.AxisA, p.AxisB, p.AxisC, p.Epsilon1, p.Epsilon2);

            if (OutputTypeBox.SelectedIndex == 1)
                CreateNurbs(p, name);
            else
                CreateMesh(p, name);

            Close();
        }

        /// <summary>以 Mesh 网格方式创建（使用用户设置的真实分辨率）。</summary>
        private void CreateMesh(SuperquadricParams p, string name)
        {
            Mesh mesh = SuperquadricMeshBuilder.Build(p);
            if (mesh == null || !mesh.IsValid)
            {
                MessageBox.Show("网格生成失败，请检查参数", "创建失败",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            var attributes = new ObjectAttributes();
            attributes.Name = name;

            if (_doc.Objects.AddMesh(mesh, attributes) == Guid.Empty)
            {
                MessageBox.Show("网格加入文档失败", "创建失败",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            _doc.Views.Redraw();
            RhinoApp.WriteLine(string.Format(
                "已创建 Mesh 超二次曲面：a={0:F3}, b={1:F3}, c={2:F3}, ε1={3:F2}, ε2={4:F2}，" +
                "网格 {5} 顶点 / {6} 面",
                p.AxisA, p.AxisB, p.AxisC, p.Epsilon1, p.Epsilon2,
                mesh.Vertices.Count, mesh.Faces.Count));
        }

        /// <summary>以 NURBS 曲面方式创建（控制点网格，3 阶，可继续用曲面控制点编辑）。</summary>
        private void CreateNurbs(SuperquadricParams p, string name)
        {
            NurbsSurface srf = SuperquadricNurbsBuilder.Build(p);
            if (srf == null || !srf.IsValid)
            {
                MessageBox.Show("NURBS 曲面生成失败，请尝试调整形状指数或降低分辨率", "创建失败",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            var attributes = new ObjectAttributes();
            attributes.Name = name;

            if (_doc.Objects.AddSurface(srf, attributes) == Guid.Empty)
            {
                MessageBox.Show("曲面加入文档失败", "创建失败",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            _doc.Views.Redraw();
            RhinoApp.WriteLine(string.Format(
                "已创建 NURBS 超二次曲面：a={0:F3}, b={1:F3}, c={2:F3}, ε1={3:F2}, ε2={4:F2}，" +
                "控制点 {5}×{6}（3 阶非有理）",
                p.AxisA, p.AxisB, p.AxisC, p.Epsilon1, p.Epsilon2,
                srf.Points.CountU, srf.Points.CountV));
        }

        /// <summary>关闭窗口：停止预览并释放 conduit（只影响预览，不改动文档中已创建的对象）。</summary>
        private void OnCancel(object sender, RoutedEventArgs e)
        {
            Close();
        }

        // ===================== 布尔运算（交集 / 并集 / 差集） =====================

        private enum BooleanOpKind { Union, Intersection, Difference }

        private void OnBooleanUnion(object sender, RoutedEventArgs e)
        {
            RunBooleanWithPicker(BooleanOpKind.Union);
        }

        private void OnBooleanIntersection(object sender, RoutedEventArgs e)
        {
            RunBooleanWithPicker(BooleanOpKind.Intersection);
        }

        private void OnBooleanDifference(object sender, RoutedEventArgs e)
        {
            RunBooleanWithPicker(BooleanOpKind.Difference);
        }

        /// <summary>
        /// 点击按钮后直接进入交互选择：
        /// - 并集 / 交集：一次多选全部参与者（至少 2 个）
        /// - 差集：先选被剪模型（主体 A），再选裁剪模型（B、C…），结果为 A - B - C…
        /// 选完立即运算并生成结果，结果继承主体对象的图层、材质与颜色。
        /// </summary>
        private void RunBooleanWithPicker(BooleanOpKind kind)
        {
            string opName = kind == BooleanOpKind.Union ? "并集"
                          : kind == BooleanOpKind.Intersection ? "交集"
                          : "差集";

            var picked = new List<RhinoObject>();
            RhinoObject mainObj = null;

            // 选择期间临时隐藏预览网格，避免与待选实体混淆
            _conduit.SetMesh(null);
            _doc.Views.Redraw();

            try
            {
                if (kind == BooleanOpKind.Difference)
                {
                    var getMain = new GetObject();
                    getMain.SetCommandPrompt("选择被剪模型（差集主体 A）");
                    getMain.GeometryFilter = ObjectType.Mesh | ObjectType.Brep;
                    getMain.SubObjectSelect = false;
                    getMain.EnablePreSelect(false, true);
                    if (getMain.Get() != GetResult.Object)
                    {
                        Warn("差集已取消");
                        return;
                    }

                    mainObj = getMain.Object(0).Object();
                    picked.Add(mainObj);

                    var getTool = new GetObject();
                    getTool.SetCommandPrompt("选择裁剪模型（可多选，回车完成）");
                    getTool.GeometryFilter = ObjectType.Mesh | ObjectType.Brep;
                    getTool.SubObjectSelect = false;
                    getTool.EnablePreSelect(false, true);
                    if (getTool.GetMultiple(1, 0) != GetResult.Object)
                    {
                        Warn("差集已取消");
                        return;
                    }
                    for (int i = 0; i < getTool.ObjectCount; i++)
                        picked.Add(getTool.Object(i).Object());
                }
                else
                {
                    var getSet = new GetObject();
                    getSet.SetCommandPrompt(kind == BooleanOpKind.Union
                        ? "选择参与并集的实体（可多选，回车完成）"
                        : "选择参与交集的实体（可多选，回车完成）");
                    getSet.GeometryFilter = ObjectType.Mesh | ObjectType.Brep;
                    getSet.SubObjectSelect = false;
                    getSet.EnablePreSelect(false, true);
                    if (getSet.GetMultiple(2, 0) != GetResult.Object)
                    {
                        Warn(opName + "已取消");
                        return;
                    }
                    for (int i = 0; i < getSet.ObjectCount; i++)
                        picked.Add(getSet.Object(i).Object());
                    mainObj = picked[0];
                }
            }
            catch (Exception ex)
            {
                Warn("无法启动选择交互：" + ex.Message);
                return;
            }
            finally
            {
                UpdatePreview(silent: true);   // 恢复预览
            }

            if (picked.Count < 2 || mainObj == null)
            {
                Warn(string.Format("{0}至少需要 2 个实体（当前 {1} 个）", opName, picked.Count));
                return;
            }

            ExecuteBoolean(kind, opName, picked, mainObj);
        }

        /// <summary>执行布尔运算并落盘结果；结果属性继承自 mainObj（图层/材质/颜色）。</summary>
        private void ExecuteBoolean(BooleanOpKind kind, string opName, List<RhinoObject> picked, RhinoObject mainObj)
        {
            var meshes = new List<Mesh>();
            var breps = new List<Brep>();
            int skipped = 0;
            bool mainAccepted = false;

            foreach (RhinoObject o in picked)
            {
                bool accepted = false;
                if (o.ObjectType == ObjectType.Mesh)
                {
                    Mesh m = o.Geometry as Mesh;
                    if (m != null && m.IsClosed)
                    {
                        meshes.Add(m);
                        accepted = true;
                    }
                }
                else
                {
                    Brep b = o.Geometry as Brep;
                    if (b != null && b.IsSolid)
                    {
                        breps.Add(b);
                        accepted = true;
                    }
                }
                if (!accepted)
                    skipped++;
                if (accepted && o == mainObj)
                    mainAccepted = true;
            }

            if (!mainAccepted)
            {
                Warn(kind == BooleanOpKind.Difference
                    ? "被剪模型必须是封闭实体（Mesh 需封闭、NURBS 需为实体）"
                    : "主体对象必须是封闭实体（Mesh 需封闭、NURBS 需为实体）");
                return;
            }

            if (skipped > 0)
                RhinoApp.WriteLine(string.Format("已跳过 {0} 个非封闭对象（布尔运算要求封闭实体）", skipped));

            // 混合类型：把 Brep 统一网格化，走网格布尔
            bool mixed = meshes.Count > 0 && breps.Count > 0;
            if (mixed)
            {
                var mp = new MeshingParameters(0.3);
                foreach (Brep b in breps)
                {
                    Mesh[] parts = Mesh.CreateFromBrep(b, mp);
                    if (parts != null)
                    {
                        foreach (Mesh part in parts)
                            if (part.IsValid)
                                meshes.Add(part);
                    }
                }
                breps.Clear();
                RhinoApp.WriteLine("检测到混合类型（Mesh + NURBS），已统一网格化后执行布尔");
            }

            if (meshes.Count < 2 && breps.Count < 2)
            {
                Warn(string.Format("可参与{0}运算的封闭实体不足 2 个（有效 {1} 个，跳过 {2} 个）",
                    opName, meshes.Count + breps.Count, skipped));
                return;
            }

            StatusText.Text = string.Format("正在计算{0}…", opName);
            UpdateLayout();

            double tol = _doc.ModelAbsoluteTolerance;
            Mesh[] resultMeshes = null;
            Brep[] resultBreps = null;

            if (meshes.Count >= 2)
                resultMeshes = RunMeshBoolean(meshes, kind);
            else
                resultBreps = RunBrepBoolean(breps, kind, tol);

            int resultCount = resultMeshes != null ? resultMeshes.Length
                            : resultBreps != null ? resultBreps.Length : 0;

            if (resultCount == 0)
            {
                Warn(string.Format("{0}运算未产生结果：输入的实体可能不互相重叠（交集要求全部重叠）", opName));
                _doc.Views.Redraw();
                return;
            }

            // 结果自动修复为封闭实体（布尔运算常留下微小缝隙、退化面或面法线不一致）
            int closedCount = 0;
            if (resultMeshes != null)
            {
                for (int i = 0; i < resultMeshes.Length; i++)
                {
                    resultMeshes[i] = RepairMeshToClosed(resultMeshes[i], tol);
                    if (resultMeshes[i] != null && resultMeshes[i].IsClosed)
                        closedCount++;
                }
            }
            else
            {
                foreach (Brep b in resultBreps)
                {
                    if (RepairBrepToSolid(b, tol))
                        closedCount++;
                }
            }

            // 结果继承主体对象的图层、材质与颜色
            ObjectAttributes resultAttr = mainObj.Attributes.Duplicate();
            resultAttr.Name = "Superquadric_Boolean_" + opName;

            // 结果落地：删除输入对象，加入运算结果（整体注册撤销）
            uint undoId = _doc.BeginUndoRecord("Superquadric " + opName);
            foreach (RhinoObject o in picked)
                _doc.Objects.Delete(o, true);

            int added = 0;
            if (resultMeshes != null)
            {
                foreach (Mesh m in resultMeshes)
                {
                    if (_doc.Objects.AddMesh(m, resultAttr) != Guid.Empty)
                        added++;
                }
            }
            else
            {
                foreach (Brep b in resultBreps)
                {
                    if (_doc.Objects.AddBrep(b, resultAttr) != Guid.Empty)
                        added++;
                }
            }
            _doc.EndUndoRecord(undoId);

            _doc.Views.Redraw();
            StatusText.Text = string.Format(
                "{0}完成：{1} 个输入实体 → {2} 个结果（{3}，已继承主体材质与颜色）",
                opName, picked.Count - skipped, added,
                resultMeshes != null ? "Mesh" : "NURBS 实体");

            if (closedCount < added)
                StatusText.Text += string.Format("；其中 {0}/{1} 个已修复为封闭实体，{2} 个仍非封闭",
                    closedCount, added, added - closedCount);

            RhinoApp.WriteLine(StatusText.Text);
        }

        /// <summary>
        /// 把网格修复为封闭模型，返回修复后的网格（可能是新对象）；无法修复时返回原网格。
        /// 流程：自适应容差焊点 → 剔除退化面 → 多轮填洞 → 统一法线；
        /// 若仍有开放边界，再走"网格 → Brep → 缝合裸边 → 转回网格"这条更强的兜底路径。
        /// </summary>
        private static Mesh RepairMeshToClosed(Mesh src, double tolerance)
        {
            if (src == null)
                return null;
            if (src.IsClosed)
                return src;

            Mesh mesh = (Mesh)src.Duplicate();

            // 焊点容差按模型尺度自适应：布尔缝隙通常远小于包围盒对角线
            BoundingBox bbox = mesh.GetBoundingBox(true);
            double diag = bbox.Diagonal.Length;
            double weldTol = Math.Max(tolerance, diag * 0.001);
            if (weldTol <= 0.0)
                weldTol = 1e-6;

            Mesh welded = WeldVertices(mesh, weldTol);
            if (welded != null)
                mesh = welded;

            mesh.Faces.CullDegenerateFaces();

            for (int pass = 0; pass < 4 && !mesh.IsClosed; pass++)
                mesh.FillHoles();

            mesh.UnifyNormals();
            mesh.Compact();
            if (mesh.IsClosed)
                return mesh;

            // 兜底：借 B-rep 拓扑缝合（Rhino 对裸边的配对比网格更宽容）
            Mesh viaBrep = RepairViaBrep(mesh, weldTol);
            if (viaBrep != null && viaBrep.IsClosed)
                return viaBrep;

            return mesh;
        }

        /// <summary>
        /// 按容差合并距离过近的顶点，并重建面拓扑（退化为零面积的面会被丢弃）。
        /// 这是解决"缝隙顶点没被 FillHoles 识别"的关键一步。无合并时返回 null。
        /// </summary>
        private static Mesh WeldVertices(Mesh source, double tolerance)
        {
            int n = source.Vertices.Count;
            if (n == 0)
                return null;

            double cell = Math.Max(tolerance, 1e-12);
            var buckets = new Dictionary<long, List<int>>();
            var rep = new int[n];
            var keep = new List<int>(n);

            for (int i = 0; i < n; i++)
            {
                Point3d p = source.Vertices[i];
                int match = -1;

                long cx = (long)Math.Floor(p.X / cell);
                long cy = (long)Math.Floor(p.Y / cell);
                long cz = (long)Math.Floor(p.Z / cell);

                for (long dx = -1; dx <= 1; dx++)
                {
                    for (long dy = -1; dy <= 1; dy++)
                    {
                        for (long dz = -1; dz <= 1; dz++)
                        {
                            List<int> list;
                            if (!buckets.TryGetValue(CellKey(cx + dx, cy + dy, cz + dz), out list))
                                continue;

                            foreach (int j in list)
                            {
                                if (p.DistanceTo(source.Vertices[j]) <= tolerance)
                                {
                                    match = j;
                                    break;
                                }
                            }
                            if (match >= 0)
                                break;
                        }
                        if (match >= 0)
                            break;
                    }
                    if (match >= 0)
                        break;
                }

                if (match >= 0)
                {
                    rep[i] = rep[match];
                }
                else
                {
                    rep[i] = i;
                    keep.Add(i);

                    long key = CellKey(cx, cy, cz);
                    List<int> list;
                    if (!buckets.TryGetValue(key, out list))
                    {
                        list = new List<int>(4);
                        buckets.Add(key, list);
                    }
                    list.Add(i);
                }
            }

            if (keep.Count == n)
                return null;   // 没有发生合并

            Mesh mesh = new Mesh();
            var newIndex = new int[n];
            for (int i = 0; i < n; i++)
                newIndex[i] = -1;

            foreach (int i in keep)
                newIndex[i] = mesh.Vertices.Add(source.Vertices[i]);
            for (int i = 0; i < n; i++)
            {
                if (newIndex[i] < 0)
                    newIndex[i] = newIndex[rep[i]];
            }

            for (int fi = 0; fi < source.Faces.Count; fi++)
            {
                MeshFace f = source.Faces[fi];
                int a = newIndex[f.A];
                int b = newIndex[f.B];
                int c = newIndex[f.C];

                if (f.IsTriangle)
                {
                    if (a == b || b == c || a == c)
                        continue;
                    mesh.Faces.AddFace(a, b, c);
                }
                else
                {
                    int d = newIndex[f.D];
                    if (a == b && c == d)
                        continue;
                    if (a == d && b == c)
                        continue;

                    if (a == b)      mesh.Faces.AddFace(a, c, d);
                    else if (b == c) mesh.Faces.AddFace(a, b, d);
                    else if (c == d) mesh.Faces.AddFace(a, b, c);
                    else if (d == a) mesh.Faces.AddFace(a, b, c);
                    else             mesh.Faces.AddFace(a, b, c, d);
                }
            }

            return mesh;
        }

        /// <summary>网格 → B-rep → 缝合裸边 → 转回网格；成功得到封闭网格时返回，否则 null。</summary>
        private static Mesh RepairViaBrep(Mesh mesh, double tolerance)
        {
            Brep brep = Brep.CreateFromMesh(mesh, false);
            if (brep == null)
                return null;

            for (int pass = 0; pass < 4 && !brep.IsSolid; pass++)
                brep.JoinNakedEdges(tolerance);

            if (!brep.IsSolid)
                return null;

            Mesh[] parts = Mesh.CreateFromBrep(brep, new MeshingParameters(0.4));
            if (parts == null || parts.Length == 0)
                return null;

            Mesh joined = new Mesh();
            foreach (Mesh part in parts)
                joined.Append(part);
            joined.Vertices.CombineIdentical(true, true);
            joined.Faces.CullDegenerateFaces();
            joined.Compact();

            return joined;
        }

        private static long CellKey(long x, long y, long z)
        {
            return x * 73856093L ^ y * 19349663L ^ z * 83492791L;
        }

        /// <summary>
        /// Brep 修复为封闭实体：拼接裸边（naked edges）成闭合壳。
        /// 返回是否最终为 solid。
        /// </summary>
        private static bool RepairBrepToSolid(Brep brep, double tolerance)
        {
            if (brep == null)
                return false;
            if (brep.IsSolid)
                return true;

            // 缝合未配对边；多轮以处理链式缝隙
            for (int pass = 0; pass < 4 && !brep.IsSolid; pass++)
                brep.JoinNakedEdges(tolerance);

            return brep.IsSolid;
        }

        /// <summary>
        /// 「修复封闭」按钮：对当前选中的网格/曲面执行与布尔结果相同的修复流程，
        /// 修复成功后用新对象替换原对象（可撤销）。
        /// </summary>
        private void OnRepairClosed(object sender, RoutedEventArgs e)
        {
            var targets = new List<RhinoObject>();
            foreach (RhinoObject o in _doc.Objects.GetSelectedObjects(false, false))
            {
                if (o.ObjectType == ObjectType.Mesh || o.ObjectType == ObjectType.Brep)
                    targets.Add(o);
            }

            if (targets.Count == 0)
            {
                Warn("请先选中需要修复的网格或曲面对象");
                return;
            }

            double tol = _doc.ModelAbsoluteTolerance;
            int repairedCount = 0;
            int failedCount = 0;

            StatusText.Text = "正在修复…";
            UpdateLayout();

            uint undoId = _doc.BeginUndoRecord("Superquadric 修复封闭");

            foreach (RhinoObject o in targets)
            {
                if (o.ObjectType == ObjectType.Mesh)
                {
                    Mesh src = o.Geometry as Mesh;
                    Mesh fixedMesh = RepairMeshToClosed(src, tol);
                    if (fixedMesh == null)
                        continue;

                    if (fixedMesh.IsClosed && src != null && !src.IsClosed)
                    {
                        ObjectAttributes attr = o.Attributes.Duplicate();
                        _doc.Objects.Delete(o, true);
                        _doc.Objects.AddMesh(fixedMesh, attr);
                        repairedCount++;
                    }
                    else if (fixedMesh.IsClosed)
                    {
                        repairedCount++;   // 本来就是封闭的
                    }
                    else
                    {
                        failedCount++;
                    }
                }
                else
                {
                    Brep src = o.Geometry as Brep;
                    if (RepairBrepToSolid(src, tol))
                    {
                        _doc.Objects.Replace(o.Id, src);
                        repairedCount++;
                    }
                    else
                    {
                        failedCount++;
                    }
                }
            }

            _doc.EndUndoRecord(undoId);
            _doc.Views.Redraw();

            StatusText.Text = string.Format(
                "修复完成：{0} 个已为封闭实体，{1} 个仍无法修复（可尝试加大文档容差或手动封盖）",
                repairedCount, failedCount);
            RhinoApp.WriteLine(StatusText.Text);
        }

        private void Warn(string message)
        {
            StatusText.Text = message;
            RhinoApp.WriteLine(message);
        }

        /// <summary>逐个迭代的网格布尔：Union/Intersection/Difference 都用同一个累积数组。</summary>
        private static Mesh[] RunMeshBoolean(List<Mesh> input, BooleanOpKind kind)
        {
            Mesh[] acc = new Mesh[] { input[0] };

            for (int i = 1; i < input.Count; i++)
            {
                var next = new List<Mesh>();
                foreach (Mesh a in acc)
                {
                    Mesh[] r;
                    switch (kind)
                    {
                        case BooleanOpKind.Union:
                            r = Mesh.CreateBooleanUnion(new Mesh[] { a, input[i] });
                            break;
                        case BooleanOpKind.Intersection:
                            r = Mesh.CreateBooleanIntersection(new Mesh[] { a }, new Mesh[] { input[i] });
                            break;
                        default:
                            r = Mesh.CreateBooleanDifference(new Mesh[] { a }, new Mesh[] { input[i] });
                            break;
                    }
                    if (r != null)
                    {
                        foreach (Mesh m in r)
                            next.Add(m);
                    }
                }
                acc = next.ToArray();
                if (acc.Length == 0)
                    return null;
            }
            return acc;
        }

        /// <summary>逐个迭代的 B-rep（NURBS 实体）布尔。</summary>
        private static Brep[] RunBrepBoolean(List<Brep> input, BooleanOpKind kind, double tolerance)
        {
            Brep[] acc = new Brep[] { input[0] };

            for (int i = 1; i < input.Count; i++)
            {
                var next = new List<Brep>();
                foreach (Brep a in acc)
                {
                    Brep[] r;
                    switch (kind)
                    {
                        case BooleanOpKind.Union:
                            r = Brep.CreateBooleanUnion(new Brep[] { a, input[i] }, tolerance);
                            break;
                        case BooleanOpKind.Intersection:
                            r = Brep.CreateBooleanIntersection(a, input[i], tolerance);
                            break;
                        default:
                            r = Brep.CreateBooleanDifference(a, input[i], tolerance);
                            break;
                    }
                    if (r != null)
                    {
                        foreach (Brep b in r)
                            next.Add(b);
                    }
                }
                acc = next.ToArray();
                if (acc.Length == 0)
                    return null;
            }
            return acc;
        }

        private void OnWindowClosed(object sender, EventArgs e)
        {
            _rebuildTimer.Stop();
            _conduit.SetMesh(null);
            _conduit.Enabled = false;
            _doc.Views.Redraw();
        }
    }

    /// <summary>
    /// 超二次曲面实时预览 conduit：在 PostDrawObjects 阶段绘制半透明实体 + 网格线，
    /// 窗口关闭后由 SuperquadricWindow 关闭并清空预览。
    /// </summary>
    internal class SuperquadricPreviewConduit : Rhino.Display.DisplayConduit
    {
        private Mesh _mesh;
        private Rhino.Display.DisplayMaterial _material;

        public void SetMesh(Mesh mesh)
        {
            if (mesh == null)
            {
                _mesh = null;
                _material = null;
                Enabled = false;
                return;
            }

            _mesh = mesh;
            // 半透明预览色：浅青蓝，便于与已有模型区分
            _material = new Rhino.Display.DisplayMaterial(
                System.Drawing.Color.FromArgb(110, 120, 210),
                0.45);
            _material.Emission = System.Drawing.Color.FromArgb(90, 100, 190);
            Enabled = true;
        }

        protected override void PostDrawObjects(Rhino.Display.DrawEventArgs e)
        {
            if (_mesh == null || _material == null)
                return;

            e.Display.DrawMeshShaded(_mesh, _material);
            e.Display.DrawMeshWires(_mesh, System.Drawing.Color.FromArgb(160, 40, 90, 160));
        }
    }
}
