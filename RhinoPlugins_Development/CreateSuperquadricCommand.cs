using System;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace RhinoPlugins_Development
{
    /// <summary>
    /// CreateSuperquadric：打开参数窗口设置超二次曲面（Barr 参数化）的半轴 a/b/c、
    /// 形状指数 ε1/ε2、网格分辨率、中心位置与旋转角度，点击"创建"后
    /// 按参数化方程生成封闭网格并加入当前文档。
    /// ε1=ε2=1 为椭球；ε→0 趋近长方体；ε=2 趋近八面体。
    /// </summary>
    public class CreateSuperquadricCommand : Command
    {
        public override string EnglishName => "CreateSuperquadric";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (mode != RunMode.Interactive)
            {
                // 脚本模式：无法开窗交互，直接用默认参数（单位椭球）创建网格
                return CreateFromParams(doc, SuperquadricParams.Default());
            }

            // 交互模式：非模态窗口 + 实时预览 conduit
            // 修改参数时窗口重建预览网格并刷新视口；点「创建」才把网格加入文档。
            var conduit = new SuperquadricPreviewConduit();
            var window = new SuperquadricWindow(doc, conduit);
            var helper = new System.Windows.Interop.WindowInteropHelper(window);
            helper.Owner = RhinoApp.MainWindowHandle();

            RhinoApp.WriteLine("已打开超二次曲面参数窗口：修改参数可在视口实时预览，点「创建」确认生成。");
            window.Show();
            return Result.Success;
        }

        /// <summary>按给定参数生成超二次曲面网格并加入文档（供脚本模式与窗口创建共用）。</summary>
        private static Result CreateFromParams(RhinoDoc doc, SuperquadricParams parms)
        {
            Mesh mesh = SuperquadricMeshBuilder.Build(parms);
            if (mesh == null || !mesh.IsValid)
            {
                RhinoApp.WriteLine("超二次曲面网格生成失败，请检查参数");
                return Result.Failure;
            }

            var attributes = new ObjectAttributes();
            attributes.Name = string.Format(
                "Superquadric_a{0:F2}_b{1:F2}_c{2:F2}_e1{3:F2}_e2{4:F2}",
                parms.AxisA, parms.AxisB, parms.AxisC, parms.Epsilon1, parms.Epsilon2);

            Guid objectId = doc.Objects.AddMesh(mesh, attributes);
            if (objectId == Guid.Empty)
            {
                RhinoApp.WriteLine("网格加入文档失败");
                return Result.Failure;
            }

            doc.Views.Redraw();
            RhinoApp.WriteLine(string.Format(
                "已创建超二次曲面：a={0:F3}, b={1:F3}, c={2:F3}, ε1={3:F2}, ε2={4:F2}，" +
                "网格 {5} 顶点 / {6} 面（命令名 CreateSuperquadric）",
                parms.AxisA, parms.AxisB, parms.AxisC, parms.Epsilon1, parms.Epsilon2,
                mesh.Vertices.Count, mesh.Faces.Count));

            return Result.Success;
        }
    }

    /// <summary>
    /// 超二次曲面参数化网格生成器（Barr 1981）：
    ///   x = a·sgn(cosη)|cosη|^ε1 · sgn(cosω)|cosω|^ε2
    ///   y = b·sgn(cosη)|cosη|^ε1 · sgn(sinω)|sinω|^ε2
    ///   z = c·sgn(sinη)|sinη|^ε1
    /// 其中 η ∈ [-π/2, π/2]（纬度），ω ∈ [-π, π)（经度）。
    /// 顶点布局：南极点 + (V-1) 个纬线环×U 个顶点 + 北极点，拓扑封闭。
    /// </summary>
    internal static class SuperquadricMeshBuilder
    {
        public static Mesh Build(SuperquadricParams p)
        {
            int nu = p.SegmentsU;
            int nv = p.SegmentsV;
            double a = p.AxisA;
            double b = p.AxisB;
            double c = p.AxisC;
            double e1 = p.Epsilon1;
            double e2 = p.Epsilon2;

            var mesh = new Mesh();

            // ===== 顶点：南极 + (nv-1) 个纬线环 + 北极 =====
            mesh.Vertices.Add(new Point3d(0.0, 0.0, -c)); // 南极，索引 0

            for (int j = 0; j < nv - 1; j++)
            {
                double eta = -Math.PI / 2.0 + Math.PI * (j + 1) / nv;
                double cosEta = SignedPow(Math.Cos(eta), e1);
                double sinEta = SignedPow(Math.Sin(eta), e1);
                for (int i = 0; i < nu; i++)
                {
                    double omega = -Math.PI + 2.0 * Math.PI * i / nu;
                    double cosOmega = SignedPow(Math.Cos(omega), e2);
                    double sinOmega = SignedPow(Math.Sin(omega), e2);
                    mesh.Vertices.Add(new Point3d(
                        a * cosEta * cosOmega,
                        b * cosEta * sinOmega,
                        c * sinEta));
                }
            }

            int northPoleIndex = mesh.Vertices.Count;
            mesh.Vertices.Add(new Point3d(0.0, 0.0, c)); // 北极

            // ===== 面：按右手法则保证法线朝外 =====
            // 底部三角扇：(环0[i], 南极, 环0[i+1])
            for (int i = 0; i < nu; i++)
            {
                int iNext = (i + 1) % nu;
                mesh.Faces.AddFace(1 + i, 0, 1 + iNext);
            }

            // 中间四边形：环 j → 环 j+1
            for (int j = 0; j < nv - 2; j++)
            {
                int rowA = 1 + j * nu;
                int rowB = rowA + nu;
                for (int i = 0; i < nu; i++)
                {
                    int iNext = (i + 1) % nu;
                    mesh.Faces.AddFace(rowA + i, rowA + iNext, rowB + iNext, rowB + i);
                }
            }

            // 顶部三角扇：(最后环[i], 最后环[i+1], 北极)
            int topRow = 1 + (nv - 2) * nu;
            for (int i = 0; i < nu; i++)
            {
                int iNext = (i + 1) % nu;
                mesh.Faces.AddFace(topRow + i, topRow + iNext, northPoleIndex);
            }

            mesh.Normals.ComputeNormals();

            // ===== 朝向保险：北极点法线应指向 +Z，否则反转所有面的绕序 =====
            if (northPoleIndex < mesh.Normals.Count && mesh.Normals[northPoleIndex].Z < 0.0)
            {
                for (int fi = 0; fi < mesh.Faces.Count; fi++)
                {
                    var face = mesh.Faces[fi];
                    if (face.IsQuad)
                        mesh.Faces[fi] = new MeshFace(face.A, face.D, face.C, face.B);
                    else
                        mesh.Faces[fi] = new MeshFace(face.A, face.C, face.B);
                }
                mesh.Normals.ComputeNormals();
            }

            // ===== 旋转（世界系 X→Y→Z 依次施加）并平移到中心 =====
            const double degToRad = Math.PI / 180.0;
            if (Math.Abs(p.RotationXDeg) > 1e-12)
                mesh.Transform(Transform.Rotation(p.RotationXDeg * degToRad, Vector3d.XAxis, Point3d.Origin));
            if (Math.Abs(p.RotationYDeg) > 1e-12)
                mesh.Transform(Transform.Rotation(p.RotationYDeg * degToRad, Vector3d.YAxis, Point3d.Origin));
            if (Math.Abs(p.RotationZDeg) > 1e-12)
                mesh.Transform(Transform.Rotation(p.RotationZDeg * degToRad, Vector3d.ZAxis, Point3d.Origin));

            var center = new Point3d(p.CenterX, p.CenterY, p.CenterZ);
            if (center.DistanceTo(Point3d.Origin) > 1e-12)
                mesh.Transform(Transform.Translation(center - Point3d.Origin));

            mesh.Compact();
            return mesh;
        }

        /// <summary>带符号幂：sgn(v)·|v|^e（超二次曲面参数化的核心运算）。</summary>
        internal static double SignedPow(double v, double e)
        {
            return Math.Sign(v) * Math.Pow(Math.Abs(v), e);
        }
    }

    /// <summary>
    /// 超二次曲面的 NURBS 表示：把参数化采样点当作控制点网格，构造 3 阶（degree 3）非有理
    /// clamped NURBS 曲面。控制点按球面拓扑排布——纬向为"南极行 + 中间行 + 北极行"，
    /// 极点整行重合（与 Rhino 球面 NURBS 的做法一致）；若重合极点导致退化，
    /// 自动退化为极小极环以保证曲面有效。经向首末控制点重合，接缝处几何闭合。
    /// </summary>
    internal static class SuperquadricNurbsBuilder
    {
        /// <summary>NURBS 控制点网格上限，兼顾形状精度与构建/显示开销。</summary>
        private const int MaxControlU = 96;
        private const int MaxControlV = 48;

        public static NurbsSurface Build(SuperquadricParams p)
        {
            int nu = Math.Max(8, Math.Min(p.SegmentsU, MaxControlU));
            int nv = Math.Max(4, Math.Min(p.SegmentsV, MaxControlV));

            // 先尝试严格重合的极点（标准球面 NURBS 拓扑）
            NurbsSurface srf = BuildInternal(p, nu, nv, 0.0);
            if (srf != null && srf.IsValid)
                return ApplyTransform(srf, p);

            // 重合极点被判定为退化时，改用极小极环（几何上几乎完全闭合）
            srf = BuildInternal(p, nu, nv, 1e-5);
            if (srf == null || !srf.IsValid)
                return null;
            return ApplyTransform(srf, p);
        }

        private static NurbsSurface BuildInternal(SuperquadricParams p, int nu, int nv, double poleRelativeScale)
        {
            int rows = nv + 1;                     // 南极行 + (nv-1) 中间行 + 北极行
            NurbsSurface srf = NurbsSurface.Create(3, false, 4, 4, nu, rows);
            if (srf == null)
                return null;

            double a = p.AxisA, b = p.AxisB, c = p.AxisC;
            double e1 = p.Epsilon1, e2 = p.Epsilon2;
            double poleR = poleRelativeScale > 0.0 ? poleRelativeScale * Math.Max(a, b) : 0.0;

            // 经向：首末控制点角度重合（u=0 与 u=nu-1 位于同一位置），使接缝几何闭合
            double dOmega = 2.0 * Math.PI / (nu - 1);

            // ---- v=0：南极行 ----
            for (int u = 0; u < nu; u++)
            {
                Point3d pt = poleR > 0.0
                    ? new Point3d(poleR * Math.Cos(dOmega * u), poleR * Math.Sin(dOmega * u), -c)
                    : new Point3d(0.0, 0.0, -c);
                srf.Points.SetPoint(u, 0, pt);
            }

            // ---- 中间纬线行 ----
            for (int j = 0; j < nv - 1; j++)
            {
                double eta = -Math.PI / 2.0 + Math.PI * (j + 1) / nv;
                double cosEta = SuperquadricMeshBuilder.SignedPow(Math.Cos(eta), e1);
                double sinEta = SuperquadricMeshBuilder.SignedPow(Math.Sin(eta), e1);

                for (int u = 0; u < nu; u++)
                {
                    double omega = dOmega * u;
                    srf.Points.SetPoint(u, j + 1, new Point3d(
                        a * cosEta * SuperquadricMeshBuilder.SignedPow(Math.Cos(omega), e2),
                        b * cosEta * SuperquadricMeshBuilder.SignedPow(Math.Sin(omega), e2),
                        c * sinEta));
                }
            }

            // ---- v=rows-1：北极行 ----
            int northRow = rows - 1;
            for (int u = 0; u < nu; u++)
            {
                Point3d pt = poleR > 0.0
                    ? new Point3d(poleR * Math.Cos(dOmega * u), poleR * Math.Sin(dOmega * u), c)
                    : new Point3d(0.0, 0.0, c);
                srf.Points.SetPoint(u, northRow, pt);
            }

            return srf;
        }

        /// <summary>按与 Mesh 版本相同的顺序施加旋转（X→Y→Z）与中心平移。</summary>
        private static NurbsSurface ApplyTransform(NurbsSurface srf, SuperquadricParams p)
        {
            const double degToRad = Math.PI / 180.0;

            if (Math.Abs(p.RotationXDeg) > 1e-12)
                srf.Transform(Transform.Rotation(p.RotationXDeg * degToRad, Vector3d.XAxis, Point3d.Origin));
            if (Math.Abs(p.RotationYDeg) > 1e-12)
                srf.Transform(Transform.Rotation(p.RotationYDeg * degToRad, Vector3d.YAxis, Point3d.Origin));
            if (Math.Abs(p.RotationZDeg) > 1e-12)
                srf.Transform(Transform.Rotation(p.RotationZDeg * degToRad, Vector3d.ZAxis, Point3d.Origin));

            var center = new Point3d(p.CenterX, p.CenterY, p.CenterZ);
            if (center.DistanceTo(Point3d.Origin) > 1e-12)
                srf.Transform(Transform.Translation(center - Point3d.Origin));

            return srf;
        }
    }
}
