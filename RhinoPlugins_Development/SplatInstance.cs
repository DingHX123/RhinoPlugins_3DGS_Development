using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading.Tasks;

namespace RhinoPlugins_Development
{
    /// <summary>一份 splat 数据集的 GPU 资源与状态（纹理缓冲 + 代理点云变换跟踪）。</summary>
    public class SplatInstance
    {
        public uint ColorBuffer;
        public uint ColorTex;
        public uint RotBuffer;
        public uint RotTex;
        public uint ScaleBuffer;
        public uint ScaleTex;
        public uint PosBuffer;
        public uint PosTex;
        public bool GlInitialized;
        public readonly Dictionary<Guid, ViewportState> ViewportStates = new Dictionary<Guid, ViewportState>();

        public SplatInstance(Splat[] splats, Guid proxyObjectId)
        {
            ProxyObjectId = proxyObjectId;
            SetSplats(splats);
        }

        public Guid ProxyObjectId { get; set; }
        public Splat[] Splats { get; private set; }
        public Splat[] OriginalSplats { get; private set; }
        public BoundingBox BBox { get; private set; }
        public bool IsLoaded { get; private set; }
        public bool GpuDataDirty { get; set; }
        public bool PrintedCamSpace { get; set; }
        public Point3d OrigP0 { get; set; }
        public Point3d OrigP1 { get; set; }
        public Point3d OrigP2 { get; set; }
        public Point3d LastProxyP0 { get; set; }
        public Point3d LastProxyP1 { get; set; }
        public Point3d LastProxyP2 { get; set; }

        /// <summary>将 splat 数据上传到 GPU 纹理缓冲（位置/缩放/旋转/颜色各一个 TBO）。</summary>
        public bool InitGL()
        {
            try
            {
                int count = Splats.Length;
                float[] positions = new float[count * 3];
                float[] scales = new float[count * 3];
                float[] rotations = new float[count * 4];
                byte[] colors = new byte[count * 4];

                Parallel.For(0, count, i =>
                {
                    Splat splat = Splats[i];
                    positions[i * 3] = splat.X;
                    positions[i * 3 + 1] = splat.Y;
                    positions[i * 3 + 2] = splat.Z;
                    scales[i * 3] = splat.ScaleX;
                    scales[i * 3 + 1] = splat.ScaleY;
                    scales[i * 3 + 2] = splat.ScaleZ;
                    rotations[i * 4] = splat.RotW;
                    rotations[i * 4 + 1] = splat.RotX;
                    rotations[i * 4 + 2] = splat.RotY;
                    rotations[i * 4 + 3] = splat.RotZ;
                    colors[i * 4] = splat.R;
                    colors[i * 4 + 1] = splat.G;
                    colors[i * 4 + 2] = splat.B;
                    colors[i * 4 + 3] = splat.A;
                });

                OpenGL.glGenBuffers(1, out PosBuffer);
                OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, PosBuffer);
                GCHandle posHandle = GCHandle.Alloc(positions, GCHandleType.Pinned);
                OpenGL.glBufferData(OpenGL.GL_ARRAY_BUFFER, (IntPtr)(positions.Length * 4), posHandle.AddrOfPinnedObject(), OpenGL.GL_STATIC_DRAW);
                posHandle.Free();
                OpenGL.glGenTextures(1, out PosTex);
                OpenGL.glBindTexture(OpenGL.GL_TEXTURE_BUFFER, PosTex);
                OpenGL.glTexBuffer(OpenGL.GL_TEXTURE_BUFFER, OpenGL.GL_RGB32F, PosBuffer);

                OpenGL.glGenBuffers(1, out ScaleBuffer);
                OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, ScaleBuffer);
                GCHandle scaleHandle = GCHandle.Alloc(scales, GCHandleType.Pinned);
                OpenGL.glBufferData(OpenGL.GL_ARRAY_BUFFER, (IntPtr)(scales.Length * 4), scaleHandle.AddrOfPinnedObject(), OpenGL.GL_STATIC_DRAW);
                scaleHandle.Free();
                OpenGL.glGenTextures(1, out ScaleTex);
                OpenGL.glBindTexture(OpenGL.GL_TEXTURE_BUFFER, ScaleTex);
                OpenGL.glTexBuffer(OpenGL.GL_TEXTURE_BUFFER, OpenGL.GL_RGB32F, ScaleBuffer);

                OpenGL.glGenBuffers(1, out RotBuffer);
                OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, RotBuffer);
                GCHandle rotHandle = GCHandle.Alloc(rotations, GCHandleType.Pinned);
                OpenGL.glBufferData(OpenGL.GL_ARRAY_BUFFER, (IntPtr)(rotations.Length * 4), rotHandle.AddrOfPinnedObject(), OpenGL.GL_STATIC_DRAW);
                rotHandle.Free();
                OpenGL.glGenTextures(1, out RotTex);
                OpenGL.glBindTexture(OpenGL.GL_TEXTURE_BUFFER, RotTex);
                OpenGL.glTexBuffer(OpenGL.GL_TEXTURE_BUFFER, OpenGL.GL_RGBA32F, RotBuffer);

                OpenGL.glGenBuffers(1, out ColorBuffer);
                OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, ColorBuffer);
                GCHandle colorHandle = GCHandle.Alloc(colors, GCHandleType.Pinned);
                OpenGL.glBufferData(OpenGL.GL_ARRAY_BUFFER, (IntPtr)colors.Length, colorHandle.AddrOfPinnedObject(), OpenGL.GL_STATIC_DRAW);
                colorHandle.Free();
                OpenGL.glGenTextures(1, out ColorTex);
                OpenGL.glBindTexture(OpenGL.GL_TEXTURE_BUFFER, ColorTex);
                OpenGL.glTexBuffer(OpenGL.GL_TEXTURE_BUFFER, OpenGL.GL_RGBA8, ColorBuffer);

                OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, 0);
                GlInitialized = true;
                return true;
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine("Failed to initialize OpenGL for instance: " + ex.Message);
                return false;
            }
        }

        /// <summary>释放本实例的全部 GPU 资源。</summary>
        public void Cleanup()
        {
            if (!GlInitialized)
                return;

            try
            {
                if (PosBuffer != 0) OpenGL.glDeleteBuffers(1, ref PosBuffer);
                if (ScaleBuffer != 0) OpenGL.glDeleteBuffers(1, ref ScaleBuffer);
                if (RotBuffer != 0) OpenGL.glDeleteBuffers(1, ref RotBuffer);
                if (ColorBuffer != 0) OpenGL.glDeleteBuffers(1, ref ColorBuffer);
                if (PosTex != 0) OpenGL.glDeleteTextures(1, ref PosTex);
                if (ScaleTex != 0) OpenGL.glDeleteTextures(1, ref ScaleTex);
                if (RotTex != 0) OpenGL.glDeleteTextures(1, ref RotTex);
                if (ColorTex != 0) OpenGL.glDeleteTextures(1, ref ColorTex);
            }
            catch
            {
            }

            PosBuffer = 0;
            ScaleBuffer = 0;
            RotBuffer = 0;
            ColorBuffer = 0;
            PosTex = 0;
            ScaleTex = 0;
            RotTex = 0;
            ColorTex = 0;
            GlInitialized = false;
            ClearViewportStates();
        }

        public void ClearViewportStates()
        {
            lock (ViewportStates)
            {
                foreach (ViewportState state in ViewportStates.Values)
                {
                    if (state.IndexVbo != 0)
                    {
                        uint indexVbo = state.IndexVbo;
                        try
                        {
                            OpenGL.glDeleteBuffers(1, ref indexVbo);
                        }
                        catch
                        {
                        }
                        state.IndexVbo = 0;
                    }
                }
                ViewportStates.Clear();
            }
        }

        /// <summary>替换 splat 数据（清空 GPU 资源、重算包围盒）。</summary>
        public void SetSplats(Splat[] splats)
        {
            if (GlInitialized)
                Cleanup();

            Splats = splats;
            OriginalSplats = (Splat[])splats.Clone();

            ComputeBoundingBox(splats);
            IsLoaded = true;
            GlInitialized = false;
            ClearViewportStates();
            OrigP0 = Point3d.Unset;
            OrigP1 = Point3d.Unset;
            OrigP2 = Point3d.Unset;
            LastProxyP0 = Point3d.Unset;
            LastProxyP1 = Point3d.Unset;
            LastProxyP2 = Point3d.Unset;
            GpuDataDirty = false;
        }

        /// <summary>仅更新 GPU 上的位置/缩放/旋转数据（颜色不变）。</summary>
        public void UpdateGPUData()
        {
            if (!GlInitialized)
                return;

            int count = Splats.Length;
            float[] positions = new float[count * 3];
            float[] scales = new float[count * 3];
            float[] rotations = new float[count * 4];

            Parallel.For(0, count, i =>
            {
                Splat splat = Splats[i];
                positions[i * 3] = splat.X;
                positions[i * 3 + 1] = splat.Y;
                positions[i * 3 + 2] = splat.Z;
                scales[i * 3] = splat.ScaleX;
                scales[i * 3 + 1] = splat.ScaleY;
                scales[i * 3 + 2] = splat.ScaleZ;
                rotations[i * 4] = splat.RotW;
                rotations[i * 4 + 1] = splat.RotX;
                rotations[i * 4 + 2] = splat.RotY;
                rotations[i * 4 + 3] = splat.RotZ;
            });

            OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, PosBuffer);
            GCHandle posHandle = GCHandle.Alloc(positions, GCHandleType.Pinned);
            OpenGL.glBufferSubData(OpenGL.GL_ARRAY_BUFFER, IntPtr.Zero, (IntPtr)(positions.Length * 4), posHandle.AddrOfPinnedObject());
            posHandle.Free();

            OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, ScaleBuffer);
            GCHandle scaleHandle = GCHandle.Alloc(scales, GCHandleType.Pinned);
            OpenGL.glBufferSubData(OpenGL.GL_ARRAY_BUFFER, IntPtr.Zero, (IntPtr)(scales.Length * 4), scaleHandle.AddrOfPinnedObject());
            scaleHandle.Free();

            OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, RotBuffer);
            GCHandle rotHandle = GCHandle.Alloc(rotations, GCHandleType.Pinned);
            OpenGL.glBufferSubData(OpenGL.GL_ARRAY_BUFFER, IntPtr.Zero, (IntPtr)(rotations.Length * 4), rotHandle.AddrOfPinnedObject());
            rotHandle.Free();

            OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, 0);
        }

        /// <summary>根据代理点云的刚体变换（含缩放）更新 splat 数据。</summary>
        public void UpdateTransformedSplats(Transform T, double scale)
        {
            if (OriginalSplats == null || OriginalSplats.Length == 0)
                return;

            // 提取旋转矩阵（去掉平移分量，并把缩放从线性部分中除掉）
            Transform rotOnly = T;
            rotOnly.M03 = 0.0;
            rotOnly.M13 = 0.0;
            rotOnly.M23 = 0.0;
            rotOnly.M30 = 0.0;
            rotOnly.M31 = 0.0;
            rotOnly.M32 = 0.0;
            rotOnly.M33 = 1.0;

            if (scale != 1.0 && scale > 1e-6)
            {
                for (int row = 0; row < 3; row++)
                    for (int col = 0; col < 3; col++)
                        rotOnly[row, col] = rotOnly[row, col] / scale;
            }

            // 旋转矩阵 -> 四元数
            double qw, qx, qy, qz;
            double trace = rotOnly.M00 + rotOnly.M11 + rotOnly.M22;
            if (trace > 0.0)
            {
                double s = Math.Sqrt(trace + 1.0) * 2.0;
                qw = 0.25 * s;
                qx = (rotOnly.M21 - rotOnly.M12) / s;
                qy = (rotOnly.M02 - rotOnly.M20) / s;
                qz = (rotOnly.M10 - rotOnly.M01) / s;
            }
            else if (rotOnly.M00 > rotOnly.M11 && rotOnly.M00 > rotOnly.M22)
            {
                double s = Math.Sqrt(1.0 + rotOnly.M00 - rotOnly.M11 - rotOnly.M22) * 2.0;
                qw = (rotOnly.M21 - rotOnly.M12) / s;
                qx = 0.25 * s;
                qy = (rotOnly.M01 + rotOnly.M10) / s;
                qz = (rotOnly.M02 + rotOnly.M20) / s;
            }
            else if (rotOnly.M11 > rotOnly.M22)
            {
                double s = Math.Sqrt(1.0 + rotOnly.M11 - rotOnly.M00 - rotOnly.M22) * 2.0;
                qw = (rotOnly.M02 - rotOnly.M20) / s;
                qx = (rotOnly.M01 + rotOnly.M10) / s;
                qy = 0.25 * s;
                qz = (rotOnly.M12 + rotOnly.M21) / s;
            }
            else
            {
                double s = Math.Sqrt(1.0 + rotOnly.M22 - rotOnly.M00 - rotOnly.M11) * 2.0;
                qw = (rotOnly.M10 - rotOnly.M01) / s;
                qx = (rotOnly.M02 + rotOnly.M20) / s;
                qy = (rotOnly.M12 + rotOnly.M21) / s;
                qz = 0.25 * s;
            }

            Parallel.For(0, OriginalSplats.Length, i =>
            {
                Splat splat = OriginalSplats[i];
                Point3d pt = new Point3d(splat.X, splat.Y, splat.Z);
                pt.Transform(T);

                // 四元数乘法：旋转应用于 splat 的朝向四元数
                double w = qw, x = qx, y = qy, z = qz;
                double rw = splat.RotW, rx = splat.RotX, ry = splat.RotY, rz = splat.RotZ;

                Splats[i] = new Splat
                {
                    X = (float)pt.X,
                    Y = (float)pt.Y,
                    Z = (float)pt.Z,
                    ScaleX = splat.ScaleX * (float)scale,
                    ScaleY = splat.ScaleY * (float)scale,
                    ScaleZ = splat.ScaleZ * (float)scale,
                    RotW = (float)(w * rw - x * rx - y * ry - z * rz),
                    RotX = (float)(w * rx + x * rw + y * rz - z * ry),
                    RotY = (float)(w * ry - x * rz + y * rw + z * rx),
                    RotZ = (float)(w * rz + x * ry - y * rx + z * rw),
                    R = splat.R,
                    G = splat.G,
                    B = splat.B,
                    A = splat.A
                };
            });

            ComputeBoundingBox(Splats);
            GpuDataDirty = true;
            ClearViewportStates();
        }

        private void ComputeBoundingBox(Splat[] splats)
        {
            double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue;
            double maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;
            for (int i = 0; i < splats.Length; i++)
            {
                Splat splat = splats[i];
                if (splat.X < minX) minX = splat.X;
                if (splat.Y < minY) minY = splat.Y;
                if (splat.Z < minZ) minZ = splat.Z;
                if (splat.X > maxX) maxX = splat.X;
                if (splat.Y > maxY) maxY = splat.Y;
                if (splat.Z > maxZ) maxZ = splat.Z;
            }
            BBox = new BoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
        }
    }
}
