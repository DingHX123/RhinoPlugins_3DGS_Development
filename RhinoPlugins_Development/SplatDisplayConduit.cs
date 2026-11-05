using Rhino;
using Rhino.Display;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace RhinoPlugins_Development
{
    /// <summary>
    /// 高斯泼溅显示管道：在 Rhino 视口中以 OpenGL 实例化渲染真实 splat 椭球，
    /// 并通过代理点云对象跟踪用户对模型的移动/旋转/缩放。
    /// </summary>
    public class SplatDisplayConduit : DisplayConduit
    {
        // float <-> uint 位级转换（net48 无 BitConverter.SingleToUInt32Bits）
        [StructLayout(LayoutKind.Explicit)]
        private struct FloatUIntUnion
        {
            [FieldOffset(0)] public float F;
            [FieldOffset(0)] public uint U;
        }

        private readonly List<SplatInstance> _instances = new List<SplatInstance>();
        private bool _globalGlInitialized;
        private uint _program;
        private uint _quadVbo;
        private int _viewLoc = -1;
        private int _projLoc = -1;
        private int _viewportSizeLoc = -1;
        private int _clipPlanesLoc = -1;
        private int _clipPlaneCountLoc = -1;
        private int _posSamplerLoc = -1;
        private int _scaleSamplerLoc = -1;
        private int _rotSamplerLoc = -1;
        private int _colorSamplerLoc = -1;
        private int _splatScaleLoc = -1;
        private int _filterRadiusLoc = -1;
        private int _isOrthoLoc = -1;
        private static Guid _pendingProxyId = Guid.Empty;

        /// <summary>splat 全局缩放系数（设置命令可调）。</summary>
        public static float SplatScale { get; set; } = 1.0f;

        /// <summary>2D 高斯滤波半径（像素²，越大越柔和）。</summary>
        public static float FilterRadius { get; set; } = 0.5f;

        public static SplatDisplayConduit Instance { get; private set; }

        public List<SplatInstance> Instances => _instances;

        public static Guid ProxyObjectId
        {
            get
            {
                SplatInstance inst = Instance?.GetActiveOrFirstInstance();
                if (inst == null)
                    return _pendingProxyId;
                return inst.ProxyObjectId;
            }
            set
            {
                _pendingProxyId = value;
            }
        }

        public BoundingBox SplatBoundingBox
        {
            get
            {
                if (_instances.Count == 0)
                    return BoundingBox.Empty;

                BoundingBox box = BoundingBox.Empty;
                RhinoDoc doc = RhinoDoc.ActiveDoc;
                foreach (SplatInstance instance in _instances)
                {
                    if (!instance.BBox.IsValid)
                        continue;
                    if (instance.ProxyObjectId != Guid.Empty && doc != null)
                    {
                        RhinoObject obj = doc.Objects.FindId(instance.ProxyObjectId);
                        if (obj == null || obj.IsDeleted)
                            continue;
                    }
                    box.Union(instance.BBox);
                }
                return box;
            }
        }

        // ========== 启用 / 禁用 ==========

        public static void EnableConduit(Splat[] splats)
        {
            EnableConduit(splats, Guid.Empty, Point3d.Unset, Point3d.Unset, Point3d.Unset);
        }

        public static void EnableConduit(Splat[] splats, Guid proxyId)
        {
            EnableConduit(splats, proxyId, Point3d.Unset, Point3d.Unset, Point3d.Unset);
        }

        public static void EnableConduit(Splat[] splats, Guid proxyId, Point3d origP0, Point3d origP1, Point3d origP2)
        {
            if (Instance == null)
                Instance = new SplatDisplayConduit();

            Guid proxyObjectId = proxyId == Guid.Empty ? _pendingProxyId : proxyId;
            _pendingProxyId = Guid.Empty;

            SplatInstance instance = new SplatInstance(splats, proxyObjectId);
            if (origP0 != Point3d.Unset && origP1 != Point3d.Unset && origP2 != Point3d.Unset)
            {
                instance.OrigP0 = origP0;
                instance.OrigP1 = origP1;
                instance.OrigP2 = origP2;
            }

            Instance._instances.Add(instance);
            Instance.Enabled = true;

            RhinoDoc doc = RhinoDoc.ActiveDoc;
            doc?.Views.Redraw();
        }

        public static void DisableConduit()
        {
            if (Instance != null)
            {
                Instance.Enabled = false;
                RhinoDoc doc = RhinoDoc.ActiveDoc;
                if (doc != null)
                {
                    foreach (SplatInstance instance in Instance._instances)
                    {
                        if (instance.ProxyObjectId != Guid.Empty)
                        {
                            try
                            {
                                doc.Objects.Delete(instance.ProxyObjectId, true);
                            }
                            catch
                            {
                            }
                        }
                    }
                }
                Instance.CleanupAll();
            }

            RhinoDoc activeDoc = RhinoDoc.ActiveDoc;
            activeDoc?.Views.Redraw();
        }

        public void RemoveInstance(SplatInstance inst, RhinoDoc doc)
        {
            if (inst == null)
                return;

            if (inst.ProxyObjectId != Guid.Empty && doc != null)
            {
                try
                {
                    doc.Objects.Delete(inst.ProxyObjectId, true);
                }
                catch
                {
                }
            }

            inst.Cleanup();
            _instances.Remove(inst);
            if (_instances.Count == 0)
                Enabled = false;

            doc?.Views.Redraw();
        }

        private void CleanupAll()
        {
            foreach (SplatInstance instance in _instances)
                instance.Cleanup();

            _instances.Clear();

            if (_globalGlInitialized)
            {
                try
                {
                    if (_quadVbo != 0)
                        OpenGL.glDeleteBuffers(1, ref _quadVbo);
                    if (_program != 0)
                        OpenGL.glDeleteProgram(_program);
                }
                catch
                {
                }
                _quadVbo = 0;
                _program = 0;
                _globalGlInitialized = false;
            }
        }

        // ========== 实例访问 ==========

        public SplatInstance GetActiveOrFirstInstance()
        {
            if (_instances.Count == 0)
                return null;
            if (_instances.Count == 1)
                return _instances[0];

            RhinoDoc doc = RhinoDoc.ActiveDoc;
            if (doc != null)
            {
                foreach (RhinoObject rhinoObj in doc.Objects.GetSelectedObjects(false, false))
                {
                    SplatInstance instance = _instances.Find(x => x.ProxyObjectId == rhinoObj.Id);
                    if (instance != null)
                        return instance;
                }
            }
            return _instances[_instances.Count - 1];
        }

        /// <summary>按选中代理点云确定目标实例；多个实例且无选中时提示用户选择。</summary>
        public static SplatInstance GetTargetInstance(RhinoDoc doc, string prompt)
        {
            if (Instance == null || Instance._instances.Count == 0)
                return null;

            if (Instance._instances.Count == 1)
                return Instance._instances[0];

            RhinoObject[] selected = doc != null
                ? System.Linq.Enumerable.ToArray(doc.Objects.GetSelectedObjects(false, false))
                : null;
            if (selected != null)
            {
                foreach (RhinoObject rhinoObj in selected)
                {
                    SplatInstance instance = Instance._instances.Find(x => x.ProxyObjectId == rhinoObj.Id);
                    if (instance != null)
                        return instance;
                }
            }

            GetObject go = new GetObject();
            go.SetCommandPrompt(prompt);
            go.GeometryFilter = ObjectType.PointSet;
            go.AcceptNothing(false);
            go.SetCustomGeometryFilter((rhinoObject, geometry, componentIndex) =>
                Instance._instances.Exists(x => x.ProxyObjectId == rhinoObject.Id));
            if (go.Get() == GetResult.Object)
            {
                ObjRef objRef = go.Object(0);
                return Instance._instances.Find(x => x.ProxyObjectId == objRef.ObjectId);
            }
            return null;
        }

        public Splat[] GetSplats()
        {
            SplatInstance instance = GetActiveOrFirstInstance();
            return instance?.Splats;
        }

        public void UpdateSplats(Splat[] newSplats)
        {
            SplatInstance instance = GetActiveOrFirstInstance();
            instance?.SetSplats(newSplats);
        }

        public void UpdateInstanceSplats(Guid proxyId, Splat[] newSplats)
        {
            SplatInstance instance = _instances.Find(x => x.ProxyObjectId == proxyId);
            instance?.SetSplats(newSplats);
        }

        public void UpdateInstanceSplats(Guid proxyId, Splat[] newSplats, Point3d origP0, Point3d origP1, Point3d origP2)
        {
            SplatInstance instance = _instances.Find(x => x.ProxyObjectId == proxyId);
            if (instance != null)
            {
                instance.SetSplats(newSplats);
                instance.OrigP0 = origP0;
                instance.OrigP1 = origP1;
                instance.OrigP2 = origP2;
            }
        }

        // ========== 绘制 ==========

        protected override void CalculateBoundingBox(CalculateBoundingBoxEventArgs e)
        {
            base.CalculateBoundingBox(e);
            BoundingBox box = SplatBoundingBox;
            if (box.IsValid)
                e.IncludeBoundingBox(box);
        }

        protected override void PreDrawObject(DrawObjectEventArgs e)
        {
            if (e.RhinoObject != null)
            {
                SplatInstance instance = _instances.Find(x => x.ProxyObjectId == e.RhinoObject.Id);
                if (instance != null)
                {
                    // 代理点云不直接绘制，由 splat 渲染替代；选中时画黄色包围盒
                    if (e.RhinoObject.IsSelected(false) > 0)
                    {
                        BoundingBox box = instance.BBox;
                        if (box.IsValid)
                            e.Display.DrawBox(box, Color.Yellow, 2);
                    }
                    e.DrawObject = false;
                }
            }
            base.PreDrawObject(e);
        }

        protected override void PostDrawObjects(DrawEventArgs e)
        {
            base.PostDrawObjects(e);
            if (_instances.Count == 0)
                return;

            RhinoViewport viewport = e.Viewport;
            Point3d camPos = viewport.CameraLocation;
            Vector3d camLook = viewport.CameraDirection;
            if (camLook.Length > 1e-6)
                camLook.Unitize();
            bool isOrtho = viewport.IsParallelProjection;

            OpenGL.Initialize();
            if (!_globalGlInitialized && !InitGlobalGL())
                return;

            // ---- 跟踪代理点云的变换（用户移动/旋转/缩放代理时同步 splat） ----
            foreach (SplatInstance instance in _instances)
            {
                if (!instance.IsLoaded || instance.Splats == null || instance.Splats.Length == 0 || instance.ProxyObjectId == Guid.Empty)
                    continue;

                RhinoObject proxyObj = e.RhinoDoc.Objects.FindId(instance.ProxyObjectId);
                if (proxyObj == null || proxyObj.IsDeleted)
                    continue;

                PointCloud cloud = proxyObj.Geometry as PointCloud;
                if (cloud == null || cloud.Count < 3)
                    continue;

                Point3d p0 = cloud[0].Location;
                Point3d p1 = cloud[1].Location;
                Point3d p2 = cloud[2].Location;

                if (instance.OrigP0 == Point3d.Unset)
                {
                    Point3d o0, o1, o2;
                    if (TryParsePoint(proxyObj.Attributes.GetUserString("OrigP0"), out o0) &&
                        TryParsePoint(proxyObj.Attributes.GetUserString("OrigP1"), out o1) &&
                        TryParsePoint(proxyObj.Attributes.GetUserString("OrigP2"), out o2))
                    {
                        instance.OrigP0 = o0;
                        instance.OrigP1 = o1;
                        instance.OrigP2 = o2;
                    }
                    else
                    {
                        instance.OrigP0 = p0;
                        instance.OrigP1 = p1;
                        instance.OrigP2 = p2;
                        proxyObj.Attributes.SetUserString("OrigP0", FormatPoint(p0));
                        proxyObj.Attributes.SetUserString("OrigP1", FormatPoint(p1));
                        proxyObj.Attributes.SetUserString("OrigP2", FormatPoint(p2));
                        proxyObj.CommitChanges();
                    }
                    instance.LastProxyP0 = Point3d.Unset;
                    instance.LastProxyP1 = Point3d.Unset;
                    instance.LastProxyP2 = Point3d.Unset;
                }

                if (instance.LastProxyP0 == Point3d.Unset ||
                    p0.DistanceTo(instance.LastProxyP0) > 1e-5 ||
                    p1.DistanceTo(instance.LastProxyP1) > 1e-5 ||
                    p2.DistanceTo(instance.LastProxyP2) > 1e-5)
                {
                    double origDist = instance.OrigP0.DistanceTo(instance.OrigP1);
                    double curDist = p0.DistanceTo(p1);
                    double scale = origDist > 1e-6 ? curDist / origDist : 1.0;
                    Transform planeToPlane = Transform.PlaneToPlane(
                        ConstructPlane(instance.OrigP0, instance.OrigP1, instance.OrigP2),
                        ConstructPlane(p0, p1, p2));
                    Transform combined = Transform.Scale(p0, scale) * planeToPlane;
                    instance.UpdateTransformedSplats(combined, scale);
                    instance.LastProxyP0 = p0;
                    instance.LastProxyP1 = p1;
                    instance.LastProxyP2 = p2;
                }
            }

            // ---- 收集裁剪平面 ----
            List<Plane> clipPlanes = new List<Plane>();
            RhinoObject[] clipObjects = e.RhinoDoc.Objects.FindByObjectType(ObjectType.ClipPlane);
            if (clipObjects != null)
            {
                foreach (RhinoObject clipObj in clipObjects)
                {
                    ClippingPlaneObject clipPlaneObj = clipObj as ClippingPlaneObject;
                    if (clipPlaneObj == null || clipObj.IsDeleted)
                        continue;

                    ClippingPlaneSurface surface = clipPlaneObj.ClippingPlaneGeometry;
                    if (surface == null)
                        continue;

                    bool appliesToViewport = false;
                    Guid[] viewportIds = surface.ViewportIds();
                    foreach (Guid vpId in viewportIds)
                    {
                        if (vpId == e.Viewport.Id)
                        {
                            appliesToViewport = true;
                            break;
                        }
                    }
                    if (appliesToViewport)
                        clipPlanes.Add(surface.Plane);
                }
            }

            int clipCount = Math.Min(6, clipPlanes.Count);
            float[] clipPlaneData = new float[clipCount * 4];
            for (int i = 0; i < clipCount; i++)
            {
                Plane plane = clipPlanes[i];
                clipPlaneData[i * 4] = (float)plane.Normal.X;
                clipPlaneData[i * 4 + 1] = (float)plane.Normal.Y;
                clipPlaneData[i * 4 + 2] = (float)plane.Normal.Z;
                clipPlaneData[i * 4 + 3] = (float)-(plane.Normal.X * plane.Origin.X + plane.Normal.Y * plane.Origin.Y + plane.Normal.Z * plane.Origin.Z);
            }

            // ---- 设置着色器全局 uniform ----
            float[] worldToCamera = e.Display.GetOpenGLWorldToCamera(true);
            float[] cameraToClip = e.Display.GetOpenGLCameraToClip();

            OpenGL.glUseProgram(_program);
            OpenGL.glUniformMatrix4fv(_viewLoc, 1, false, worldToCamera);
            OpenGL.glUniformMatrix4fv(_projLoc, 1, false, cameraToClip);

            int[] viewportParams = new int[4];
            OpenGL.glGetIntegerv(OpenGL.GL_VIEWPORT, viewportParams);
            float vpWidth = viewportParams[2];
            float vpHeight = viewportParams[3];
            OpenGL.glUniform2f(_viewportSizeLoc, vpWidth, vpHeight);

            if (_clipPlanesLoc != -1 && clipCount > 0)
                OpenGL.glUniform4fv(_clipPlanesLoc, clipCount, clipPlaneData);
            if (_clipPlaneCountLoc != -1)
                OpenGL.glUniform1i(_clipPlaneCountLoc, clipCount);
            if (_splatScaleLoc != -1)
                OpenGL.glUniform1f(_splatScaleLoc, SplatScale);
            if (_filterRadiusLoc != -1)
                OpenGL.glUniform1f(_filterRadiusLoc, FilterRadius);
            if (_isOrthoLoc != -1)
                OpenGL.glUniform1i(_isOrthoLoc, isOrtho ? 1 : 0);

            // ---- 保存并修改 GL 状态（premultiplied alpha 混合 + GL_LESS 深度测试，不写深度） ----
            int[] blendEnabled = new int[1];
            int[] depthEnabled = new int[1];
            OpenGL.glGetIntegerv(OpenGL.GL_BLEND, blendEnabled);
            OpenGL.glGetIntegerv(OpenGL.GL_DEPTH_TEST, depthEnabled);

            OpenGL.glEnable(OpenGL.GL_BLEND);
            OpenGL.glBlendFunc(OpenGL.GL_SRC_ALPHA, OpenGL.GL_ONE_MINUS_SRC_ALPHA);
            OpenGL.glEnable(OpenGL.GL_DEPTH_TEST);
            OpenGL.glDepthFunc(OpenGL.GL_LESS);
            OpenGL.glDepthMask(0);

            // ---- 收集可见实例并按相机距离排序（远 -> 近） ----
            RhinoDoc doc = e.RhinoDoc;
            List<SplatInstance> visibleInstances = new List<SplatInstance>();
            foreach (SplatInstance instance in _instances)
            {
                if (!instance.IsLoaded || instance.Splats == null || instance.Splats.Length == 0)
                    continue;
                if (instance.ProxyObjectId != Guid.Empty && doc != null)
                {
                    RhinoObject proxyObj = doc.Objects.FindId(instance.ProxyObjectId);
                    if (proxyObj == null || proxyObj.IsDeleted)
                        continue;
                }
                visibleInstances.Add(instance);
            }

            visibleInstances.Sort((a, b) =>
            {
                if (isOrtho)
                {
                    double da = (a.BBox.Center - camPos) * camLook;
                    double db = (b.BBox.Center - camPos) * camLook;
                    return db.CompareTo(da);
                }
                double distA = camPos.DistanceTo(a.BBox.Center);
                double distB = camPos.DistanceTo(b.BBox.Center);
                return distB.CompareTo(distA);
            });

            foreach (SplatInstance instance in visibleInstances)
                RenderInstance(instance, e, camPos, camLook);

            // ---- 恢复 GL 状态 ----
            OpenGL.glUseProgram(0);
            OpenGL.glDepthMask(1);
            if (blendEnabled[0] == 0)
                OpenGL.glDisable(OpenGL.GL_BLEND);
            if (depthEnabled[0] == 0)
                OpenGL.glDisable(OpenGL.GL_DEPTH_TEST);
        }

        private void RenderInstance(SplatInstance inst, DrawEventArgs e, Point3d camPos, Vector3d camLook)
        {
            if (!inst.GlInitialized && !inst.InitGL())
                return;

            if (inst.GpuDataDirty)
            {
                inst.UpdateGPUData();
                inst.GpuDataDirty = false;
            }

            // 获取/创建该视口的排序状态
            ViewportState state;
            lock (inst.ViewportStates)
            {
                if (!inst.ViewportStates.TryGetValue(e.Viewport.Id, out state))
                {
                    state = new ViewportState
                    {
                        Indices = new float[inst.Splats.Length],
                        SortKeys = new uint[inst.Splats.Length],
                        TempKeys = new uint[inst.Splats.Length],
                        SortIndices = new float[inst.Splats.Length],
                        TempIndices = new float[inst.Splats.Length]
                    };
                    OpenGL.glGenBuffers(1, out state.IndexVbo);
                    OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, state.IndexVbo);
                    GCHandle handle = GCHandle.Alloc(state.Indices, GCHandleType.Pinned);
                    OpenGL.glBufferData(OpenGL.GL_ARRAY_BUFFER, (IntPtr)(state.Indices.Length * 4), handle.AddrOfPinnedObject(), OpenGL.GL_DYNAMIC_DRAW);
                    handle.Free();
                    OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, 0);
                    inst.ViewportStates[e.Viewport.Id] = state;
                }
            }

            bool isOrtho = e.Viewport.IsParallelProjection;

            // 相机未变化则跳过重新排序
            bool needResort = true;
            if (camPos.DistanceTo(state.LastCamPos) <= 0.05)
            {
                Vector3d lookDelta = camLook - state.LastCamLook;
                if (lookDelta.Length <= 0.05 && isOrtho == state.LastIsOrtho)
                    needResort = false;
            }

            if (needResort)
            {
                state.LastCamPos = camPos;
                state.LastCamLook = camLook;
                state.LastIsOrtho = isOrtho;

                int count = inst.Splats.Length;
                float cx = (float)camPos.X, cy = (float)camPos.Y, cz = (float)camPos.Z;
                float lx = (float)camLook.X, ly = (float)camLook.Y, lz = (float)camLook.Z;
                Splat[] splats = inst.Splats;
                uint[] sortKeys = state.SortKeys;
                float[] sortIndices = state.SortIndices;

                // 计算深度键（可比较的 float 位模式），并行填充
                Parallel.For(0, count, idx =>
                {
                    Splat splat = splats[idx];
                    float dx = splat.X - cx;
                    float dy = splat.Y - cy;
                    float dz = splat.Z - cz;
                    float depth = -(dx * lx + dy * ly + dz * lz);

                    FloatUIntUnion union = default(FloatUIntUnion);
                    union.F = depth;
                    uint bits = union.U;
                    sortKeys[idx] = (bits & 0x80000000) != 0 ? ~bits : (bits ^ 0x80000000);
                    sortIndices[idx] = idx;
                });

                RadixSort(state.SortKeys, state.SortIndices, state.TempKeys, state.TempIndices, count);
                Array.Copy(state.SortIndices, state.Indices, count);

                OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, state.IndexVbo);
                GCHandle handle = GCHandle.Alloc(state.Indices, GCHandleType.Pinned);
                OpenGL.glBufferSubData(OpenGL.GL_ARRAY_BUFFER, IntPtr.Zero, (IntPtr)(count * 4), handle.AddrOfPinnedObject());
                handle.Free();
                OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, 0);
            }

            // 绑定 splat 数据纹理
            OpenGL.glActiveTexture(OpenGL.GL_TEXTURE0);
            OpenGL.glBindTexture(OpenGL.GL_TEXTURE_BUFFER, inst.PosTex);
            OpenGL.glUniform1i(_posSamplerLoc, 0);
            OpenGL.glActiveTexture(OpenGL.GL_TEXTURE0 + 1);
            OpenGL.glBindTexture(OpenGL.GL_TEXTURE_BUFFER, inst.ScaleTex);
            OpenGL.glUniform1i(_scaleSamplerLoc, 1);
            OpenGL.glActiveTexture(OpenGL.GL_TEXTURE0 + 2);
            OpenGL.glBindTexture(OpenGL.GL_TEXTURE_BUFFER, inst.RotTex);
            OpenGL.glUniform1i(_rotSamplerLoc, 2);
            OpenGL.glActiveTexture(OpenGL.GL_TEXTURE0 + 3);
            OpenGL.glBindTexture(OpenGL.GL_TEXTURE_BUFFER, inst.ColorTex);
            OpenGL.glUniform1i(_colorSamplerLoc, 3);

            if (!inst.PrintedCamSpace && inst.Splats.Length > 0)
            {
                Splat splat = inst.Splats[0];
                Point3d camSpacePt = e.Viewport.GetTransform(CoordinateSystem.World, CoordinateSystem.Camera) * new Point3d(splat.X, splat.Y, splat.Z);
                RhinoApp.WriteLine($"[Splat Render] First Splat CamSpace: Z={camSpacePt.Z:F4} (CamLoc={camPos}, SplatWorld=({splat.X:F2}, {splat.Y:F2}, {splat.Z:F2}))");
                inst.PrintedCamSpace = true;
            }

            // 绘制：每个 splat 一个实例化的四边形条带
            OpenGL.glBindVertexArray(0);
            OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, _quadVbo);
            OpenGL.glEnableVertexAttribArray(0);
            OpenGL.glVertexAttribPointer(0, 2, OpenGL.GL_FLOAT, false, 0, IntPtr.Zero);
            OpenGL.glVertexAttribDivisor(0, 0);

            OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, state.IndexVbo);
            OpenGL.glEnableVertexAttribArray(1);
            OpenGL.glVertexAttribPointer(1, 1, OpenGL.GL_FLOAT, false, 0, IntPtr.Zero);
            OpenGL.glVertexAttribDivisor(1, 1);

            OpenGL.glDrawArraysInstanced(OpenGL.GL_TRIANGLE_STRIP, 0, 4, inst.Splats.Length);

            OpenGL.glDisableVertexAttribArray(0);
            OpenGL.glDisableVertexAttribArray(1);
            OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, 0);
        }

        // ========== 着色器编译 ==========

        private bool InitGlobalGL()
        {
            try
            {
                const string vertexSource = @"#version 330 core
layout(location = 0) in vec2 in_Position;    // Quad coordinate (-1,-1) to (1,1)
layout(location = 1) in float in_SplatIndex;  // Instanced index

out vec2 v_QuadCoord;
out vec4 v_Color;
out float v_Fade;

uniform mat4 u_View;
uniform mat4 u_Proj;
uniform vec2 u_ViewportSize;
uniform int u_IsOrtho;

uniform samplerBuffer u_PosSampler;
uniform samplerBuffer u_ScaleSampler;
uniform samplerBuffer u_RotSampler;
uniform samplerBuffer u_ColorSampler;

uniform vec4 u_ClipPlanes[6];
uniform int u_ClipPlaneCount;

uniform float u_SplatScale;
uniform float u_FilterRadius;

mat3 computeRotation(vec4 q) {
    float r = q.x;
    float x = q.y;
    float y = q.z;
    float z = q.w;
    return mat3(
        1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y + r*z), 2.0 * (x*z - r*y), // Col 0
        2.0 * (x*y - r*z), 1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z + r*x), // Col 1
        2.0 * (x*z + r*y), 2.0 * (y*z - r*x), 1.0 - 2.0 * (x*x + y*y)  // Col 2
    );
}

void main() {
    int idx = int(in_SplatIndex + 0.5);
    vec3 splatPos = texelFetch(u_PosSampler, idx).xyz;

    for (int i = 0; i < u_ClipPlaneCount; i++) {
        if (dot(vec4(splatPos, 1.0), u_ClipPlanes[i]) < 0.0) {
            gl_Position = vec4(0.0, 0.0, 0.0, 0.0);
            v_Fade = 0.0;
            return;
        }
    }

    vec3 splatScale = texelFetch(u_ScaleSampler, idx).xyz * u_SplatScale;
    splatScale = max(splatScale, vec3(1e-6)); // guard against degenerate scales (needle artifacts)
    vec4 splatRot = texelFetch(u_RotSampler, idx);
    vec4 splatColor = texelFetch(u_ColorSampler, idx);

    v_Color = splatColor;
    v_QuadCoord = in_Position * 3.0; // quad local coordinate in sigma units [-3, 3]

    mat3 R = computeRotation(splatRot);
    mat3 S = mat3(
        splatScale.x, 0.0, 0.0,
        0.0, splatScale.y, 0.0,
        0.0, 0.0, splatScale.z
    );
    mat3 M = R * S;
    mat3 Sigma = M * transpose(M);

    vec4 camSpacePos = u_View * vec4(splatPos, 1.0);
    vec4 clip = u_Proj * camSpacePos;
    float w = clip.w;
    if (w <= 0.0) {
        gl_Position = vec4(0.0, 0.0, 0.0, 0.0);
        v_Fade = 0.0;
        return;
    }

    // Frustum culling with a margin: skips splats whose center is far outside
    // the view and prevents off-screen giants from rasterizing huge quads.
    float lim = 1.2 * w;
    if (clip.z < -w || clip.z > w || abs(clip.x) > lim || abs(clip.y) > lim) {
        gl_Position = vec4(0.0, 0.0, 0.0, 0.0);
        v_Fade = 0.0;
        return;
    }

    bool isOrtho = (u_IsOrtho == 1 || (u_Proj[3][3] == 1.0 && u_Proj[2][3] == 0.0));
    float nearPlane = 0.1;
    if (!isOrtho && u_Proj[2][2] - 1.0 != 0.0) {
        nearPlane = max(0.01, u_Proj[2][3] / (u_Proj[2][2] - 1.0));
    }

    float fx = u_Proj[0][0] * (u_ViewportSize.x * 0.5);
    float fy = u_Proj[1][1] * (u_ViewportSize.y * 0.5);

    mat3 J;
    if (isOrtho) {
        J = mat3(
            vec3(fx, 0.0, 0.0),
            vec3(0.0, fy, 0.0),
            vec3(0.0, 0.0, 0.0)
        );
    } else {
        // Clamped perspective Jacobian: the tangential terms explode with 1/z^2
        // near the camera and produce giant streaky splats. Clamping tx/ty to
        // slightly beyond the frustum tangents (reference implementation) is
        // smooth and independent of the scene scale.
        float z = camSpacePos.z; // negative in front of the camera
        float limX = 1.3 / u_Proj[0][0];
        float limY = 1.3 / u_Proj[1][1];
        float tx = clamp(camSpacePos.x / -z, -limX, limX) * -z;
        float ty = clamp(camSpacePos.y / -z, -limY, limY) * -z;
        J = mat3(
            vec3(-fx / z, 0.0, 0.0),
            vec3(0.0, -fy / z, 0.0),
            vec3(fx * tx / (z * z), fy * ty / (z * z), 0.0)
        );
    }

    mat3 W = mat3(u_View);
    mat3 T = J * W;
    mat3 Sigma2D = T * Sigma * transpose(T);

    Sigma2D[0][0] += u_FilterRadius;
    Sigma2D[1][1] += u_FilterRadius;

    float a = Sigma2D[0][0];
    float b = Sigma2D[0][1];
    float c = Sigma2D[1][1];

    float mid = 0.5 * (a + c);
    float rad = length(vec2(0.5 * (a - c), b));
    float lambda1 = mid + rad;
    float lambda2 = mid - rad;
    if (lambda2 <= 0.0) {
        gl_Position = vec4(0.0, 0.0, 0.0, 0.0);
        v_Fade = 0.0;
        return;
    }

    // Major-axis eigenvector, cancellation-free form (reference implementation).
    // (lambda1 - a) is pure float noise for near axis-aligned splats and would
    // flip the ellipse 90 degrees (needle artifacts); (lambda1 - c) is well
    // conditioned when a >= c.
    vec2 majorAxis = (a >= c) ? vec2(lambda1 - c, b) : vec2(b, lambda1 - a);
    float axisLen = length(majorAxis);
    majorAxis = axisLen > 1e-20 ? majorAxis / axisLen : ((a >= c) ? vec2(1.0, 0.0) : vec2(0.0, 1.0));
    vec2 minorAxis = vec2(-majorAxis.y, majorAxis.x);

    // eigenvalue floor (~0.1px radius): avoids degenerate zero-size quads
    float s1 = max(sqrt(lambda1), 0.1);
    float s2 = max(sqrt(lambda2), 0.1);

    // anti-needle: cap single-axis radius so a giant near-camera splat cannot
    // rasterize a screen-filling quad (half-extent <= 3 sigma <= ~1023 px).
    s1 = min(s1, 341.0);
    s2 = min(s2, 341.0);

    float nearFade = clamp((-camSpacePos.z - nearPlane) / (nearPlane * 1.5 + 0.01), 0.0, 1.0);
    v_Fade = nearFade;

    if (v_Fade < 0.01) {
        gl_Position = vec4(0.0, 0.0, 0.0, 0.0);
        return;
    }

    // Exact projection mapping: the fragment evaluates exp(-0.5 * dot(v_QuadCoord, v_QuadCoord))
    // with v_QuadCoord = in_Position * 3.0, so the screen offset must use the
    // same 3.0 factor in sigma radii for the alpha profile to match the true
    // projected 2D gaussian (a mismatch renders splats too narrow and sharp).
    vec2 pixelOffset = (in_Position.x * majorAxis * s1 * 3.0) +
                       (in_Position.y * minorAxis * s2 * 3.0);

    vec2 screenOffset = pixelOffset / (u_ViewportSize * 0.5);
    gl_Position = clip;
    gl_Position.xy += screenOffset * gl_Position.w;
}";

                const string fragmentSource = @"#version 330 core
in vec2 v_QuadCoord;
in vec4 v_Color;
in float v_Fade;
out vec4 fragColor;

void main() {
    float d2 = dot(v_QuadCoord, v_QuadCoord);
    if (d2 > 9.0) {
        discard;
    }
    float alpha = exp(-0.5 * d2) * v_Color.a * v_Fade;
    if (alpha < 0.005) {
        discard;
    }
    fragColor = vec4(v_Color.rgb, alpha);
}";

                uint vertexShader = CompileShader(OpenGL.GL_VERTEX_SHADER, vertexSource);
                uint fragmentShader = CompileShader(OpenGL.GL_FRAGMENT_SHADER, fragmentSource);
                if (vertexShader == 0 || fragmentShader == 0)
                    return false;

                _program = OpenGL.glCreateProgram();
                OpenGL.glAttachShader(_program, vertexShader);
                OpenGL.glAttachShader(_program, fragmentShader);
                OpenGL.glLinkProgram(_program);
                OpenGL.glDeleteShader(vertexShader);
                OpenGL.glDeleteShader(fragmentShader);

                int linkStatus;
                OpenGL.glGetProgramiv(_program, OpenGL.GL_LINK_STATUS, out linkStatus);
                if (linkStatus == 0)
                {
                    int logLength;
                    OpenGL.glGetProgramiv(_program, OpenGL.GL_INFO_LOG_LENGTH, out logLength);
                    StringBuilder infoLog = new StringBuilder(logLength);
                    int dummy;
                    OpenGL.glGetProgramInfoLog(_program, logLength, out dummy, infoLog);
                    RhinoApp.WriteLine("Shader linking error: " + infoLog.ToString());
                    return false;
                }

                _viewLoc = OpenGL.glGetUniformLocation(_program, "u_View");
                _projLoc = OpenGL.glGetUniformLocation(_program, "u_Proj");
                _viewportSizeLoc = OpenGL.glGetUniformLocation(_program, "u_ViewportSize");
                _clipPlanesLoc = OpenGL.glGetUniformLocation(_program, "u_ClipPlanes");
                _clipPlaneCountLoc = OpenGL.glGetUniformLocation(_program, "u_ClipPlaneCount");
                _posSamplerLoc = OpenGL.glGetUniformLocation(_program, "u_PosSampler");
                _scaleSamplerLoc = OpenGL.glGetUniformLocation(_program, "u_ScaleSampler");
                _rotSamplerLoc = OpenGL.glGetUniformLocation(_program, "u_RotSampler");
                _colorSamplerLoc = OpenGL.glGetUniformLocation(_program, "u_ColorSampler");
                _splatScaleLoc = OpenGL.glGetUniformLocation(_program, "u_SplatScale");
                _filterRadiusLoc = OpenGL.glGetUniformLocation(_program, "u_FilterRadius");
                _isOrthoLoc = OpenGL.glGetUniformLocation(_program, "u_IsOrtho");

                // 单位四边形顶点（三角形条带）
                float[] quadVertices = new float[] { -1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f };
                OpenGL.glGenBuffers(1, out _quadVbo);
                OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, _quadVbo);
                GCHandle handle = GCHandle.Alloc(quadVertices, GCHandleType.Pinned);
                OpenGL.glBufferData(OpenGL.GL_ARRAY_BUFFER, (IntPtr)(quadVertices.Length * 4), handle.AddrOfPinnedObject(), OpenGL.GL_STATIC_DRAW);
                handle.Free();
                OpenGL.glBindBuffer(OpenGL.GL_ARRAY_BUFFER, 0);

                _globalGlInitialized = true;
                return true;
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine("Failed to initialize global OpenGL: " + ex.Message);
                return false;
            }
        }

        private static uint CompileShader(uint type, string source)
        {
            uint shader = OpenGL.glCreateShader(type);
            string[] sourceArray = new string[] { source };
            // 必须传 UTF-8 字节数而非 char 数，否则源码含非 ASCII 字符时会被驱动截断
            int[] lengths = new int[] { System.Text.Encoding.UTF8.GetByteCount(source) };
            OpenGL.glShaderSource(shader, 1, sourceArray, lengths);
            OpenGL.glCompileShader(shader);

            int compileStatus;
            OpenGL.glGetShaderiv(shader, OpenGL.GL_COMPILE_STATUS, out compileStatus);
            if (compileStatus == 0)
            {
                int logLength;
                OpenGL.glGetShaderiv(shader, OpenGL.GL_INFO_LOG_LENGTH, out logLength);
                StringBuilder infoLog = new StringBuilder(logLength);
                int dummy;
                OpenGL.glGetShaderInfoLog(shader, logLength, out dummy, infoLog);
                RhinoApp.WriteLine($"Shader compilation error (Type {type}): {infoLog.ToString()}");
                OpenGL.glDeleteShader(shader);
                return 0;
            }
            return shader;
        }

        // ========== 基数排序（按深度键） ==========

        private static void RadixPass(uint[] inKeys, float[] inValues, uint[] outKeys, float[] outValues, int[] histogram, int[] offsets, int count, int shift)
        {
            Array.Clear(histogram, 0, 256);
            for (int i = 0; i < count; i++)
            {
                int bucket = (int)((inKeys[i] >> shift) & 0xff);
                histogram[bucket]++;
            }

            offsets[0] = 0;
            for (int j = 1; j < 256; j++)
                offsets[j] = offsets[j - 1] + histogram[j - 1];

            for (int k = 0; k < count; k++)
            {
                int bucket = (int)((inKeys[k] >> shift) & 0xff);
                int index = offsets[bucket]++;
                outKeys[index] = inKeys[k];
                outValues[index] = inValues[k];
            }
        }

        private static void RadixSort(uint[] keys, float[] values, uint[] tempKeys, float[] tempValues, int count)
        {
            int[] histogram = new int[256];
            int[] offsets = new int[256];
            RadixPass(keys, values, tempKeys, tempValues, histogram, offsets, count, 0);
            RadixPass(tempKeys, tempValues, keys, values, histogram, offsets, count, 8);
            RadixPass(keys, values, tempKeys, tempValues, histogram, offsets, count, 16);
            RadixPass(tempKeys, tempValues, keys, values, histogram, offsets, count, 24);
        }

        // ========== 撤销 / 重做 ==========

        public static void RegisterImportUndo(RhinoDoc doc, string actionDescription, SplatInstance importedInstance)
        {
            doc.AddCustomUndoEvent(actionDescription, (sender, e) =>
            {
                if (Instance != null && importedInstance != null)
                {
                    Instance.RemoveInstance(importedInstance, doc);
                    RegisterImportRedo(doc, actionDescription, importedInstance);
                }
                doc.Views.Redraw();
            });
        }

        private static void RegisterImportRedo(RhinoDoc doc, string actionDescription, SplatInstance importedInstance)
        {
            doc.AddCustomUndoEvent(actionDescription, (sender, e) =>
            {
                if (Instance == null)
                    Instance = new SplatDisplayConduit();

                // 重新创建代理点云（保持原 ObjectId）
                if (importedInstance.ProxyObjectId != Guid.Empty && doc != null)
                {
                    try
                    {
                        PointCloud cloud = new PointCloud();
                        int step = Math.Max(1, importedInstance.Splats.Length / 50000);
                        for (int i = 0; i < importedInstance.Splats.Length; i += step)
                        {
                            Splat splat = importedInstance.Splats[i];
                            cloud.Add(new Point3d(splat.X, splat.Y, splat.Z), Color.FromArgb(splat.R, splat.G, splat.B));
                        }
                        while (cloud.Count < 3)
                            cloud.Add(new Point3d(0, 0, 0), Color.Black);

                        ObjectAttributes attributes = new ObjectAttributes();
                        attributes.ObjectId = importedInstance.ProxyObjectId;
                        doc.Objects.AddPointCloud(cloud, attributes);
                    }
                    catch (Exception ex)
                    {
                        RhinoApp.WriteLine("Error restoring proxy point cloud: " + ex.Message);
                    }
                }

                Instance._instances.Add(importedInstance);
                Instance.Enabled = true;
                RegisterImportUndo(doc, actionDescription, importedInstance);
                doc.Views.Redraw();
            });
        }

        /// <summary>注册"删除 splat 实例"的撤销事件（撤销时重建代理点云并恢复实例）。</summary>
        public static void RegisterDeleteUndo(RhinoDoc doc, string actionDescription, SplatInstance deletedInstance)
        {
            doc.AddCustomUndoEvent(actionDescription, (sender, e) =>
            {
                if (Instance == null)
                    Instance = new SplatDisplayConduit();

                // 重新创建代理点云（保持原 ObjectId）
                if (deletedInstance.ProxyObjectId != Guid.Empty && doc != null)
                {
                    try
                    {
                        PointCloud cloud = new PointCloud();
                        int step = Math.Max(1, deletedInstance.Splats.Length / 50000);
                        for (int i = 0; i < deletedInstance.Splats.Length; i += step)
                        {
                            Splat splat = deletedInstance.Splats[i];
                            cloud.Add(new Point3d(splat.X, splat.Y, splat.Z), Color.FromArgb(splat.R, splat.G, splat.B));
                        }
                        while (cloud.Count < 3)
                            cloud.Add(new Point3d(0, 0, 0), Color.Black);

                        ObjectAttributes attributes = new ObjectAttributes();
                        attributes.ObjectId = deletedInstance.ProxyObjectId;
                        doc.Objects.AddPointCloud(cloud, attributes);
                    }
                    catch (Exception ex)
                    {
                        RhinoApp.WriteLine("Error restoring proxy point cloud: " + ex.Message);
                    }
                }

                Instance._instances.Add(deletedInstance);
                Instance.Enabled = true;
                RegisterDeleteRedo(doc, actionDescription, deletedInstance);
                doc.Views.Redraw();
            });
        }

        private static void RegisterDeleteRedo(RhinoDoc doc, string actionDescription, SplatInstance deletedInstance)
        {
            doc.AddCustomUndoEvent(actionDescription, (sender, e) =>
            {
                if (Instance != null && deletedInstance != null)
                {
                    Instance.RemoveInstance(deletedInstance, doc);
                    RegisterDeleteUndo(doc, actionDescription, deletedInstance);
                }
                doc.Views.Redraw();
            });
        }

        /// <summary>将修改后的 splat 数据缓存到本地（%AppData%\RhinoGaussianSplatting\ModifiedSplats），返回缓存路径。</summary>
        public static string SaveModifiedSplatCache(Guid proxyId, Splat[] splats)
        {
            if (proxyId != Guid.Empty && splats != null && splats.Length > 0)
            {
                try
                {
                    string dir = Path.Combine(
                        System.Environment.GetFolderPath(System.Environment.SpecialFolder.ApplicationData),
                        "RhinoGaussianSplatting", "ModifiedSplats");
                    if (!Directory.Exists(dir))
                        Directory.CreateDirectory(dir);

                    string path = Path.Combine(dir, proxyId.ToString("N") + ".splat");
                    string error;
                    if (SplatLoader.ExportSplat(path, splats, out error))
                        return path;

                    RhinoApp.WriteLine("[RhinoGaussianSplatting] Failed to cache modified splats: " + error);
                }
                catch (Exception ex)
                {
                    RhinoApp.WriteLine("[RhinoGaussianSplatting] Error caching modified splats: " + ex.Message);
                }
            }
            return null;
        }

        /// <summary>
        /// 注册"splat 数据被修改"的撤销事件（裁剪 / 去噪 / 擦除共用）。
        /// 撤销时恢复 beforeSplats（重建代理点云并更新实例），并注册重做事件。
        /// </summary>
        public static void RegisterSplatsModifiedUndo(
            RhinoDoc doc, string description, Guid proxyId,
            Splat[] beforeSplats, Splat[] afterSplats, string previousFilePath)
        {
            doc.AddCustomUndoEvent(description, (sender, e) =>
            {
                if (Instance != null)
                {
                    Point3d p0 = Point3d.Unset, p1 = Point3d.Unset, p2 = Point3d.Unset;
                    if (proxyId != Guid.Empty)
                    {
                        try
                        {
                            PointCloud cloud = new PointCloud();
                            int step = Math.Max(1, beforeSplats.Length / 50000);
                            for (int i = 0; i < beforeSplats.Length; i += step)
                            {
                                Splat splat = beforeSplats[i];
                                cloud.Add(new Point3d(splat.X, splat.Y, splat.Z), Color.FromArgb(splat.R, splat.G, splat.B));
                            }
                            while (cloud.Count < 3)
                                cloud.Add(new Point3d(0, 0, 0), Color.Black);

                            p0 = cloud[0].Location;
                            p1 = cloud[1].Location;
                            p2 = cloud[2].Location;

                            doc.Objects.Replace(proxyId, cloud);
                            RhinoObject proxyObj = doc.Objects.FindId(proxyId);
                            if (proxyObj != null)
                            {
                                if (!string.IsNullOrEmpty(previousFilePath))
                                    proxyObj.Attributes.SetUserString("SplatFilePath", previousFilePath);
                                proxyObj.Attributes.SetUserString("OrigP0", FormatPoint(p0));
                                proxyObj.Attributes.SetUserString("OrigP1", FormatPoint(p1));
                                proxyObj.Attributes.SetUserString("OrigP2", FormatPoint(p2));
                                proxyObj.CommitChanges();
                            }
                        }
                        catch
                        {
                        }
                    }
                    Instance.UpdateInstanceSplats(proxyId, beforeSplats, p0, p1, p2);
                }
                // 注册重做（参数互换）
                RegisterSplatsModifiedUndo(doc, description, proxyId, afterSplats, beforeSplats, previousFilePath);
                doc.Views.Redraw();
            });
        }

        // ========== 文档打开时的自动恢复 ==========

        /// <summary>扫描文档中的 splat 代理对象并重新加载对应的 splat 数据。</summary>
        public static void ScanAndRestoreSplats(RhinoDoc doc)
        {
            if (doc == null)
                return;

            RhinoObject[] pointCloudObjects = doc.Objects.FindByObjectType(ObjectType.PointSet);
            if (pointCloudObjects == null)
                return;

            foreach (RhinoObject obj in pointCloudObjects)
            {
                if (obj == null || obj.IsDeleted)
                    continue;
                if (Instance != null && Instance._instances.Exists(x => x.ProxyObjectId == obj.Id))
                    continue;

                string splatFilePath = obj.Attributes.GetUserString("SplatFilePath");
                string isConverted = obj.Attributes.GetUserString("IsConvertedSplat");

                Point3d origP0 = Point3d.Unset, origP1 = Point3d.Unset, origP2 = Point3d.Unset;
                Point3d p0, p1, p2;
                if (TryParsePoint(obj.Attributes.GetUserString("OrigP0"), out p0) &&
                    TryParsePoint(obj.Attributes.GetUserString("OrigP1"), out p1) &&
                    TryParsePoint(obj.Attributes.GetUserString("OrigP2"), out p2))
                {
                    origP0 = p0;
                    origP1 = p1;
                    origP2 = p2;
                }

                if (!string.IsNullOrEmpty(splatFilePath))
                {
                    string path = splatFilePath;
                    if (!File.Exists(path))
                        path = FindSplatFileFallback(doc, obj.Id, splatFilePath);

                    if (File.Exists(path))
                    {
                        RhinoApp.WriteLine("[RhinoPlugins_Development] Automatically restoring splat dataset from: " + path);
                        try
                        {
                            string errorMessage;
                            string ext = Path.GetExtension(path).ToLowerInvariant();
                            Splat[] splats;
                            switch (ext)
                            {
                                case ".splat":
                                    splats = SplatLoader.LoadSplat(path, out errorMessage);
                                    break;
                                case ".ksplat":
                                    splats = SplatLoader.LoadKSplat(path, out errorMessage);
                                    break;
                                case ".spz":
                                    splats = SpzLoader.LoadSpz(path, out errorMessage);
                                    break;
                                case ".las":
                                    splats = LasLoader.LoadLas(path, out errorMessage);
                                    break;
                                default:
                                    splats = SplatLoader.LoadPly(path, out errorMessage);
                                    break;
                            }

                            if (splats != null && splats.Length > 0)
                            {
                                // 复现导入时的确定性处理：AutoCrop（原始数据）→ UpAxis → LevelFloor
                                bool wasSplatData = ext == ".splat" || ext == ".ksplat" || ext == ".spz" ||
                                                    (ext != ".las" && SplatLoader.LastPlyWasSplat);
                                if (wasSplatData && obj.Attributes.GetUserString("SplatAutoCrop") == "True")
                                {
                                    SplatAutoCropResult crop = SplatAutoCrop.Compute(splats, SplatCropStrength.Medium);
                                    if (crop != null && crop.Kept > 0)
                                        splats = ImportGaussianSplatCommand.FilterSplats(splats, crop.Keep);
                                }

                                if (ext != ".las")
                                {
                                    SplatUpAxis axis;
                                    if (!Enum.TryParse(obj.Attributes.GetUserString("SplatUpAxis"), out axis))
                                        axis = ImportGaussianSplatCommand.DefaultUpAxis(path);
                                    ImportGaussianSplatCommand.ApplyUpAxis(splats, axis);
                                }

                                if (obj.Attributes.GetUserString("SplatLevelFloor") == "True")
                                {
                                    Transform leveling;
                                    double tiltDegrees;
                                    if (SplatFloorLeveler.TryLevel(splats, out leveling, out tiltDegrees))
                                        ImportGaussianSplatCommand.TransformSplats(splats, leveling);
                                }

                                EnableConduit(splats, obj.Id, origP0, origP1, origP2);
                            }
                            else
                            {
                                RhinoApp.WriteLine("[RhinoPlugins_Development] Failed to restore splat model: " + (errorMessage ?? "Unknown error"));
                            }
                        }
                        catch (Exception ex)
                        {
                            RhinoApp.WriteLine("[RhinoPlugins_Development] Error restoring splat: " + ex.Message);
                        }
                    }
                    else
                    {
                        RhinoApp.WriteLine("[RhinoPlugins_Development] Splat proxy found, but original file was not found at: " + splatFilePath);
                    }
                }
                else if (isConverted == "True")
                {
                    // 由点云转换而来的 splat：直接从代理点云数据重建
                    PointCloud cloud = obj.Geometry as PointCloud;
                    if (cloud != null)
                    {
                        RhinoApp.WriteLine("[RhinoPlugins_Development] Restoring converted splat dataset from proxy point cloud...");
                        try
                        {
                            int count = cloud.Count;
                            Splat[] splats = new Splat[count];
                            for (int i = 0; i < count; i++)
                            {
                                PointCloudItem item = cloud[i];
                                splats[i] = new Splat
                                {
                                    X = (float)item.Location.X,
                                    Y = (float)item.Location.Y,
                                    Z = (float)item.Location.Z,
                                    R = item.Color.R,
                                    G = item.Color.G,
                                    B = item.Color.B,
                                    A = item.Color.A,
                                    RotW = 1f
                                };
                            }
                            EnableConduit(splats, obj.Id, origP0, origP1, origP2);
                        }
                        catch (Exception ex)
                        {
                            RhinoApp.WriteLine("[RhinoPlugins_Development] Error restoring converted splat: " + ex.Message);
                        }
                    }
                }
            }
        }

        private static string FindSplatFileFallback(RhinoDoc doc, Guid proxyId, string originalPath)
        {
            // 1. 文档同目录候选
            if (!string.IsNullOrEmpty(doc.Path))
            {
                string docFolder = Path.GetDirectoryName(doc.Path);
                string docName = Path.GetFileNameWithoutExtension(doc.Path);
                if (!string.IsNullOrEmpty(docFolder))
                {
                    string[] candidates = new string[]
                    {
                        Path.Combine(docFolder, Path.GetFileName(originalPath)),
                        Path.Combine(docFolder, docName + "_splats", Path.GetFileName(originalPath)),
                        Path.Combine(docFolder, docName + "_splats", proxyId.ToString("N") + ".splat"),
                        Path.Combine(docFolder, proxyId.ToString("N") + ".splat")
                    };
                    foreach (string candidate in candidates)
                    {
                        if (File.Exists(candidate))
                            return candidate;
                    }
                }
            }

            // 2. AppData 缓存目录（与原版 RhinoGaussianSplatting 插件兼容）
            string cachePath = Path.Combine(
                System.Environment.GetFolderPath(System.Environment.SpecialFolder.ApplicationData),
                "RhinoGaussianSplatting", "ModifiedSplats", proxyId.ToString("N") + ".splat");
            if (File.Exists(cachePath))
                return cachePath;

            return originalPath;
        }

        // ========== 工具方法 ==========

        private Plane ConstructPlane(Point3d p0, Point3d p1, Point3d p2)
        {
            Vector3d xAxis = p1 - p0;
            xAxis.Unitize();
            Vector3d v = p2 - p0;
            Vector3d normal = Vector3d.CrossProduct(xAxis, v);
            normal.Unitize();
            Vector3d yAxis = Vector3d.CrossProduct(normal, xAxis);
            yAxis.Unitize();
            return new Plane(p0, xAxis, yAxis);
        }

        public static string FormatPoint(Point3d pt)
        {
            return string.Format(CultureInfo.InvariantCulture, "{0:R};{1:R};{2:R}", pt.X, pt.Y, pt.Z);
        }

        public static bool TryParsePoint(string s, out Point3d pt)
        {
            pt = Point3d.Unset;
            if (string.IsNullOrEmpty(s))
                return false;

            string[] parts = s.Split(';');
            if (parts.Length != 3)
                return false;

            double x, y, z;
            if (double.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out x) &&
                double.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out y) &&
                double.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out z))
            {
                pt = new Point3d(x, y, z);
                return true;
            }
            return false;
        }
    }
}
