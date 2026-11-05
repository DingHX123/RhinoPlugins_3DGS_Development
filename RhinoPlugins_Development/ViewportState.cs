using Rhino.Geometry;
using System;

namespace RhinoPlugins_Development
{
    /// <summary>每个视口的排序状态（深度排序缓存 + 索引 VBO）。</summary>
    public class ViewportState
    {
        public uint IndexVbo;
        public float[] Indices;
        public Vector3d LastCamLook = Vector3d.Zero;
        public Point3d LastCamPos = Point3d.Unset;
        public bool LastIsOrtho;
        public float[] SortIndices;
        public uint[] SortKeys;
        public float[] TempIndices;
        public uint[] TempKeys;
    }
}
