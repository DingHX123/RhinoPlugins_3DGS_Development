using System.Runtime.InteropServices;

namespace RhinoPlugins_Development
{
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct Splat
    {
        public float X;
        public float Y;
        public float Z;
        public float ScaleX;
        public float ScaleY;
        public float ScaleZ;
        public float RotW;
        public float RotX;
        public float RotY;
        public float RotZ;
        public byte R;
        public byte G;
        public byte B;
        public byte A;
    }
}