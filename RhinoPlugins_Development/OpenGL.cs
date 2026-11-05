using System;
using System.Runtime.InteropServices;

namespace RhinoPlugins_Development
{
    /// <summary>
    /// 轻量 OpenGL 3.3 core 加载器。
    /// 固定功能入口直接 DllImport opengl32.dll；
    /// 核心着色器/VAO 入口通过 wglGetProcAddress 解析，
    /// 失败时回退到 LoadLibrary + GetProcAddress（兼容 net48 与 net7.0-windows）。
    /// </summary>
    public static class OpenGL
    {
        private static IntPtr _glModuleHandle = IntPtr.Zero;
        private static bool _initialized;

        // ===== OpenGL 常量 =====
        public const uint GL_ARRAY_BUFFER = 0x8892;
        public const uint GL_BLEND = 0x0BE2;
        public const uint GL_COMPILE_STATUS = 0x8B81;
        public const uint GL_DEPTH_TEST = 0x0B71;
        public const uint GL_DYNAMIC_DRAW = 0x88E8;
        public const uint GL_ELEMENT_ARRAY_BUFFER = 0x8893;
        public const uint GL_FLOAT = 0x1406;
        public const uint GL_FRAGMENT_SHADER = 0x8B30;
        public const uint GL_INFO_LOG_LENGTH = 0x8B84;
        public const uint GL_LEQUAL = 0x0203;
        public const uint GL_LESS = 0x0201;
        public const uint GL_LINK_STATUS = 0x8B82;
        public const uint GL_ONE = 1;
        public const uint GL_ONE_MINUS_SRC_ALPHA = 0x0303;
        public const uint GL_RGB32F = 0x8815;
        public const uint GL_RGBA32F = 0x8814;
        public const uint GL_RGBA8 = 0x8058;
        public const uint GL_SRC_ALPHA = 770;
        public const uint GL_STATIC_DRAW = 0x88E4;
        public const uint GL_TEXTURE_2D = 0x0DE1;
        public const uint GL_TEXTURE_BUFFER = 0x8C2A;
        public const uint GL_TEXTURE0 = 0x84C0;
        public const uint GL_TRIANGLE_STRIP = 5;
        public const uint GL_TRIANGLES = 4;
        public const uint GL_UNSIGNED_BYTE = 0x1401;
        public const uint GL_UNSIGNED_INT = 0x1405;
        public const uint GL_VERTEX_SHADER = 0x8B31;
        public const uint GL_VIEWPORT = 0x0BA2;

        // ===== 核心函数委托（运行时解析） =====
        public static GLActiveTexture glActiveTexture;
        public static GLAttachShader glAttachShader;
        public static GLBindBuffer glBindBuffer;
        public static GLBindVertexArray glBindVertexArray;
        public static GLBufferData glBufferData;
        public static GLBufferSubData glBufferSubData;
        public static GLCompileShader glCompileShader;
        public static GLCreateProgram glCreateProgram;
        public static GLCreateShader glCreateShader;
        public static GLDeleteBuffers glDeleteBuffers;
        public static GLDeleteProgram glDeleteProgram;
        public static GLDeleteShader glDeleteShader;
        public static GLDeleteVertexArrays glDeleteVertexArrays;
        public static GLDisableVertexAttribArray glDisableVertexAttribArray;
        public static GLDrawArraysInstanced glDrawArraysInstanced;
        public static GLEnableVertexAttribArray glEnableVertexAttribArray;
        public static GLGenBuffers glGenBuffers;
        public static GLGenVertexArrays glGenVertexArrays;
        public static GLGetProgramInfoLog glGetProgramInfoLog;
        public static GLGetProgramiv glGetProgramiv;
        public static GLGetShaderInfoLog glGetShaderInfoLog;
        public static GLGetShaderiv glGetShaderiv;
        public static GLGetUniformLocation glGetUniformLocation;
        public static GLLinkProgram glLinkProgram;
        public static GLShaderSource glShaderSource;
        public static GLTexBuffer glTexBuffer;
        public static GLUniform1f glUniform1f;
        public static GLUniform1i glUniform1i;
        public static GLUniform2f glUniform2f;
        public static GLUniform3f glUniform3f;
        public static GLUniform4f glUniform4f;
        public static GLUniform4fv glUniform4fv;
        public static GLUniformMatrix4fv glUniformMatrix4fv;
        public static GLUseProgram glUseProgram;
        public static GLVertexAttribDivisor glVertexAttribDivisor;
        public static GLVertexAttribPointer glVertexAttribPointer;

        // ===== 固定功能入口（opengl32.dll 直接导出） =====
        [DllImport("opengl32.dll")]
        public static extern void glBindTexture(uint target, uint texture);
        [DllImport("opengl32.dll")]
        public static extern void glBlendFunc(uint sfactor, uint dfactor);
        [DllImport("opengl32.dll")]
        public static extern void glDeleteTextures(int n, ref uint textures);
        [DllImport("opengl32.dll")]
        public static extern void glDepthFunc(uint func);
        [DllImport("opengl32.dll")]
        public static extern void glDepthMask(byte flag);
        [DllImport("opengl32.dll")]
        public static extern void glDisable(uint cap);
        [DllImport("opengl32.dll")]
        public static extern void glEnable(uint cap);
        [DllImport("opengl32.dll")]
        public static extern void glGenTextures(int n, out uint textures);
        [DllImport("opengl32.dll")]
        public static extern void glGetIntegerv(uint pname, int[] parameters);

        [DllImport("opengl32.dll", CharSet = CharSet.Ansi, SetLastError = true)]
        private static extern IntPtr wglGetProcAddress(string procName);

        [DllImport("kernel32.dll", CharSet = CharSet.Ansi, SetLastError = true)]
        private static extern IntPtr LoadLibrary(string fileName);

        [DllImport("kernel32.dll", CharSet = CharSet.Ansi, SetLastError = true)]
        private static extern IntPtr GetProcAddress(IntPtr hModule, string procName);

        /// <summary>解析所有核心函数入口，只需在具备 GL 上下文的线程上调用一次。</summary>
        public static void Initialize()
        {
            if (_initialized)
                return;

            GetOrLoadOpenGLModule();
            glCreateShader = GetProcAddressT<GLCreateShader>("glCreateShader");
            glShaderSource = GetProcAddressT<GLShaderSource>("glShaderSource");
            glCompileShader = GetProcAddressT<GLCompileShader>("glCompileShader");
            glGetShaderiv = GetProcAddressT<GLGetShaderiv>("glGetShaderiv");
            glGetShaderInfoLog = GetProcAddressT<GLGetShaderInfoLog>("glGetShaderInfoLog");
            glCreateProgram = GetProcAddressT<GLCreateProgram>("glCreateProgram");
            glAttachShader = GetProcAddressT<GLAttachShader>("glAttachShader");
            glLinkProgram = GetProcAddressT<GLLinkProgram>("glLinkProgram");
            glGetProgramiv = GetProcAddressT<GLGetProgramiv>("glGetProgramiv");
            glGetProgramInfoLog = GetProcAddressT<GLGetProgramInfoLog>("glGetProgramInfoLog");
            glUseProgram = GetProcAddressT<GLUseProgram>("glUseProgram");
            glDeleteShader = GetProcAddressT<GLDeleteShader>("glDeleteShader");
            glDeleteProgram = GetProcAddressT<GLDeleteProgram>("glDeleteProgram");
            glGetUniformLocation = GetProcAddressT<GLGetUniformLocation>("glGetUniformLocation");
            glUniformMatrix4fv = GetProcAddressT<GLUniformMatrix4fv>("glUniformMatrix4fv");
            glUniform4fv = GetProcAddressT<GLUniform4fv>("glUniform4fv");
            glUniform1f = GetProcAddressT<GLUniform1f>("glUniform1f");
            glUniform3f = GetProcAddressT<GLUniform3f>("glUniform3f");
            glUniform4f = GetProcAddressT<GLUniform4f>("glUniform4f");
            glUniform1i = GetProcAddressT<GLUniform1i>("glUniform1i");
            glUniform2f = GetProcAddressT<GLUniform2f>("glUniform2f");
            glGenBuffers = GetProcAddressT<GLGenBuffers>("glGenBuffers");
            glBindBuffer = GetProcAddressT<GLBindBuffer>("glBindBuffer");
            glBufferData = GetProcAddressT<GLBufferData>("glBufferData");
            glBufferSubData = GetProcAddressT<GLBufferSubData>("glBufferSubData");
            glDeleteBuffers = GetProcAddressT<GLDeleteBuffers>("glDeleteBuffers");
            glGenVertexArrays = GetProcAddressT<GLGenVertexArrays>("glGenVertexArrays");
            glBindVertexArray = GetProcAddressT<GLBindVertexArray>("glBindVertexArray");
            glDeleteVertexArrays = GetProcAddressT<GLDeleteVertexArrays>("glDeleteVertexArrays");
            glEnableVertexAttribArray = GetProcAddressT<GLEnableVertexAttribArray>("glEnableVertexAttribArray");
            glDisableVertexAttribArray = GetProcAddressT<GLDisableVertexAttribArray>("glDisableVertexAttribArray");
            glVertexAttribPointer = GetProcAddressT<GLVertexAttribPointer>("glVertexAttribPointer");
            glVertexAttribDivisor = GetProcAddressT<GLVertexAttribDivisor>("glVertexAttribDivisor");
            glDrawArraysInstanced = GetProcAddressT<GLDrawArraysInstanced>("glDrawArraysInstanced");
            glActiveTexture = GetProcAddressT<GLActiveTexture>("glActiveTexture");
            glTexBuffer = GetProcAddressT<GLTexBuffer>("glTexBuffer");

            _initialized = true;
        }

        private static IntPtr GetOrLoadOpenGLModule()
        {
            if (_glModuleHandle == IntPtr.Zero)
                _glModuleHandle = LoadLibrary("opengl32.dll");
            return _glModuleHandle;
        }

        private static T GetProcAddressT<T>(string name) where T : Delegate
        {
            IntPtr proc = IntPtr.Zero;
            try
            {
                proc = wglGetProcAddress(name);
            }
            catch
            {
            }

            long value = proc.ToInt64();
            // wglGetProcAddress 失败时可能返回 0、-1、1、2、3 等哨兵值
            if (proc == IntPtr.Zero || (value >= -1L && value <= 3L))
            {
                proc = IntPtr.Zero;
                IntPtr module = GetOrLoadOpenGLModule();
                if (module != IntPtr.Zero)
                    proc = GetProcAddress(module, name);
            }

            if (proc == IntPtr.Zero)
                throw new InvalidOperationException("Failed to load OpenGL function pointer for: " + name);

            return Marshal.GetDelegateForFunctionPointer<T>(proc);
        }

        // ===== 委托类型定义 =====
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLActiveTexture(uint texture);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLAttachShader(uint program, uint shader);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLBindBuffer(uint target, uint buffer);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLBindVertexArray(uint array);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLBufferData(uint target, IntPtr size, IntPtr data, uint usage);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLBufferSubData(uint target, IntPtr offset, IntPtr size, IntPtr data);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLCompileShader(uint shader);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate uint GLCreateProgram();
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate uint GLCreateShader(uint type);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLDeleteBuffers(int n, ref uint buffers);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLDeleteProgram(uint program);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLDeleteShader(uint shader);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLDeleteVertexArrays(int n, ref uint arrays);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLDisableVertexAttribArray(uint index);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLDrawArraysInstanced(uint mode, int first, int count, int instancecount);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLEnableVertexAttribArray(uint index);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLGenBuffers(int n, out uint buffers);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLGenVertexArrays(int n, out uint arrays);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLGetProgramInfoLog(uint program, int maxLength, out int length, System.Text.StringBuilder infoLog);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLGetProgramiv(uint program, uint pname, out int result);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLGetShaderInfoLog(uint shader, int maxLength, out int length, System.Text.StringBuilder infoLog);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLGetShaderiv(uint shader, uint pname, out int result);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate int GLGetUniformLocation(uint program, string name);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLLinkProgram(uint program);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLShaderSource(uint shader, int count, string[] text, int[] length);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLTexBuffer(uint target, uint internalformat, uint buffer);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLUniform1f(int location, float v0);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLUniform1i(int location, int v0);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLUniform2f(int location, float v0, float v1);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLUniform3f(int location, float v0, float v1, float v2);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLUniform4f(int location, float v0, float v1, float v2, float v3);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLUniform4fv(int location, int count, float[] value);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLUniformMatrix4fv(int location, int count, bool transpose, float[] value);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLUseProgram(uint program);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLVertexAttribDivisor(uint index, uint divisor);
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate void GLVertexAttribPointer(uint index, int size, uint type, bool normalized, int stride, IntPtr pointer);
    }
}
