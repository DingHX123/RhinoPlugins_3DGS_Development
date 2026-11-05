using Rhino.Commands;
using Rhino.Input.Custom;
using Rhino.Input;
using Rhino;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.DocObjects;
using System.IO;
using Rhino.Geometry;
using System.Windows.Interop;

namespace RhinoPlugins_Development
{
    public class Get3dgsPointEllipsoid : Command
    {
        public override string EnglishName => "Get3dgsPointEllipsoid";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            // 1. 选择点云中的单个顶点
            PointCloud pointCloud = null;
            ObjRef cloudObjRef = null;

            // 优先使用用户提前选中的点云
            var selectedObjects = doc.Objects.GetSelectedObjects(false, false);
            foreach (var rhinoObj in selectedObjects)
            {
                if (rhinoObj.ObjectType == ObjectType.PointSet)
                {
                    pointCloud = rhinoObj.Geometry as PointCloud;
                    cloudObjRef = new ObjRef(rhinoObj.Id);
                    break;
                }
            }

            // 没有预选中则让用户手动选择
            if (pointCloud == null)
            {
                var go = new GetObject();
                go.SetCommandPrompt("请选择一个点云对象");
                go.GeometryFilter = ObjectType.PointSet;
                if (go.Get() != GetResult.Object)
                    return Result.Cancel;

                cloudObjRef = go.Object(0);
                pointCloud = cloudObjRef.PointCloud();
            }

            if (pointCloud == null || pointCloud.Count == 0)
            {
                RhinoApp.WriteLine("未获取到有效的点云");
                return Result.Failure;
            }

            // ========== 2. 获取用户点击的视口位置 ==========
            var gpo = new GetPoint();
            gpo.SetCommandPrompt("点击目标位置，自动查找最近顶点");
            GetResult pointResult = gpo.Get();
            if (pointResult != GetResult.Point)
                return Result.Cancel;

            Point3d clickPosition = gpo.Point();

            // ========== 3. 查找最近的顶点 ==========
            // 调用内置高效方法查找最近点索引
            int pointIndex = pointCloud.ClosestPoint(clickPosition);

            if (pointIndex < 0)
            {
                RhinoApp.WriteLine("未找到匹配的顶点");
                return Result.Failure;
            }

            // ========== 4. 取出选中点的椭球参数 ==========
            // 优先使用 ImportGaussianSplat 渲染管线中的 splat 数据（与光栅渲染的椭球严格一致）
            Point3d rhinoPos = pointCloud.PointAt(pointIndex);
            GaussianPoint gp = null;
            string sourceLabel = null;
            bool useRhinoCenter = false;

            Guid cloudId = cloudObjRef != null ? cloudObjRef.ObjectId : Guid.Empty;
            SplatInstance renderInstance = null;
            if (SplatDisplayConduit.Instance != null && cloudId != Guid.Empty)
            {
                renderInstance = SplatDisplayConduit.Instance.Instances.Find(x => x.ProxyObjectId == cloudId);
            }

            if (renderInstance != null && renderInstance.Splats != null && renderInstance.Splats.Length > 0)
            {
                // 代理点云是按固定步长抽样的：点云第 k 点对应 splat 索引 k * step
                int sampleStep = Math.Max(1, (int)Math.Ceiling((double)renderInstance.Splats.Length / Math.Max(1, pointCloud.Count)));
                int splatIdx = pointIndex * sampleStep;
                if (splatIdx < 0 || splatIdx >= renderInstance.Splats.Length)
                {
                    RhinoApp.WriteLine("点索引超出渲染数据范围");
                    return Result.Failure;
                }

                gp = Create3dgsellipsoid.SplatToGaussianPoint(renderInstance.Splats[splatIdx]);
                sourceLabel = $"渲染管线 splat #{splatIdx} (代理点 #{pointIndex})";
            }
            else
            {
                // 回退：点云不是当前渲染的代理点云，只按需解析 PLY 中被选中的那一个顶点
                // （随机访问单顶点，避免整文件解析导致的长等待）
                string plyPath = @"E:\Users\Windows11\Desktop\models\kitchen\point_cloud\iteration_30000\point_cloud.ply";
                if (!File.Exists(plyPath))
                {
                    RhinoApp.WriteLine("点云不是当前渲染的代理点云，且 PLY 文件不存在，请修改代码中的路径");
                    return Result.Failure;
                }

                gp = Parse3dgsPlyPoint(plyPath, pointIndex);
                if (gp == null)
                {
                    RhinoApp.WriteLine("点索引超出范围");
                    return Result.Failure;
                }

                useRhinoCenter = rhinoPos.DistanceTo(new Point3d(gp.X, gp.Y, gp.Z)) > 0.01;
                sourceLabel = plyPath;
            }

            // 5. 使用 WPF 窗口展示所有属性
            var window = new GaussianPointPropertiesWindow(pointIndex, gp, sourceLabel, rhinoPos, useRhinoCenter);
            var helper = new WindowInteropHelper(window);
            helper.Owner = RhinoApp.MainWindowHandle();
            window.ShowDialog();

            return Result.Success;
        }

        /// <summary>
        /// 只解析二进制 3DGS PLY 中指定索引的单个顶点（随机访问）：
        /// 先解析头部得到属性数，按 属性数×4 字节计算顶点步长后直接 Seek 到目标顶点，
        /// 避免为取一个点而解析整个文件。索引无效或头部不完整返回 null。
        /// </summary>
        internal static GaussianPoint Parse3dgsPlyPoint(string filePath, int pointIndex)
        {
            int vertexCount;
            List<string> propertyNames;
            long dataStartOffset;
            if (!ParsePlyHeader(filePath, out vertexCount, out propertyNames, out dataStartOffset))
                return null;

            if (pointIndex < 0 || pointIndex >= vertexCount || propertyNames.Count == 0)
                return null;

            var gp = new GaussianPoint();

            using (var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read))
            using (var reader = new BinaryReader(fs, Encoding.ASCII))
            {
                // 定位到目标顶点：数据起始偏移 + 索引 × 顶点步长（每属性 4 字节 float）
                fs.Position = dataStartOffset + (long)pointIndex * (propertyNames.Count * 4L);

                for (int j = 0; j < propertyNames.Count; j++)
                {
                    string propName = propertyNames[j];
                    float rawValue = reader.ReadSingle();
                    gp.AllProperties[propName] = rawValue;

                    switch (propName)
                    {
                        case "x":
                            gp.X = rawValue;
                            break;
                        case "y":
                            gp.Y = rawValue;
                            break;
                        case "z":
                            gp.Z = rawValue;
                            break;
                        case "scale_0":
                            gp.ScaleX = Math.Exp(rawValue);
                            break;
                        case "scale_1":
                            gp.ScaleY = Math.Exp(rawValue);
                            break;
                        case "scale_2":
                            gp.ScaleZ = Math.Exp(rawValue);
                            break;
                        case "rot_0":
                            gp.RotW = rawValue;
                            break;
                        case "rot_1":
                            gp.RotX = rawValue;
                            break;
                        case "rot_2":
                            gp.RotY = rawValue;
                            break;
                        case "rot_3":
                            gp.RotZ = rawValue;
                            break;
                    }
                }
            }

            return gp;
        }

        /// <summary>
        /// 解析二进制 PLY 头部，返回顶点数、属性名列表与二进制数据起始字节偏移。
        /// 头部完整（读到 end_header）返回 true。
        /// </summary>
        private static bool ParsePlyHeader(string filePath, out int vertexCount, out List<string> propertyNames, out long dataStartOffset)
        {
            vertexCount = 0;
            propertyNames = new List<string>();
            dataStartOffset = 0;

            using (var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read))
            using (var reader = new BinaryReader(fs, Encoding.ASCII))
            {
                string line;
                while ((line = ReadLine(reader)) != null)
                {
                    line = line.Trim();
                    if (line == "end_header")
                    {
                        // 记录数据部分的起始字节位置
                        dataStartOffset = fs.Position;
                        return true;
                    }
                    else if (line.StartsWith("element vertex"))
                    {
                        string[] parts = line.Split(' ');
                        vertexCount = int.Parse(parts[2]);
                    }
                    else if (line.StartsWith("property"))
                    {
                        string[] parts = line.Split(' ');
                        // parts[1] 是数据类型，parts[2] 是属性名
                        propertyNames.Add(parts[2]);
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// 解析二进制格式的3DGS PLY文件，返回所有高斯点的椭球参数
        /// 支持 format binary_little_endian 1.0
        /// </summary>
        internal static List<GaussianPoint> Parse3dgsPlyBinary(string filePath)
        {
            var points = new List<GaussianPoint>();
            int vertexCount;
            List<string> propertyNames;
            long dataStartOffset;
            ParsePlyHeader(filePath, out vertexCount, out propertyNames, out dataStartOffset);

            using (var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read))
            using (var reader = new BinaryReader(fs, Encoding.ASCII))
            {
                // 定位到数据起始位置
                fs.Position = dataStartOffset;

                // 逐个解析二进制顶点
                for (int i = 0; i < vertexCount; i++)
                {
                    var gp = new GaussianPoint();

                    for (int j = 0; j < propertyNames.Count; j++)
                    {
                        string propName = propertyNames[j];
                        float rawValue = reader.ReadSingle();
                        gp.AllProperties[propName] = rawValue;

                        switch (propName)
                        {
                            case "x":
                                gp.X = rawValue;
                                break;
                            case "y":
                                gp.Y = rawValue;
                                break;
                            case "z":
                                gp.Z = rawValue;
                                break;
                            case "scale_0":
                                gp.ScaleX = Math.Exp(rawValue);
                                break;
                            case "scale_1":
                                gp.ScaleY = Math.Exp(rawValue);
                                break;
                            case "scale_2":
                                gp.ScaleZ = Math.Exp(rawValue);
                                break;
                            case "rot_0":
                                gp.RotW = rawValue;
                                break;
                            case "rot_1":
                                gp.RotX = rawValue;
                                break;
                            case "rot_2":
                                gp.RotY = rawValue;
                                break;
                            case "rot_3":
                                gp.RotZ = rawValue;
                                break;
                        }
                    }

                    points.Add(gp);
                }
            }

            return points;
        }

        /// <summary>
        /// 从BinaryReader中读取一行ASCII文本
        /// </summary>
        internal static string ReadLine(BinaryReader reader)
        {
            var sb = new StringBuilder();
            char c;
            while (true)
            {
                if (reader.BaseStream.Position >= reader.BaseStream.Length)
                    return sb.Length > 0 ? sb.ToString() : null;

                c = reader.ReadChar();
                if (c == '\n')
                    break;
                if (c != '\r')
                    sb.Append(c);
            }
            return sb.ToString();
        }
    }

    /// <summary>
    /// 3DGS高斯点数据结构
    /// </summary>
    public class GaussianPoint
    {
        public double X, Y, Z;
        public double ScaleX, ScaleY, ScaleZ;
        public double RotW, RotX, RotY, RotZ;
        public Dictionary<string, double> AllProperties { get; set; } = new Dictionary<string, double>();
    }
}