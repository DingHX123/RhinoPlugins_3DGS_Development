# RhinoPlugins_Development

Rhino 8 插件，用于在 Rhino 中导入、查看、编辑 3D Gaussian Splatting（3DGS）模型，并支持从高斯数据提取几何、构建超二次曲面等建模工作。

- 目标框架：`net7.0-windows` / `net48`（Rhino 8）
- 输出：`bin\debug\RhinoPlugins_Development.rhp`
- 命令由 RhinoCommon 自动注册，加载插件后在命令行直接键入命令名即可

---

## 命令总览

### 一、导入与导出

| 命令 | 作用 |
|---|---|
| `ImportGaussianSplat` | 导入 3DGS 模型（`.ply` / `.splat` / `.ksplat`），生成代理点云并启用 GPU 实时渲染。导入时可开关：上轴扶正（Z-up）、自动剔除 floaters（AutoCrop）、自动找平地面（LevelFloor） |
| `ReLoadGaussianSplat` | 手动重载文档中已保存但未成功挂接渲染的 splat 代理点云（用于插件晚于文档加载、`EndOpenDocument` 未触发的场景） |
| `ExportGaussianSplat` | 将当前激活的 splat 导出为 `.ply` / `.splat` / `.ksplat`，支持多实例合并导出 |
| `ClearGaussianSplat` | 清除已加载的 splat，可选择单个实例、选定实例或全部实例 |
| `ConvertPointCloudToSplat` | 把选中的 Rhino 点云转换为高斯泼溅模型并启用实时渲染，自动估计每个 splat 的尺度 |

### 二、查看与显示

| 命令 | 作用 |
|---|---|
| `Get3dgsPointEllipsoid` | 点击点云上的单个点，弹出属性窗口展示该高斯的全部参数：位置、三轴尺度、旋转四元数、各阶球谐系数（颜色）、不透明度等 |
| `SplatSettings` | 调整全局渲染参数：整体缩放倍率与低通滤波半径 |

### 三、编辑与筛选

| 命令 | 作用 |
|---|---|
| `EraseSplatWithBrush` | 球形笔刷擦除：在视口中涂抹删除范围内的 splat，可连续操作，结束后统一注册一次撤销 |
| `CleanSplatNoise` | 统计离群点去除：基于体素网格哈希 + 27 邻域邻居计数，删除孤立 splat（飘浮噪点） |
| `CropGaussianSplat` | 用封闭几何体（Box / 封闭 Brep）裁剪 splat，可选择保留内部或外部 |
| `FilterGaussianSplat` | 递归过滤：从种子 splat 出发，按"搜索放大倍率"做相交扩散（BFS），保留颜色变化阈值内且透明度达标的椭球，仅显示过滤结果 |
| `Create3dgsellipsoid` | 为 splat 创建椭球实体（Brep）。三种选取方式：单点（最近点）、笔刷涂抹、视口可见点；椭球继承该高斯的颜色与透明度 |
| `CreateSuperquadric` | 超二次曲面参数面板（见下节） |

### 四、几何提取与拟合

| 命令 | 作用 |
|---|---|
| `ExtractMeshFromSplat` | 从 splat 数据提取网格：Surface Nets 等值面提取，后台线程执行并显示进度条 |
| `SectionSplat` | 剖切：① 选取已有平面或三点定义平面；② 正交取样（X/Y/Z 各方向自动均布剖面）。以解析计算平面与每个 splat 椭球的相交椭圆，截面线颜色与椭球渲染颜色一致 |
| `FitPlaneToSplat` | 在指定边界区域内对 splat 中心点做 RANSAC 基元拟合，支持平面、圆柱、球 |
| `EllipseOuterHull` | 选取若干封闭椭圆曲线，生成其外接曲线。三种模式：`Auto`（自动比较真矩形与椭圆谁覆盖更紧）、`Polygon`（收缩拟合为真矩形）、`Ellipse`（收缩拟合为单个椭圆） |

---

## CreateSuperquadric 详细说明

### 数学模型

Barr 超二次曲面隐式方程：

```
((|x/a|^(2/ε2) + |y/b|^(2/ε2))^(ε2/ε1) + |z/c|^(2/ε1))^(ε1/2) = 1
```

- `a / b / c`：三个主轴半轴长度
- `ε1`（纵向指数）：→0 趋近盒状，=1 为椭球，→2 趋近八面体
- `ε2`（横向指数）：控制水平截面的饱满程度，可与 `ε1` 独立设置

### 交互

命令打开一个非模态参数窗口，**拖动滑杆时 Rhino 视口实时显示半透明预览**，确认后点「创建」才真正写入文档。

| 参数 | 滑杆范围 | 说明 |
|---|---|---|
| 半轴 a / b / c | 0.01 ~ 10 | 连续调节；右侧数值框可输入超出滑杆范围的精确值 |
| ε1 / ε2 | 0.05 ~ 4 | 形状指数，可先用预设快速切换 |
| 网格分辨率 U / V | 8 ~ 256 | 预览自动限制为 128×64 保证流畅，创建时使用设定值 |
| 中心 X / Y / Z | -10 ~ 10 | 世界坐标偏移 |
| 旋转 X / Y / Z | -180 ~ 180° | 依次绕世界 X→Y→Z 轴旋转 |

预设：椭球（1,1）、长方体（0.15,0.15）、八面体（2,2）、枕形（0.5,0.5）。

### 输出类型

| 类型 | 实现 | 适用 |
|---|---|---|
| **Mesh 网格** | 参数化生成封闭网格（南极点 + 纬线环 + 北极点，右手法则保证法线朝外） | 布尔运算、3D 打印、后续网格处理 |
| **NURBS 曲面** | 控制点网格构造 3 阶非有理 NURBS（极点行重合，退化时自动改用极小极环） | 需要拖拽控制点编辑的光滑曲面工作流 |

### 布尔运算按钮

对选中的多个封闭实体做并集 / 交集 / 差集：

| 按钮 | 交互 | 运算 |
|---|---|---|
| 交集 | 一次多选（≥2，回车完成） | A ∩ B ∩ C… |
| 并集 | 一次多选（≥2，回车完成） | A ∪ B ∪ C… |
| 差集 | 先选**被剪模型**（主体 A），再选**裁剪模型**（B、C…） | A − B − C… |

- 全 Mesh → 网格布尔；全 Brep → B-rep 布尔（用文档绝对容差）；混合类型 → Brep 网格化后走网格布尔
- 结果**继承主体对象的图层、材质、颜色**
- 未封闭的输入对象会被跳过并在命令行提示数量
- 整个操作注册为一次撤销，`Ctrl+Z` 可回滚

### 修复封闭按钮

对选中的网格 / 曲面执行三级修复流程，把开放模型转为封闭实体：

1. **自适应容差焊点**：容差 = `max(文档容差, 包围盒对角线 × 0.001)`，用空间哈希合并距离过近的顶点并重建拓扑（解决布尔缝隙大于合并容差、`FillHoles` 识别不出边界环的问题）
2. **常规补洞**：剔除退化面 → `FillHoles` 最多 4 轮 → 统一法线 → 压缩
3. **Brep 中转兜底**：网格 → Brep → `JoinNakedEdges` 缝合裸边 → 成为 solid 后转回网格（solid Brep 转网格必定封闭）

布尔运算产生的结果会自动走同一套修复流程；无法修复时会明确报告 `N 个仍非封闭`。

---

## 典型工作流

**扫描 → 编辑 → 建模**

```
ImportGaussianSplat        导入 3DGS 模型，实时查看
CleanSplatNoise            去除飘浮噪点
CropGaussianSplat          用 Box 裁掉多余环境
ExtractMeshFromSplat       提取等值面网格
SectionSplat               生成剖面线用于制图
```

**从高斯数据构建设计体块**

```
ImportGaussianSplat        导入模型作为参照
CreateSuperquadric         用超二次曲面逐块拼装主体
   └ 差集 / 并集 / 交集     组合成目标形体
   └ 修复封闭              保证结果可直接打印或继续布尔
```

**点云转高斯**

```
ConvertPointCloudToSplat   把已有 Rhino 点云转成 splat 渲染
SplatSettings              调尺度与滤波让显示更清晰
```

---

## 主要源文件

| 文件 | 内容 |
|---|---|
| `RhinoPlugins_DevelopmentPlugin.cs` | 插件入口：文档事件挂接、加载时恢复 splat 实例 |
| `SplatDisplayConduit.cs` | splat 渲染管线（OpenGL 绘制）、实例管理与文档恢复 |
| `SplatLoader.cs` / `SpzLoader.cs` / `LasLoader.cs` | 各格式解析 |
| `Splat.cs` / `SplatInstance.cs` | splat 数据结构与实例封装 |
| `CreateSuperquadricCommand.cs` | 超二次曲面网格生成器 + NURBS 构造器 |
| `SuperquadricWindow.xaml(.cs)` | 参数面板、实时预览 conduit、布尔运算与修复逻辑 |
