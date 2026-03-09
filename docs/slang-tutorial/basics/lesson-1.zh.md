# Slang Basics · 第 1 节：Editable ShaderToy 与 Compute 最小闭环

本文档是 `Slang Practice` 板块下 `Slang Basics` chapter 的第 1 节，基于 RTR2 里的 `editable_shadertoy` 示例，帮助你在一天内建立 Slang 基本语法和 compute shader 计算模型的最小认知闭环。

这不是一份完整 Slang 手册。今天只关注一件事：你能看懂并修改一个可热重载的 compute shader，并且知道它为什么这样执行。

相关导航：

- [Slang Practice](../overview.md)
- [Slang Basics](overview.md)

## 今日目标

完成今天的学习后，你应该能回答下面几个问题：

- 当前样例改哪个文件？
- `SV_DispatchThreadID` 是什么？
- `[numthreads(8, 8, 1)]` 表示什么？
- `ConstantBuffer` 和 `RWTexture2D` 在这个例子里各负责什么？
- 今天应该完成哪些练习？

## 环境入口

今天只围绕这 3 个入口文件：

- `examples/editor/editable_shadertoy_editor.cpp`：示例程序入口
- `src/rtr/editor/panel/editable_shadertoy_settings_panel.hpp`：热重载状态和参数滑条 UI
- `shaders/editable_shadertoy_compute.slang`：今天实际要修改的 shader 文件

推荐运行命令：

```bash
cmake --build --preset conan-debug --target editable_shadertoy_editor
./build/Debug/examples/editable_shadertoy_editor
```

如果你的 `build/Debug` 已经存在，也可以直接运行：

```bash
./build/Debug/examples/editable_shadertoy_editor
```

运行后重点观察 `Editable ShaderToy Settings` 面板：

- `Shader Source`：当前热重载的 Slang 文件路径
- `Auto Reload`：保存文件后自动重编译
- `Reload`：手动触发重编译
- `Param 0 ~ Param 3`：CPU 侧传给 shader 的 4 个浮点参数
- `Latest Error`：最近一次编译错误

## Slang 最小语法

今天只学习当前样例会用到的最小子集。

### 标量和向量类型

Slang 的基本数值类型和 C/C++ 接近，但为图形计算提供了向量版本：

- `float`：单个浮点数
- `float2`：2 维向量，常用于 `uv` 坐标
- `float3`：3 维向量，常用于 RGB 颜色
- `float4`：4 维向量，常用于 RGBA、齐次坐标或一组打包参数
- `uint3`：3 维无符号整型向量，当前样例用它表示线程 ID

示例：

```slang
float2 uv = ((float2)tid.xy + 0.5) / params.iResolution.xy;
float3 color = float3(1.0, 0.0, 0.0);
```

### `struct`

和 C/C++ 一样，`struct` 用来组织数据。当前样例把分辨率、时间和自定义参数打包成一个 uniform 结构：

```slang
struct ShaderToyParams {
    float4 iResolution;
    float4 iTime;
    float4 iParams;
};
```

这里不是为了“面向对象”，而是为了让 CPU 能按固定布局把一块数据传给 GPU。

### 函数

Slang 支持普通函数。你可以把调色、形状、动画拆成小函数，避免把所有逻辑堆进入口函数。

```slang
float3 palette(float t) {
    float3 a = float3(0.45, 0.35, 0.20);
    float3 b = float3(0.55, 0.40, 0.25);
    float3 c = float3(0.90, 0.70, 0.35);
    float3 d = float3(0.15, 0.10, 0.05);
    return a + b * cos(6.28318 * (c * t + d));
}
```

今天你要习惯一件事：shader 里很多“函数”本质上是在描述每个线程如何计算一个像素。

### 资源声明

当前样例只用两个资源：

```slang
[[vk_binding(0, 0)]]
ConstantBuffer<ShaderToyParams> params;

[[vk_binding(1, 0)]]
RWTexture2D<float4> outColor;
```

- `ConstantBuffer<ShaderToyParams>`：只读 uniform 数据。这里承载分辨率、时间和 4 个参数。
- `RWTexture2D<float4>`：可读写二维纹理。当前例子只往里面写颜色，相当于 compute shader 的输出画布。

`[[vk_binding(set, binding)]]` 用来告诉 Vulkan 这个资源绑定在哪个槽位上。今天不要改这些 binding，先把计算模型看懂。

### Attribute 和入口函数

当前样例的入口是：

```slang
[numthreads(8, 8, 1)]
[shader("compute")]
void compMain(uint3 tid : SV_DispatchThreadID) {
    ...
}
```

- `[shader("compute")]`：声明这是一个 compute shader 入口
- `[numthreads(8, 8, 1)]`：声明一个线程组里有 `8 x 8 x 1` 个线程
- `SV_DispatchThreadID`：当前线程在整个 dispatch 空间中的全局坐标

这个仓库的热重载编译逻辑要求：一个 Slang 文件里必须有且只有一个 compute entry point。

## Compute Shader 计算模型

### 先建立一个正确心智模型

不要把 compute shader 理解成“CPU 调一次函数”。更接近真实情况的理解是：

1. CPU 发起一次 dispatch。
2. GPU 创建很多线程。
3. 每个线程都执行同一份 `compMain` 程序。
4. 每个线程根据自己的 `tid` 计算不同的输出位置和颜色。

在这个样例里，你可以把它粗略理解成“一个线程负责一个像素”。

### `numthreads` 是线程组大小，不是总线程数

```slang
[numthreads(8, 8, 1)]
```

这表示一个 thread group 中有 64 个线程，而不是整张图只有 64 个线程。

真正总共会执行多少线程，取决于 CPU 侧 dispatch 的维度。你今天不需要先追 dispatch 细节，只要先抓住一点：

- `tid` 是全局线程坐标
- 你看到的每个像素颜色，都是某个线程算出来并写进 `outColor[tid.xy]`

### 为什么要做边界判断

```slang
uint width = (uint)params.iResolution.x;
uint height = (uint)params.iResolution.y;
if (tid.x >= width || tid.y >= height) {
    return;
}
```

线程组通常按整块调度，但图像宽高不一定正好是线程组大小的整数倍，所以最右侧和最下侧可能有一些“多出来的线程”。这些线程如果继续写纹理，就会越界。

这段判断的目的就是：

- 防止访问超出纹理边界
- 把多余线程提前返回

### 从线程坐标到屏幕坐标

当前样例最重要的一条数据流是：

```slang
float2 uv = ((float2)tid.xy + 0.5) / params.iResolution.xy;
float2 p = uv * 2.0 - 1.0;
p.x *= params.iResolution.x / max(params.iResolution.y, 1.0);
```

可以按这三个阶段理解：

- `tid.xy`：像素索引，范围大致是 `[0, width)` 和 `[0, height)`
- `uv`：归一化到 `[0, 1]` 的屏幕坐标
- `p`：再映射到 `[-1, 1]` 的中心化坐标，便于画圆、环、波纹等形状

`+ 0.5` 的作用是取像素中心而不是左上角。

最后一行是宽高比修正。如果不修正，圆往往会被拉伸成椭圆。

### 输出是什么

当前线程算完颜色后，会写入自己负责的位置：

```slang
outColor[tid.xy] = float4(color, 1.0);
```

这就是 compute shader 今天最核心的 I/O 模型：

- 输入：线程 ID、uniform 参数
- 输出：写入一张二维图像

## 对照当前 shader 逐段阅读

你可以把 `shaders/editable_shadertoy_compute.slang` 分成 5 段来看。

### 1. 参数结构

```slang
struct ShaderToyParams {
    float4 iResolution;
    float4 iTime;
    float4 iParams;
};
```

- `iResolution.xy`：当前渲染目标分辨率
- `iTime.x`：当前时间
- `iParams.xyzw`：来自编辑器滑条的 4 个参数

### 2. 资源绑定

```slang
[[vk_binding(0, 0)]]
ConstantBuffer<ShaderToyParams> params;

[[vk_binding(1, 0)]]
RWTexture2D<float4> outColor;
```

今天先把它记成：

- `params` 是“读配置”
- `outColor` 是“写结果”

### 3. 辅助函数 `palette`

这是一个纯数学调色函数。给它一个标量 `t`，它返回一个颜色。今天你不需要完全推导公式，只需要理解：

- 输入是某个连续变化的值
- 输出是可随时间和位置变化的颜色

### 4. 入口函数前半段

```slang
uint width = (uint)params.iResolution.x;
uint height = (uint)params.iResolution.y;
...
float2 uv = ((float2)tid.xy + 0.5) / params.iResolution.xy;
float2 p = uv * 2.0 - 1.0;
```

这一段的任务是建立像素对应的坐标系。

### 5. 入口函数后半段

```slang
float time = params.iTime.x * (0.5 + params.iParams.y);
float radius = 1 + 0.25 * sin(time * 0.7 + params.iParams.z);
float ripple = sin(18.0 * length(p) - time * (1.5 + params.iParams.x)) * 0.5 + 0.5;
float ring = smoothstep(radius + 0.04, radius - 0.04, abs(length(p) - radius));

float glow = exp(-4.5 * length(p)) * (0.35 + 0.65 * ripple);
float hue = ripple + ring * 0.5 + uv.x * 0.2;
float3 color = palette(hue) * (glow + ring);
```

这一段是纯视觉公式。你今天不需要发明新公式，只要学会拆开看：

- 哪些变量来自位置：`uv`、`p`
- 哪些变量来自时间：`time`
- 哪些变量来自 UI：`iParams`
- 哪些变量是中间视觉量：`ripple`、`ring`、`glow`
- 最终如何合成：`color -> outColor`

## 今天该做的练习

今天只做 5 个微练习，全部在 `shaders/editable_shadertoy_compute.slang` 内完成。

### 练习 1：改成纯渐变

目标：熟悉 `tid -> uv -> color -> outColor`

建议结果：

- `uv.x` 控制红色
- `uv.y` 控制蓝色
- 绿色固定或简单插值

完成标准：你能不依赖现成公式，自己写出一个渐变背景。

### 练习 2：让画面随时间闪烁

目标：熟悉 `params.iTime.x`

建议做法：

- 在颜色前乘一个 `pulse`
- `pulse = sin(time) * 0.5 + 0.5`

完成标准：你知道时间不是“全局魔法变量”，而是 uniform 数据的一部分。

### 练习 3：画一个居中的圆

目标：熟悉中心化坐标 `p` 和距离函数 `length(p)`

建议做法：

- 先把背景设成黑色
- 当 `length(p) < radius` 时输出亮色

完成标准：你能解释为什么 `p` 比 `uv` 更适合画几何图形。

### 练习 4：把圆改成呼吸圆环

目标：熟悉动画半径和软边缘

建议做法：

- 用 `sin(time)` 改变半径
- 用 `smoothstep` 做边缘过渡

完成标准：你能做出“不是硬边”的圆环。

### 练习 5：让 4 个参数都变得有意义

把 UI 中的参数约定为：

- `Param 0 = scale`
- `Param 1 = speed`
- `Param 2 = radius`
- `Param 3 = edge softness`

完成标准：

- 4 个参数都参与公式
- 拖动滑条时你能准确说出每个参数控制的视觉含义

## 常见错误

### 1. 语法错误

最常见的是漏分号、括号不匹配、函数返回值不对。这个仓库会把最近一次编译错误显示在 `Latest Error` 面板中。

### 2. 没有 compute entry point

如果你删掉了 `[shader("compute")]` 或者把入口函数改坏，热重载会失败，因为当前编译器要求找到一个 compute entry point。

### 3. 有多个 compute entry point

同一个 `.slang` 文件里不要同时保留两个 compute 入口，否则也会被拒绝。

### 4. 越界访问

如果你删掉边界判断，`tid.xy` 可能超出纹理范围，结果可能是错误图像、验证层报错或未定义行为。

### 5. 画面全黑

优先检查这几类问题：

- `color` 是否始终为 0
- `radius` 或阈值是否过小
- 是否没有写 `outColor[tid.xy]`
- 是否把时间或参数乘成了极端值

## 今日验收标准

今天结束前，确认你已经完成这 6 件事：

1. 成功运行 `editable_shadertoy_editor`
2. 修改 `.slang` 文件后能触发热重载
3. 故意制造一个语法错误后，能在面板里看到报错并修复
4. 能把当前效果改成纯渐变
5. 能做出一个圆或圆环
6. 能解释 `tid -> uv -> p -> outColor` 这条链路

如果这 6 项都达成，今天的入门目标就完成了。

## 今日复盘模板

最后花 10 到 15 分钟，自己写一段简短复盘，至少回答下面 5 个问题：

- Slang 和 C/C++ 最像的地方是什么？
- Slang 和 C/C++ 最不同的地方是什么？
- 为什么说 compute shader 是“很多线程执行同一份程序”？
- 当前样例的输入有哪些？
- 当前样例的输出是什么？

如果你能用自己的话把这些问题讲清楚，后续再去学纹理采样、多 pass 或 shared memory 才会更稳。
