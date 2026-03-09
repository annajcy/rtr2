# Slang Basics

`Slang Basics` 是 `Slang Practice` 板块中的第一个 chapter，目标是让你先把最小可运行的 Slang compute shader 看懂、改动并解释清楚。

这个 chapter 暂时只包含一节，但结构已经按系列化内容预留好了，后续可以继续追加。

## Chapter 目标

学完这个 chapter 后，你应该能做到：

- 看懂 RTR2 当前 `editable_shadertoy` 示例的 shader 输入输出结构
- 理解 `ConstantBuffer`、`RWTexture2D`、`SV_DispatchThreadID`、`numthreads`
- 知道一个 compute shader 如何从线程坐标走到最终图像写入
- 能独立完成一组小型 shader 改写练习

## 当前小节

### 第 1 节：Editable ShaderToy 与 Compute 最小闭环

这一节聚焦：

- Slang 最小语法
- compute shader 的线程模型
- `tid -> uv -> p -> outColor` 这条核心数据流
- 今日 5 个微练习

阅读入口：

- [第 1 节：Editable ShaderToy 与 Compute 最小闭环](slang-compute-basics.md)

返回板块首页：

- [Slang Practice](slang-practice.md)
