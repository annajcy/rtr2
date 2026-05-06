# Render System 总览

`src/rtr/system/render/` 是引擎当前的实时 GPU 渲染主干。

这棵目录按职责分成几层：

- 根目录文件：帧调度、pipeline 协议、output backend 抽象和每帧状态
- `pass/`：可复用的输出型 render pass，例如 present pass
- `pipeline/forward/`：forward 图形场景渲染路径
- `pipeline/shadertoy/`：基于 compute 的全屏管线与热重载支持
- `utils/`：shader 编译辅助

当前实现的关键结构拆分是：

- pipeline 负责把内容渲染到最终 offscreen image
- output backend 决定这个 image 如何被消费
- renderer 负责 realtime / preview 路径的 window、context、device、swapchain 引导

更具体的类型、生命周期和耦合点见下面各文件页面。
