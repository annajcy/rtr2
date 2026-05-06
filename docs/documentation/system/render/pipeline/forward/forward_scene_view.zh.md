# Forward Scene View

`src/rtr/system/render/pipeline/forward/forward_scene_view.hpp` 定义了 forward 渲染器消费的 CPU 侧场景视图数据模型。

它包含：

- `MeshView`：GPU buffer handle 和 index 数量
- `ForwardSceneCameraData`：view/proj 矩阵和 camera 世界坐标
- `ForwardScenePointLight`：紧凑的逐灯光数据
- `ForwardSceneRenderable`：一个可绘制 mesh 实例及其变换和基础颜色
- `ForwardSceneView`：forward 路径完整消费的场景载荷

这个文件是 scene 提取和 GPU 录制之间的边界：`framework/integration/render/` 下的 builder 产出它，`ForwardPipeline` 消费它。
