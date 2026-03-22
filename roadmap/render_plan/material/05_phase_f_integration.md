# Phase F: 集成与验证（~1 天）

## 目标

将所有材质组件端到端连通，验证 microfacet PBR 在各种条件下的正确性。

## 验证场景

### 1. 材质球阵列（Material Sphere Grid）

标准 PBR 验证场景：渲染一个 N×N 球体网格，每行递增 roughness，每列递增 metallic。

```
roughness →  0.0    0.25    0.5    0.75    1.0
metallic
  0.0        ●       ●       ●       ●       ●
  0.25       ●       ●       ●       ●       ●
  0.5        ●       ●       ●       ●       ●
  0.75       ●       ●       ●       ●       ●
  1.0        ●       ●       ●       ●       ●
```

**期望结果**：
- 左上角（low roughness, non-metal）：白色锐利高光，灰色漫反射
- 右上角（high roughness, non-metal）：无可见高光，均匀灰色
- 左下角（low roughness, metal）：明亮镜面反射，有色高光
- 右下角（high roughness, metal）：暗色，无漫反射，金属色模糊高光

### 2. 纹理测试

- **Albedo 纹理**：checker pattern → 漫反射颜色正确
- **Normal 纹理**：brick normal map → 可见凹凸，光照方向影响阴影
- **Metallic-Roughness 纹理**：半金属半非金属 → 材质分界清晰
- **AO 纹理**：裂缝 AO → 缝隙变暗
- **Emissive 纹理**：发光图案 → 自发光区域不受光照影响

### 3. 参考对比

与已知正确的 PBR 渲染器对比：
- Khronos glTF Sample Viewer（web）
- Filament Sceneform
- Blender Eevee

加载 Khronos glTF sample models（`DamagedHelmet`、`FlightHelmet`、`SciFiHelmet`），
与参考渲染器截图逐像素对比。

### 4. 能量守恒测试

- 白炉测试：白色 IBL 环境 + 白色材质 → 输出亮度 ≤ 环境亮度
- 各 roughness 级别：总反射能量不应超过入射能量

### 5. 边界条件

- roughness = 0：纯镜面，高光极窄但不出 NaN / Inf
- roughness = 1：完全粗糙，高光消失
- metallic = 0, F0 = 0.04：电介质标准反射率
- metallic = 1：完全金属，无漫反射
- 掠射角（相机平视表面）：Fresnel 效应明显（边缘变白）
- 法线贴图反转（handedness 错误）：检查 bitangent sign

## 与现有系统的集成

### Forward Pipeline 兼容

现有 `ForwardPipeline` 需要最小修改即可使用新材质：

```cpp
// forward_pipeline.hpp 修改：
// 1. 创建 material descriptor layout
// 2. 分配 material descriptor sets
// 3. 绑定 material descriptor set (Set 1)
// 4. 使用新的 vert_buffer.slang（包含 microfacet BRDF）
```

### SRT Pipeline 对接

SRT pipeline（Phase 0 的 G-Buffer pass）直接使用材质系统：
- G-Buffer 写入的 albedo / metallic / roughness / normal 来自材质采样
- Deferred lighting 和 ReSTIR shade 读取 G-Buffer 中的材质参数
- SDFGI probe tracing 在命中点查询材质做漫反射评估

### 材质热重载（可选）

```cpp
// 在 ImGui 编辑器中实时调整材质参数
// 修改 GPU UBO 即可立即看到效果
void MaterialInstance::update_params(const PBRMaterialParams& new_params) {
    params_ = new_params;
    upload_ubo();  // 更新 GPU buffer
}
```

## 性能分析

### 带宽估算

```
每材质 GPU 开销：
  Material UBO:              64 bytes
  Albedo (1K):               1024² × 4 = 4 MB
  Normal (1K):               1024² × 4 = 4 MB
  MetallicRoughness (1K):    1024² × 4 = 4 MB
  AO (1K):                   1024² × 1 = 1 MB
  Emissive (1K):             1024² × 4 = 4 MB
  ─────────────────────────────────────
  Total per unique material: ~17 MB

100 种材质 → ~1.7 GB（需要纹理压缩或虚拟纹理）
初期 10-20 种材质完全可行
```

### Shader 开销对比

```
Blinn-Phong (旧):   ~10 ALU ops/light
Cook-Torrance (新):  ~40 ALU ops/light

增加 ~4× ALU，但：
- 现代 GPU ALU 极其充裕
- 瓶颈通常在带宽（纹理采样）而非计算
- Deferred pipeline 中每像素只评估一次 BRDF
```

## Phase F 完成标准

- [ ] 材质球阵列 5×5 渲染视觉正确
- [ ] 5 种纹理类型（albedo/normal/MR/AO/emissive）全部采样正确
- [ ] glTF 测试模型（DamagedHelmet）与参考渲染器视觉接近
- [ ] 白炉测试通过（能量守恒）
- [ ] 边界条件无 NaN / Inf / 黑色伪影
- [ ] 掠射角 Fresnel 效应可见
- [ ] 法线贴图 handedness 正确（bitangent sign）
- [ ] Forward pipeline 集成可运行
- [ ] G-Buffer 输出材质参数正确（为 SRT 准备）
- [ ] Alpha test 正确（半透明部分 discard）
