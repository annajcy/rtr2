# Phase A: 材质数据结构与加载（~2 天）

## 目标

定义 PBR 材质参数结构，建立纹理加载管线，从 .mtl / glTF 中提取材质信息。

## Day 0: PBR 材质定义

### Metallic-Roughness 工作流

选择 metallic-roughness 工作流（而非 specular-glossiness），理由：
- glTF 2.0 标准默认工作流
- Unreal / Unity / Godot 均采用
- 参数正交、直觉简单
- 与 SRT plan 中的 `PBRMaterial` 对齐

### 材质参数

1. **PBR Material** — `pbr_material.hpp`
   ```cpp
   struct PBRMaterialParams {
       // 基础参数（当无纹理时使用）
       glm::vec3 base_color{0.8f, 0.8f, 0.8f};
       float     metallic{0.0f};
       float     roughness{0.5f};
       float     ao{1.0f};             // ambient occlusion
       glm::vec3 emissive{0.0f};       // 自发光
       float     emissive_strength{1.0f};
       float     alpha_cutoff{0.5f};   // alpha test 阈值
       float     ior{1.5f};            // 折射率 → F0 = ((ior-1)/(ior+1))²

       // 纹理标志位（shader 中按需采样）
       uint32_t  texture_flags{0};
       // bit 0: has albedo map
       // bit 1: has normal map
       // bit 2: has metallic-roughness map
       // bit 3: has AO map
       // bit 4: has emissive map
   };
   ```

2. **纹理槽位定义**
   ```cpp
   enum class MaterialTextureSlot : uint32_t {
       Albedo            = 0,   // sRGB, RGB = base color, A = alpha
       Normal            = 1,   // Linear, RG = normal.xy (tangent space)
       MetallicRoughness = 2,   // Linear, B = metallic, G = roughness (glTF 约定)
       AO                = 3,   // Linear, R = ambient occlusion
       Emissive          = 4,   // sRGB, RGB = emissive color
       Count             = 5
   };
   ```

   > **MetallicRoughness 打包**：遵循 glTF 2.0 约定 — G 通道 = roughness，B 通道 = metallic。
   > 这样一张纹理同时提供两个参数，减少纹理采样和绑定数量。

3. **材质实例** — `material_instance.hpp`
   ```cpp
   class MaterialInstance {
       PBRMaterialParams params_;

       // 纹理句柄（可选，null = 使用 params_ 中的标量值）
       std::array<TextureHandle, 5> textures_;

       // 默认纹理（1×1 白 / 法线蓝 / 黑）
       // 无纹理时绑定默认纹理，避免 shader 分支
       static MaterialInstance create_default();
   };
   ```

### 为什么用默认纹理而非 shader 分支

```
方案 A: shader 中 if (has_albedo_map) { sample } else { use constant }
  → 分支发散，性能差，shader 复杂

方案 B: 无纹理时绑定 1×1 默认纹理
  → albedo 默认白色 → sample 结果 = (1,1,1,1) → 乘以 base_color 参数 = base_color
  → normal 默认 (0.5, 0.5, 1.0) → 解码后 = (0,0,1) → 即切线空间 up = 几何法线
  → metallic-roughness 默认 (0, roughness_param, metallic_param, 0) → 待定
  → shader 统一路径，无分支
```

选择**方案 B**。

4. **默认纹理** — `material_defaults.hpp`
   ```cpp
   struct MaterialDefaults {
       // 1×1 默认纹理
       rhi::Image white_texture;         // RGBA = (1,1,1,1)
       rhi::Image default_normal;        // RGBA = (0.5, 0.5, 1.0, 1.0) — 平坦法线
       rhi::Image black_texture;         // RGBA = (0,0,0,0)
       rhi::Image default_mr;            // RGBA = (0, 0.5, 0, 0) — roughness=0.5, metallic=0
       rhi::Sampler default_sampler;     // linear + aniso16 + repeat

       void init(/* device */);
   };
   ```

### GPU 端材质结构

```cpp
// 对应 shader 中的 MaterialUBO
struct alignas(16) GPUMaterialParams {
    glm::vec4 base_color_factor;    // xyz = color, w = alpha
    float     metallic_factor;
    float     roughness_factor;
    float     ao_strength;
    float     emissive_strength;
    glm::vec4 emissive_factor;      // xyz = emissive color, w = unused
    float     alpha_cutoff;
    float     ior;
    uint32_t  texture_flags;
    uint32_t  _pad;
};
```

### 验证

- `PBRMaterialParams` 默认值渲染出中性灰色非金属球
- `MaterialInstance::create_default()` 正确创建

## Day 0-1: 材质加载

### OBJ .mtl 加载

修改 `obj_io.hpp`，从 tinyobjloader 中提取材质：

1. **提取 .mtl 参数映射**
   ```cpp
   // tinyobjloader 已经解析 .mtl，当前代码忽略了 materials 向量
   // 需要提取：
   struct ObjMaterial {
       std::string name;

       // 标量参数
       glm::vec3 diffuse;       // Kd → base_color
       glm::vec3 specular;      // Ks → 用于估算 metallic
       glm::vec3 emission;      // Ke → emissive
       float     shininess;     // Ns → roughness ≈ 1 - sqrt(Ns / 1000)
       float     dissolve;      // d  → alpha
       float     ior;           // Ni → ior
       int       illum;         // illumination model

       // 纹理路径
       std::string diffuse_tex;       // map_Kd → albedo map
       std::string specular_tex;      // map_Ks
       std::string normal_tex;        // map_Bump / bump → normal map
       std::string roughness_tex;     // map_Pr (PBR 扩展)
       std::string metallic_tex;      // map_Pm (PBR 扩展)
       std::string ao_tex;            // map_Ka → AO map
       std::string emissive_tex;      // map_Ke → emissive map
   };
   ```

2. **OBJ → PBR 参数转换**
   ```cpp
   PBRMaterialParams obj_material_to_pbr(const ObjMaterial& mtl) {
       PBRMaterialParams pbr;
       pbr.base_color = mtl.diffuse;
       pbr.emissive = mtl.emission;
       pbr.ior = mtl.ior;

       // 粗糙度：从 Phong shininess 近似转换
       // Ns 范围 [0, 1000]，roughness 范围 [0, 1]
       pbr.roughness = 1.0f - std::sqrt(std::clamp(mtl.shininess / 1000.0f, 0.0f, 1.0f));

       // 金属度：启发式 — 高 specular + 低 diffuse → 金属
       float spec_lum = glm::dot(mtl.specular, glm::vec3(0.2126f, 0.7152f, 0.0722f));
       float diff_lum = glm::dot(mtl.diffuse, glm::vec3(0.2126f, 0.7152f, 0.0722f));
       pbr.metallic = (spec_lum > 0.5f && diff_lum < 0.3f) ? 1.0f : 0.0f;

       // 实际项目中推荐直接使用 PBR 扩展字段
       return pbr;
   }
   ```

3. **纹理加载**
   - 通过 `ResourceManager<TextureResourceKind>` 加载纹理
   - albedo / emissive 纹理用 `sRGB = true`
   - normal / metallic-roughness / AO 纹理用 `sRGB = false`（线性空间）
   - 纹理路径相对于 .mtl 文件所在目录

### 可选：glTF 2.0 加载

glTF 是现代 PBR 资产的标准格式，原生 metallic-roughness 工作流。

1. **glTF 加载器** — `gltf_io.hpp`（可选，推荐使用 `tinygltf` 或 `cgltf`）
   ```cpp
   struct GltfLoadResult {
       std::vector<ObjVertex> vertices;  // 复用现有顶点类型 + tangent
       std::vector<uint32_t> indices;
       std::vector<PBRMaterialParams> materials;
       std::vector<TextureHandle> textures;
       // 每个 mesh primitive → material index
       std::vector<uint32_t> material_indices;
   };

   GltfLoadResult load_gltf(const std::string& path, ResourceManager& rm);
   ```

   glTF 的好处：
   - 材质参数直接是 metallic-roughness，无需转换
   - 纹理引用明确，含 sampler 配置
   - 通常已包含 tangent 向量
   - 大量免费 PBR 测试资产可用（Khronos sample models）

### 材质资源管理

2. **MaterialResourceKind** — 加入 `resource_kinds.hpp`
   ```cpp
   struct MaterialResourceKind {
       using cpu_type = PBRMaterialParams;
       using gpu_type = MaterialInstance;
       using options_type = MaterialCreateOptions;  // 纹理路径列表

       static cpu_type load(const std::string& path, const options_type& opts);
       static gpu_type upload(const cpu_type& data, /* device */);
   };
   ```

3. **MeshComponent 扩展**
   ```cpp
   // 修改 mesh_component.hpp
   class StaticMeshComponent : public MeshComponent {
       MeshHandle mesh_;
       MaterialHandle material_;   // 新增：指向 MaterialInstance
       // 移除原有的 Vec4 base_color
   };
   ```

### 验证

- 加载带 .mtl 的 OBJ 文件，正确提取材质参数
- 纹理路径解析正确，图片加载无错误
- sRGB / linear 色空间标记正确
- glTF 加载（如实现）正确提取所有参数

## 描述符集布局设计

这个阶段需要规划纹理描述符集，虽然实际绑定在 Phase D 实现：

```cpp
// 材质描述符集布局（Set 1，区别于 per-frame UBO 的 Set 0）
//
// Set 0: Per-frame 全局数据（VP matrix, lights, camera, etc.）
// Set 1: Per-material 数据（material UBO + 纹理）
// Set 2: Per-object 数据（model matrix, etc.）
//
// 使用频率分层：Set 0 每帧绑定一次，Set 1 每材质切换绑定，Set 2 每对象绑定

DescriptorSetLayout material_layout = DescriptorSetLayout::Builder(device)
    .add_binding(0, vk::DescriptorType::eUniformBuffer,
                 vk::ShaderStageFlagBits::eFragment)               // MaterialUBO
    .add_binding(1, vk::DescriptorType::eCombinedImageSampler,
                 vk::ShaderStageFlagBits::eFragment)               // albedo
    .add_binding(2, vk::DescriptorType::eCombinedImageSampler,
                 vk::ShaderStageFlagBits::eFragment)               // normal
    .add_binding(3, vk::DescriptorType::eCombinedImageSampler,
                 vk::ShaderStageFlagBits::eFragment)               // metallic-roughness
    .add_binding(4, vk::DescriptorType::eCombinedImageSampler,
                 vk::ShaderStageFlagBits::eFragment)               // AO
    .add_binding(5, vk::DescriptorType::eCombinedImageSampler,
                 vk::ShaderStageFlagBits::eFragment)               // emissive
    .build();
```

## Phase A 完成标准

- [ ] `PBRMaterialParams` 结构定义完整（base_color, metallic, roughness, ao, emissive, ior）
- [ ] `MaterialInstance` 管理参数 + 5 个纹理槽位
- [ ] 默认 1×1 纹理正确创建（白/蓝法线/黑）
- [ ] `GPUMaterialParams` 对齐到 std140 布局
- [ ] OBJ .mtl 加载正确提取材质参数和纹理路径
- [ ] OBJ Phong → PBR 参数转换合理
- [ ] 纹理加载 sRGB / linear 色空间正确
- [ ] `MaterialResourceKind` 注册到资源管理器
- [ ] `MeshComponent` 持有 `MaterialHandle`
- [ ] 描述符集布局设计完成（Set 0/1/2 分层）
