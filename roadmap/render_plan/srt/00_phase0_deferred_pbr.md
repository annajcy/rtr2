# Phase 0: Deferred Rendering + PBR + 阴影 + 面光源基础（~3 天）

## 目标

从前向渲染切换到 Deferred Rendering，建立 PBR 材质系统、基础阴影和面光源支持，为后续 SDFGI/ReSTIR 提供输入。

## 为什么必须先做

- SDFGI 需要场景 albedo/normal/roughness 信息 → G-Buffer
- ReSTIR 需要逐像素几何信息做 reservoir 复用判断 → G-Buffer
- 没有 PBR 材质，GI 的能量守恒无意义
- 阴影是最基本的视觉参考，也是 ReSTIR 可见性判断的基础

## Day 0-1: G-Buffer Pass

### G-Buffer 布局

```
RT0 (RGBA16F): albedo.rgb + metallic
RT1 (RGBA16F): world_normal.xyz + roughness
RT2 (RGBA32F): world_position.xyz + linear_depth  (或用 depth buffer 重建)
Depth (D32F):  硬件深度
```

> 优化方案：position 可通过 depth + inverse VP 矩阵重建，省一个 RT。初期先用显式 position，后期优化。

### 任务

1. **PBR 材质结构** — 复用 `material/pbr_material.hpp`（详见 `material_plan`）
   - `PBRMaterialParams`：base_color, metallic, roughness, ao, emissive, ior
   - `MaterialInstance`：参数 + 5 个纹理槽位
   - `brdf_common.slang`：GGX + Smith + Fresnel microfacet BRDF
   - 材质系统 Phase A-D 必须在 SRT Phase 0 之前完成

2. **G-Buffer 数据管理** — `gbuffer_data.hpp`
   - 创建 4 个 render target（Vulkan Image + ImageView）
   - 硬件 depth buffer
   - 统一 descriptor set 给后续 pass 采样

3. **G-Buffer Pass** — `gbuffer_pass.hpp`
   - 继承 / 参考 `ForwardPass` 的动态渲染模式
   - 多 render target 输出（VK_KHR_dynamic_rendering 支持 MRT）
   - vertex shader: MVP 变换 + 世界空间 normal/position 输出
   - fragment shader: 写入 G-Buffer，不做光照

4. **G-Buffer Shader** — `shaders/srt/gbuffer.slang`
   ```slang
   struct GBufferOutput {
       float4 rt0 : SV_Target0;  // albedo.rgb, metallic
       float4 rt1 : SV_Target1;  // normal.xyz, roughness
       float4 rt2 : SV_Target2;  // position.xyz, depth
   };
   ```

### 验证

- 分别可视化每个 G-Buffer 通道（写一个 debug 可视化 compute shader）
- 确认 normal 在世界空间，值域 [-1, 1]
- 确认 depth 线性化正确

## Day 1-2: PBR 光照 Pass（Deferred Lighting）

### 任务

1. **Lighting Pass** — `deferred_lighting_pass.hpp`
   - 全屏 compute shader（不用光栅化三角形）
   - 读 G-Buffer，计算 PBR BRDF
   - 输出到 HDR color buffer (RGBA16F)

2. **Cook-Torrance BRDF** — `shaders/srt/pbr_lighting.slang`
   ```slang
   // GGX NDF
   float D_GGX(float NdotH, float roughness);
   // Schlick-GGX Geometry
   float G_SchlickGGX(float NdotV, float NdotL, float roughness);
   // Fresnel-Schlick
   float3 F_Schlick(float cosTheta, float3 F0);

   // 完整 Cook-Torrance
   float3 cookTorranceBRDF(float3 N, float3 V, float3 L,
                           float3 albedo, float metallic, float roughness);
   ```

3. **Light Buffer** — 支持多光源（含面光源）的 SSBO
   ```cpp
   // 光源类型
   enum class LightType : uint32_t {
       Point       = 0,
       Directional = 1,
       Spot        = 2,
       RectArea    = 3,   // 矩形面光源
       DiskArea    = 4,   // 圆盘面光源
       SphereArea  = 5,   // 球形面光源
   };

   struct GPULight {
       glm::vec4 position;   // xyz = pos/center, w = type
       glm::vec4 color;      // xyz = color, w = intensity
       glm::vec4 direction;  // xyz = normal/direction, w = falloff
       glm::vec4 params;     // 点/spot: attenuation, spot angle
                             // rect: half_width, half_height, 0, 0
                             // disk: radius, 0, 0, 0
                             // sphere: radius, 0, 0, 0
       glm::vec4 right;      // xyz = local X axis (rect/disk), w = unused
                             // 用于定义面光源的局部坐标系
   };
   ```
   - 用 storage buffer 传入，不限数量（为 ReSTIR 准备）
   - 面光源通过 position + direction + right 定义局部坐标系（up = cross(direction, right)）
   - params 根据光源类型解释不同字段

4. **面光源定义** — `area_light.hpp`
   ```cpp
   struct AreaLight {
       glm::vec3 center;
       glm::vec3 normal;      // 面光源法线方向
       glm::vec3 right;       // 局部 X 轴
       glm::vec3 up;          // 局部 Y 轴 = cross(normal, right)
       glm::vec3 emission;    // 辐射率 (W/m²/sr)
       LightType type;
       float     half_width;  // rect only
       float     half_height; // rect only
       float     radius;      // disk/sphere only

       // 面光源面积（用于 PDF 计算）
       float area() const;

       // 获取面光源的 4 个顶点（rect only，LTC 用）
       std::array<glm::vec3, 4> corners() const;
   };
   ```

5. **G-Buffer 自发光通道**
   - 面光源几何体需要在 G-Buffer 中标记为 emissive
   - 在 RT0 或单独 RT 中输出 emissive color
   - 用于 bloom 和直接可视化面光源本身的亮度

### 验证

- Cornell Box 场景下 PBR 光照视觉正确
- metallic = 1 时出现金属高光
- roughness = 0 → 锐利高光，roughness = 1 → 漫反射主导

## Day 2: LTC 面光源解析着色

### 为什么用 LTC

面光源着色有两条路径，在不同阶段使用：

| 路径 | 用途 | 阶段 |
|------|------|------|
| **LTC（Linearly Transformed Cosines）** | 解析面光源着色，无噪点 | Phase 0 — deferred lighting |
| **随机面采样 + ReSTIR** | 随机采样面光源表面，支持阴影 | Phase 3 — ReSTIR DI |

LTC 在 Phase 0 中提供**无噪点的面光源直接光照**（不含阴影），后续 Phase 3 中 ReSTIR 通过随机采样面光源表面点来处理可见性和阴影。两者互补：

- LTC：无噪点、无阴影 → 适合 preview / fallback
- ReSTIR 采样：有少量噪点、有阴影 → 最终输出

### LTC 核心思想

将 GGX BRDF lobe 近似为一个线性变换后的 cosine 分布，使得面光源在该分布下的积分有解析解。

关键：两张预计算 LUT（64×64，roughness × NdotV）：
- `LTC1` (mat3): 变换矩阵 M 的逆
- `LTC2` (vec2): Fresnel 积分系数

### 任务

1. **LTC LUT** — `ltc_lut.hpp`
   ```cpp
   // 预计算 LTC 查找表（离线生成，编译时嵌入）
   // 来源: https://github.com/selfshadow/ltc_code
   // 格式: 64×64 RGBA32F texture
   //   LTC1: 4 个矩阵元素 (m00, m02, m20, m22)，其余为 identity
   //   LTC2: (Fresnel_scale, Fresnel_bias, 0, sphere_form_factor)
   class LTCLookupTable {
       vk::raii::Image     ltc1_texture_;  // 64×64 RGBA32F
       vk::raii::Image     ltc2_texture_;  // 64×64 RGBA32F
       vk::raii::ImageView ltc1_view_;
       vk::raii::ImageView ltc2_view_;
       vk::raii::Sampler   ltc_sampler_;   // bilinear, clamp

       void init(/* device */);  // 加载预计算数据
   };
   ```

2. **LTC 着色** — `ltc.hpp` + `shaders/srt/ltc_area_light.slang`
   ```slang
   // 在 deferred lighting pass 中对每个面光源调用
   float3 evaluate_rect_area_light_ltc(
       float3 pos, float3 normal, float3 V,
       float3 albedo, float metallic, float roughness,
       float3 light_corners[4],  // 矩形 4 个顶点（世界空间）
       float3 light_emission
   ) {
       float NdotV = saturate(dot(normal, V));
       float2 uv = float2(roughness, sqrt(1.0 - NdotV));

       // 查 LTC LUT
       float4 ltc1 = ltc1_texture.Sample(ltc_sampler, uv);
       float4 ltc2 = ltc2_texture.Sample(ltc_sampler, uv);

       // 构建逆变换矩阵 M^-1
       float3x3 Minv = float3x3(
           float3(ltc1.x, 0, ltc1.y),
           float3(0,      1, 0),
           float3(ltc1.z, 0, ltc1.w)
       );

       // 构建 TBN
       float3 T1 = normalize(V - normal * NdotV);
       float3 T2 = cross(normal, T1);
       float3x3 TBN = float3x3(T1, T2, normal);

       // 将面光源顶点变换到 LTC 空间
       float3x3 M = mul(Minv, transpose(TBN));
       float3 transformed[4];
       for (int i = 0; i < 4; i++) {
           transformed[i] = normalize(mul(M, light_corners[i] - pos));
       }

       // 计算球面多边形积分（解析）
       float specular_integral = polygon_irradiance(transformed, 4);
       float diffuse_integral = polygon_irradiance(
           /* identity transform corners */, 4);

       // Fresnel
       float3 F0 = lerp(float3(0.04), albedo, metallic);
       float3 fresnel = F0 * ltc2.x + (1.0 - F0) * ltc2.y;

       float3 diffuse_color = albedo * (1.0 - metallic);
       float3 result = light_emission * (
           diffuse_color * diffuse_integral +
           fresnel * specular_integral
       );

       return max(result, float3(0));
   }

   // 球面多边形积分（ClipQuadToHorizon + edge integration）
   float polygon_irradiance(float3 vertices[4], int vertex_count) {
       // 标准 LTC polygon integration
       // 参考 "Real-Time Polygonal-Light Shading with LTC" (Heitz et al. 2016)
       // 1. Clip polygon to upper hemisphere
       // 2. Sum edge integrations: acos(dot(v_i, v_{i+1})) * normalize(cross(v_i, v_{i+1})).z
       ...
   }
   ```

3. **集成到 Deferred Lighting Pass**
   - 点/方向/聚光灯：Cook-Torrance BRDF（已有）
   - 面光源：LTC 解析着色（新增）
   - 根据 `GPULight.type` 分支

### 验证

- 矩形面光源在光滑表面上产生正确的矩形高光
- roughness 增大 → 高光扩散
- 正对面光源 → 均匀亮区，侧面 → 衰减
- 与 reference path tracer 对比（无阴影情况下）

## Day 2-3: 阴影系统

### 方案：Cascaded Shadow Maps (CSM)

ReSTIR 中可见性检测依赖 shadow map 或 SDF ray march。初期用 CSM 保证基本阴影正确，后续阶段用 SDF 替代 / 补充。

### 任务

1. **Shadow Data** — `shadow_data.hpp`
   - 2-4 级 cascade，每级 1024x1024 或 2048x2048 depth map
   - 每级的 light-space VP 矩阵
   - Cascade split 使用 PSSM（Practical Split Scheme）

2. **Shadow Pass** — `shadow_pass.hpp`
   - 从主方向光视角渲染 depth-only pass
   - 每级 cascade 一次 draw

3. **Shadow Shader** — `shaders/srt/shadow.slang`
   - Vertex-only pass，输出 depth
   - 在 lighting pass 中采样 shadow map：PCF 5x5 软阴影

4. **集成到 Lighting Pass**
   - shadow factor 乘到直接光照上

### 验证

- 阴影边界连续（cascade 之间无明显跳变）
- PCF 产生柔和阴影边缘
- 相机移动时阴影稳定（无闪烁）

## SRT Pipeline 骨架

在这个阶段末尾，创建 `SRTPipeline` 主管线：

```cpp
class SRTPipeline : public RenderPipeline {
    GBufferPass   gbuffer_pass_;
    ShadowPass    shadow_pass_;
    LightingPass  lighting_pass_;
    PresentPass   present_pass_;

    void render(FrameTicket& ticket) override {
        shadow_pass_.execute(ticket);    // shadow maps
        gbuffer_pass_.execute(ticket);   // G-Buffer 填充
        lighting_pass_.execute(ticket);  // deferred PBR lighting
        present_pass_.execute(ticket);   // blit to swapchain
    }
};
```

## Phase 0 完成标准

- [ ] G-Buffer 4 通道正确输出并可 debug 可视化
- [ ] G-Buffer emissive 通道正确标记面光源几何体
- [ ] PBR Cook-Torrance BRDF 视觉正确
- [ ] 支持任意数量光源（含面光源）（SSBO）
- [ ] GPULight 结构支持 6 种光源类型（point/dir/spot/rect/disk/sphere）
- [ ] LTC 查找表正确加载（64×64 RGBA32F × 2）
- [ ] LTC 矩形面光源着色视觉正确（无噪点高光）
- [ ] CSM 阴影在方向光下正确
- [ ] SRTPipeline 骨架可运行，等效替代 ForwardPipeline
- [ ] 所有 shader 在 macOS (MoltenVK) 编译通过
