# Phase D: 纹理采样管线（~1.5 天）

## 目标

将 Phase A 定义的 5 个纹理槽位实际绑定到描述符集，在 shader 中采样纹理并与标量参数组合。

## 描述符集绑定

### Per-Material 描述符集创建

```cpp
class MaterialDescriptorManager {
    vk::raii::DescriptorSetLayout material_layout_;
    vk::raii::DescriptorPool material_pool_;

    // 每个 MaterialInstance 创建一个描述符集
    vk::raii::DescriptorSet create_material_descriptor(
        const MaterialInstance& material,
        const MaterialDefaults& defaults
    ) {
        auto set = allocate_from_pool();
        DescriptorWriter writer;

        // Binding 0: Material UBO
        writer.write_buffer(set, 0, material.gpu_ubo());

        // Binding 1-5: 纹理采样器
        for (uint32_t slot = 0; slot < 5; slot++) {
            const auto& tex = material.has_texture(slot)
                ? material.texture(slot)
                : defaults.get_default(slot);

            writer.write_image(set, slot + 1,
                tex.image_view(), defaults.default_sampler(),
                vk::ImageLayout::eShaderReadOnlyOptimal);
        }

        return set;
    }
};
```

### Pipeline Layout 调整

```cpp
// 从单 set 升级为 3 set
// Set 0: Per-frame globals (VP matrix, camera, lights)
// Set 1: Per-material (material UBO + 5 textures)
// Set 2: Per-object (model matrix)

vk::PipelineLayout create_pipeline_layout(
    const vk::raii::DescriptorSetLayout& frame_layout,
    const vk::raii::DescriptorSetLayout& material_layout,
    const vk::raii::DescriptorSetLayout& object_layout
) {
    std::array<vk::DescriptorSetLayout, 3> layouts = {
        *frame_layout, *material_layout, *object_layout
    };
    // ...
}
```

### 渲染时的绑定顺序

```cpp
void render_scene(CommandBuffer& cmd, const SceneView& scene) {
    cmd.bind_pipeline(pbr_pipeline);

    // Set 0: 每帧绑定一次
    cmd.bind_descriptor_set(0, frame_descriptor);

    MaterialHandle last_material = INVALID;

    for (const auto& renderable : scene.renderables) {
        // Set 1: 材质变化时才重新绑定（排序后切换最少）
        if (renderable.material != last_material) {
            cmd.bind_descriptor_set(1, renderable.material_descriptor);
            last_material = renderable.material;
        }

        // Set 2: 每个对象绑定
        cmd.bind_descriptor_set(2, renderable.object_descriptor);

        cmd.draw_indexed(renderable.mesh);
    }
}
```

> **性能关键**：按材质排序 renderable，最大化减少描述符集切换。
> 材质切换开销 > 对象 UBO 更新开销。

## Shader 纹理采样

### 完整 Fragment Shader — 纹理 + 标量参数组合

```slang
// material_sampling.slang

// 描述符集 1（per-material）
[[vk::binding(0, 1)]] ConstantBuffer<MaterialParams> material;
[[vk::binding(1, 1)]] Texture2D albedo_map;
[[vk::binding(2, 1)]] Texture2D normal_map;
[[vk::binding(3, 1)]] Texture2D metallic_roughness_map;
[[vk::binding(4, 1)]] Texture2D ao_map;
[[vk::binding(5, 1)]] Texture2D emissive_map;
[[vk::binding(6, 1)]] SamplerState material_sampler;  // 共用采样器

// 从纹理 + 标量因子组合最终材质参数
MaterialInput sample_material(float2 uv) {
    MaterialInput mat;

    // Albedo: 纹理 × factor
    // 默认纹理 = 白色 → sample = (1,1,1,1) → 结果 = base_color_factor
    float4 albedo_sample = albedo_map.Sample(material_sampler, uv);
    mat.base_color = albedo_sample.rgb * material.base_color_factor.rgb;
    float alpha = albedo_sample.a * material.base_color_factor.a;

    // Alpha test
    if (alpha < material.alpha_cutoff) {
        discard;
    }

    // Metallic-Roughness: glTF 约定 G=roughness, B=metallic
    // 默认纹理 = (0, roughness_factor, metallic_factor, 0) — 或者：
    // 使用白色默认纹理 → sample = (1,1,1,1) → roughness = 1 * roughness_factor
    float4 mr_sample = metallic_roughness_map.Sample(material_sampler, uv);
    mat.roughness = mr_sample.g * material.roughness_factor;
    mat.metallic = mr_sample.b * material.metallic_factor;

    // Clamp roughness 下限（避免 D_GGX 的数值问题）
    mat.roughness = max(mat.roughness, 0.04);

    // AO
    float ao_sample = ao_map.Sample(material_sampler, uv).r;
    mat.ao = lerp(1.0, ao_sample, material.ao_strength);

    return mat;
}

// 自发光采样
float3 sample_emissive(float2 uv) {
    float3 emissive_sample = emissive_map.Sample(material_sampler, uv).rgb;
    return emissive_sample * material.emissive_factor.rgb * material.emissive_strength;
}
```

### 法线贴图采样（与 Phase B TBN 结合）

```slang
// 在 fragment shader 中
float3 get_shading_normal(float2 uv, float3 world_normal, float4 world_tangent) {
    float3 N = get_normal_from_map(uv, world_normal, world_tangent,
                                    normal_map, material_sampler);
    return N;
}
```

### 完整 Fragment Shader 整合

```slang
#include "brdf_common.slang"
#include "material_types.slang"
#include "normal_mapping.slang"

float4 frag_main(VSOutput input) : SV_Target {
    // 1. 采样材质参数
    MaterialInput mat = sample_material(input.uv);

    // 2. 获取着色法线（法线贴图）
    float3 N = get_shading_normal(input.uv, input.world_normal, input.world_tangent);
    float3 V = normalize(camera_pos - input.world_pos);

    // 3. 光照计算
    float3 Lo = float3(0);

    for (uint i = 0; i < light_count; i++) {
        GPULight light = lights[i];
        float3 L_vec = light.position.xyz - input.world_pos;
        float dist = length(L_vec);
        float3 L = L_vec / dist;
        float3 radiance = light.color.xyz * light.color.w / (dist * dist);

        BRDFResult brdf = evaluate_brdf(N, V, L, mat);
        Lo += brdf.total * radiance;
    }

    // 4. 环境 + 自发光
    float3 ambient = float3(0.03) * mat.base_color * mat.ao;
    float3 emissive = sample_emissive(input.uv);

    return float4(Lo + ambient + emissive, 1.0);
}
```

## 纹理压缩与格式

### 纹理格式推荐

| 纹理 | 格式 | 色空间 | 说明 |
|------|------|--------|------|
| Albedo | RGBA8 / BC7 | sRGB | 颜色纹理必须 sRGB |
| Normal | RG16 / BC5 | Linear | 只需 XY，Z 重建 |
| Metallic-Roughness | RGBA8 / BC7 | Linear | G=roughness, B=metallic |
| AO | R8 / BC4 | Linear | 单通道 |
| Emissive | RGBA8 / BC7 | sRGB | 颜色纹理 |

### 法线贴图 Z 重建

```slang
// 如果法线贴图只有 RG 通道（BC5 压缩）
float3 unpack_normal_rg(float2 rg) {
    float3 n;
    n.xy = rg * 2.0 - 1.0;
    n.z = sqrt(max(0, 1.0 - dot(n.xy, n.xy)));
    return n;
}
```

## 与 SRT Pipeline (G-Buffer) 的对接

Phase D 的纹理采样逻辑直接复用到 SRT 的 G-Buffer pass：

```slang
// gbuffer.slang (SRT pipeline)

GBufferOutput gbuffer_frag(VSOutput input) {
    MaterialInput mat = sample_material(input.uv);
    float3 N = get_shading_normal(input.uv, input.world_normal, input.world_tangent);
    float3 emissive = sample_emissive(input.uv);

    GBufferOutput o;
    o.rt0 = float4(mat.base_color, mat.metallic);
    o.rt1 = float4(N * 0.5 + 0.5, mat.roughness);
    o.rt2 = float4(input.world_pos, linear_depth);
    o.emissive = float4(emissive, 1.0);
    return o;
}
```

材质采样代码只写一次，forward pass 和 deferred G-Buffer pass 共用。

## Phase D 完成标准

- [ ] 描述符集 Set 1 正确绑定 material UBO + 5 个纹理
- [ ] Pipeline layout 支持 3 个描述符集（frame / material / object）
- [ ] 渲染按材质排序，最小化描述符切换
- [ ] Albedo 纹理采样 × factor 正确
- [ ] Metallic-Roughness 纹理 glTF 约定 (G=roughness, B=metallic) 正确
- [ ] AO 纹理采样正确
- [ ] Emissive 纹理采样正确
- [ ] 法线贴图 + TBN 产生可见凹凸效果
- [ ] 无纹理时使用默认纹理，结果等同于标量参数（无 shader 分支）
- [ ] Alpha test 正确 discard
- [ ] roughness 下限 0.04 避免 NaN
- [ ] 纹理 sRGB / linear 色空间正确（albedo/emissive = sRGB，其余 linear）
