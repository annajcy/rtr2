# Phase E: IBL 环境光照（~1 天）

## 目标

用 Image-Based Lighting 替代固定 `vec3(0.03) * base_color` 环境项，提供基于环境贴图的物理正确间接光照近似。

> **与 SDFGI 的关系**：IBL 是 SRT pipeline 完成前的过渡方案。SRT pipeline 完成后，
> SDFGI 提供真正的动态 GI，IBL 退化为远景天空光 / 大气背景。
> 但 IBL 仍有独立价值：作为 skybox、反射参考、和不需要 GI 的简单场景的光照。

## IBL 原理

环境光照的渲染方程：

```
Lo(p, ωo) = ∫ fr(p, ωi, ωo) · Li(ωi) · (n · ωi) dωi
```

将 `fr` 拆分为漫反射和镜面反射两项，分别预计算：

### 漫反射 IBL

```
L_diffuse = kd · (albedo / π) · ∫ Li(ωi) · (n · ωi) dωi
                                 ─────────────────────────
                                    irradiance(n)

irradiance(n) = 对环境贴图做 cosine 卷积
             → 预卷积到 irradiance cubemap（32×32 per face 就够）
```

### 镜面 IBL（Split-Sum 近似）

```
L_specular = ∫ fr(ωi, ωo) · Li(ωi) · (n · ωi) dωi
           ≈ [ ∫ Li(ωi) · D(ωi, ωo) dωi ] × [ ∫ fr(ωi, ωo) · (n · ωi) dωi ]
             ────────────────────────────     ────────────────────────────────
             pre-filtered env map              BRDF integration LUT
             (roughness → mip level)           (2D texture, NdotV × roughness)
```

**Split-Sum**（Epic 2013）：将积分拆成两个独立部分的乘积近似。

## 任务

### 1. 环境贴图加载

```cpp
// 支持 HDR equirectangular（.hdr / .exr）→ cubemap 转换
class EnvironmentMap {
    rhi::Image cubemap_;          // 6 face, RGBA16F, 512×512
    rhi::Image irradiance_;       // 6 face, RGBA16F, 32×32 (漫反射)
    rhi::Image prefiltered_;      // 6 face, RGBA16F, 128×128, 5 mip levels (镜面)
    rhi::ImageView cubemap_view_;
    rhi::ImageView irradiance_view_;
    rhi::ImageView prefiltered_view_;

    void load_from_hdr(const std::string& path);
    void precompute(/* device */);  // 离线或首帧 GPU 计算
};
```

### 2. Equirectangular → Cubemap 转换（Compute Shader）

```slang
// shaders/equirect_to_cubemap.slang
// 对 cubemap 6 个面的每个 texel，计算方向 → 采样 equirectangular HDR
[numthreads(16, 16, 1)]
void main(uint3 tid : SV_DispatchThreadID) {
    uint2 pixel = tid.xy;
    uint face = tid.z;

    float2 uv = (float2(pixel) + 0.5) / float2(face_size);
    float3 dir = cubemap_direction(face, uv);

    // Equirectangular → UV
    float2 equirect_uv = direction_to_equirect(dir);
    float3 color = hdr_texture.Sample(linear_sampler, equirect_uv).rgb;

    cubemap_faces[face][pixel] = float4(color, 1.0);
}
```

### 3. Irradiance Convolution（Compute Shader）

```slang
// shaders/irradiance_convolve.slang
// 对每个方向 n，积分 Li(ωi) · max(0, n·ωi) 在半球上
[numthreads(16, 16, 1)]
void main(uint3 tid : SV_DispatchThreadID) {
    float3 N = cubemap_direction(tid.z, texel_to_uv(tid.xy));

    float3 irradiance = float3(0);
    float total_weight = 0;

    // 均匀半球采样（或用 importance sampling）
    // 推荐 ~2048 个样本
    for (uint i = 0; i < SAMPLE_COUNT; i++) {
        float2 xi = hammersley(i, SAMPLE_COUNT);
        float3 sample_dir = cosine_hemisphere_sample(xi, N);
        float NdotS = max(dot(N, sample_dir), 0.0);

        irradiance += cubemap.Sample(linear_sampler, sample_dir).rgb * NdotS;
        total_weight += NdotS;
    }

    irradiance_map[tid.z][tid.xy] = float4(irradiance / total_weight * PI, 1.0);
}
```

### 4. Pre-filtered Environment Map（Compute Shader）

```slang
// shaders/prefilter_envmap.slang
// 对每个 mip level（对应不同 roughness），用 GGX importance sampling 卷积
[numthreads(16, 16, 1)]
void main(uint3 tid : SV_DispatchThreadID) {
    float3 N = cubemap_direction(tid.z, texel_to_uv(tid.xy, mip_size));
    float3 R = N;  // 假设 V = N = R（split-sum 的近似条件）
    float3 V = N;

    float roughness = float(mip_level) / float(MAX_MIP_LEVELS - 1);
    float alpha = roughness * roughness;

    float3 prefiltered = float3(0);
    float total_weight = 0;

    for (uint i = 0; i < SAMPLE_COUNT; i++) {
        float2 xi = hammersley(i, SAMPLE_COUNT);
        float3 H = importance_sample_ggx(xi, N, alpha);
        float3 L = reflect(-V, H);

        float NdotL = max(dot(N, L), 0.0);
        if (NdotL > 0) {
            prefiltered += cubemap.SampleLevel(linear_sampler, L, 0).rgb * NdotL;
            total_weight += NdotL;
        }
    }

    prefiltered_map[mip_level][tid.z][tid.xy] = float4(prefiltered / total_weight, 1.0);
}
```

### 5. BRDF Integration LUT — `brdf_lut.hpp`

```slang
// shaders/brdf_lut.slang
// 2D LUT: u = NdotV, v = roughness → (scale, bias) for F0
// 只需计算一次（离线），512×512 RG16F
[numthreads(16, 16, 1)]
void main(uint3 tid : SV_DispatchThreadID) {
    float NdotV = (float(tid.x) + 0.5) / float(LUT_SIZE);
    float roughness = (float(tid.y) + 0.5) / float(LUT_SIZE);
    float alpha = roughness * roughness;

    float3 V = float3(sqrt(1.0 - NdotV * NdotV), 0, NdotV);
    float3 N = float3(0, 0, 1);

    float A = 0;  // scale
    float B = 0;  // bias

    for (uint i = 0; i < SAMPLE_COUNT; i++) {
        float2 xi = hammersley(i, SAMPLE_COUNT);
        float3 H = importance_sample_ggx(xi, N, alpha);
        float3 L = reflect(-V, H);

        float NdotL = max(L.z, 0.0);
        float NdotH = max(H.z, 0.0);
        float VdotH = max(dot(V, H), 0.0);

        if (NdotL > 0) {
            float G = G_Smith(NdotV, NdotL, roughness, true);
            float G_Vis = G * VdotH / (NdotH * NdotV + EPSILON);
            float Fc = pow(1.0 - VdotH, 5.0);

            A += (1.0 - Fc) * G_Vis;
            B += Fc * G_Vis;
        }
    }

    brdf_lut[tid.xy] = float2(A, B) / float(SAMPLE_COUNT);
}
```

### 6. IBL 着色（集成到 lighting pass）

```slang
// 在 fragment shader 的环境光部分
float3 evaluate_ibl(float3 N, float3 V, MaterialInput mat) {
    float NdotV = max(dot(N, V), EPSILON);
    float3 R = reflect(-V, N);

    float3 F0 = lerp(float3(0.04), mat.base_color, mat.metallic);

    // 漫反射 IBL
    float3 F = F_SchlickRoughness(NdotV, F0, mat.roughness);
    float3 kd = (1.0 - F) * (1.0 - mat.metallic);
    float3 irradiance = irradiance_map.Sample(linear_sampler, N).rgb;
    float3 diffuse_ibl = kd * mat.base_color * irradiance;

    // 镜面 IBL
    float mip = mat.roughness * float(MAX_MIP_LEVELS - 1);
    float3 prefiltered = prefiltered_map.SampleLevel(linear_sampler, R, mip).rgb;
    float2 brdf_lut = brdf_lut_texture.Sample(clamp_sampler, float2(NdotV, mat.roughness)).rg;
    float3 specular_ibl = prefiltered * (F * brdf_lut.x + brdf_lut.y);

    return (diffuse_ibl + specular_ibl) * mat.ao;
}
```

## Skybox 渲染

IBL 环境贴图同时用作 skybox 背景：

```slang
// shaders/skybox.slang
// 在 G-Buffer 中 depth = 无穷远的像素
// 或者用一个全屏三角形 + cubemap 采样

float4 skybox_frag(float3 view_dir) : SV_Target {
    float3 color = cubemap.Sample(linear_sampler, view_dir).rgb;
    return float4(color, 1.0);
}
```

## Phase E 完成标准

- [ ] HDR equirectangular 加载 + cubemap 转换正确
- [ ] Irradiance cubemap 预卷积视觉正确（低频环境光）
- [ ] Pre-filtered env map 5 级 mip 正确（roughness=0 镜面，roughness=1 模糊）
- [ ] BRDF LUT 生成正确（与参考实现对比）
- [ ] IBL 漫反射项：金属球不应有漫反射环境光
- [ ] IBL 镜面项：光滑金属球反射清晰环境，粗糙球反射模糊
- [ ] AO 正确影响环境光（AO 低的区域暗）
- [ ] Skybox 正确渲染背景
- [ ] 性能：IBL 预计算 < 100ms（首帧 compute shader）
