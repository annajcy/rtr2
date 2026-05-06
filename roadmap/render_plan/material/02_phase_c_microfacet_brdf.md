# Phase C: Microfacet BRDF 实现（~1.5 天）

## 目标

在 Slang shader 中实现完整的 Cook-Torrance microfacet BRDF，替换现有 Blinn-Phong。

## BRDF 函数库 — `shaders/brdf_common.slang`

这是整个材质系统的核心 shader 文件，所有光照 pass（forward / deferred / probe tracing）共用。

### 常量与工具函数

```slang
static const float PI = 3.14159265359;
static const float INV_PI = 0.31830988618;
static const float EPSILON = 1e-6;

float luminance(float3 color) {
    return dot(color, float3(0.2126, 0.7152, 0.0722));
}

// 将 roughness 重映射为 α（GGX 使用 α = roughness²）
float roughness_to_alpha(float roughness) {
    return roughness * roughness;
}

// 从 IOR 计算 F0
float ior_to_f0(float ior) {
    float r = (ior - 1.0) / (ior + 1.0);
    return r * r;
}
```

### D — GGX Normal Distribution Function

```slang
// Trowbridge-Reitz / GGX NDF
//
// D(h) =           α²
//        ───────────────────────
//        π · ((n·h)² · (α²-1) + 1)²
//
// 参数：
//   NdotH: saturate(dot(normal, half_vector))
//   alpha: roughness² (已重映射)
//
float D_GGX(float NdotH, float alpha) {
    float a2 = alpha * alpha;
    float denom = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * denom * denom + EPSILON);
}
```

**物理含义**：描述微表面法线朝向 half vector 的概率密度。
- α → 0：所有微面法线对齐宏观法线 → 完美镜面
- α → 1：微面法线均匀分布 → 完全粗糙

### G — Smith Geometry Function

```slang
// Schlick-GGX geometry function (单方向)
//
// G1(v) =       n · v
//         ──────────────────
//         (n·v) · (1 - k) + k
//
// k 的取值取决于使用场景：
//   直接光照: k = (roughness + 1)² / 8
//   IBL:     k = roughness² / 2
//
float G1_SchlickGGX(float NdotV, float k) {
    return NdotV / (NdotV * (1.0 - k) + k + EPSILON);
}

// Smith's method: 组合入射和出射方向的遮挡
//
// G(ωi, ωo) = G1(ωi) · G1(ωo)
//
float G_Smith(float NdotV, float NdotL, float roughness, bool is_ibl) {
    float k;
    if (is_ibl) {
        float a = roughness * roughness;
        k = a / 2.0;
    } else {
        float r = roughness + 1.0;
        k = r * r / 8.0;
    }
    return G1_SchlickGGX(NdotV, k) * G1_SchlickGGX(NdotL, k);
}
```

**物理含义**：微表面的自遮挡和自阴影。
- 掠射角（NdotV 或 NdotL → 0）时遮挡最严重
- 高粗糙度时遮挡更明显

### 可选升级：Height-Correlated Smith

```slang
// Height-Correlated Smith GGX (更精确，Filament 使用)
// 考虑入射和出射方向的相关性，避免过度遮挡
//
// V(ωi, ωo) = G(ωi, ωo) / (4 · NdotV · NdotL)
//           =            0.5
//             ────────────────────────────────────────
//             NdotL · √(NdotV² · (1-α²) + α²) + NdotV · √(NdotL² · (1-α²) + α²)
//
// 注意：此函数已包含 Cook-Torrance 分母中的 4 · NdotV · NdotL
// 所以最终 BRDF = D * V * F（不再除以 4·NdotV·NdotL）
//
float V_SmithGGXCorrelated(float NdotV, float NdotL, float alpha) {
    float a2 = alpha * alpha;
    float GGXV = NdotL * sqrt(NdotV * NdotV * (1.0 - a2) + a2);
    float GGXL = NdotV * sqrt(NdotL * NdotL * (1.0 - a2) + a2);
    return 0.5 / (GGXV + GGXL + EPSILON);
}

// 快速近似版本（避免 sqrt）
float V_SmithGGXCorrelatedFast(float NdotV, float NdotL, float alpha) {
    float GGXV = NdotL * (NdotV * (1.0 - alpha) + alpha);
    float GGXL = NdotV * (NdotL * (1.0 - alpha) + alpha);
    return 0.5 / (GGXV + GGXL + EPSILON);
}
```

初期用标准 `G_Smith`，性能调优时可切换到 `V_SmithGGXCorrelated`。

### F — Fresnel

```slang
// Schlick Fresnel 近似
//
// F(cosθ, F0) = F0 + (1 - F0) · (1 - cosθ)⁵
//
float3 F_Schlick(float cosTheta, float3 F0) {
    float t = 1.0 - cosTheta;
    float t2 = t * t;
    float t5 = t2 * t2 * t;
    return F0 + (1.0 - F0) * t5;
}

// 带粗糙度的 Fresnel（用于 IBL 的漫反射/镜面分割）
float3 F_SchlickRoughness(float cosTheta, float3 F0, float roughness) {
    return F0 + (max(float3(1.0 - roughness), F0) - F0) *
           pow(1.0 - cosTheta, 5.0);
}
```

**物理含义**：掠射角时反射率趋近 100%（所有材质在接近 90° 时变成镜面）。

### 完整 Cook-Torrance BRDF

```slang
struct BRDFResult {
    float3 diffuse;      // 漫反射贡献
    float3 specular;     // 镜面反射贡献
    float3 total;        // diffuse + specular
};

// 材质输入结构
struct MaterialInput {
    float3 base_color;
    float  metallic;
    float  roughness;
    float  ao;
};

BRDFResult evaluate_brdf(
    float3 N,            // 世界空间法线（可能来自法线贴图）
    float3 V,            // 观察方向
    float3 L,            // 光源方向
    MaterialInput mat
) {
    BRDFResult result;

    float3 H = normalize(V + L);

    float NdotV = max(dot(N, V), EPSILON);
    float NdotL = max(dot(N, L), 0.0);
    float NdotH = max(dot(N, H), 0.0);
    float VdotH = max(dot(V, H), 0.0);

    float alpha = roughness_to_alpha(mat.roughness);

    // F0: 电介质 = 0.04，金属 = base_color
    float3 F0 = lerp(float3(0.04), mat.base_color, mat.metallic);

    // 各组件
    float  D = D_GGX(NdotH, alpha);
    float  G = G_Smith(NdotV, NdotL, mat.roughness, false);
    float3 F = F_Schlick(VdotH, F0);

    // 镜面项
    result.specular = (D * G * F) / (4.0 * NdotV * NdotL + EPSILON);

    // 漫反射项
    // kd = (1 - F) * (1 - metallic)
    // 金属无漫反射，Fresnel 反射的部分不参与漫反射
    float3 kd = (1.0 - F) * (1.0 - mat.metallic);
    result.diffuse = kd * mat.base_color * INV_PI;

    result.total = (result.diffuse + result.specular) * NdotL;

    return result;
}
```

### 能量守恒验证

```
对于任何 (roughness, metallic, NdotL, NdotV) 组合：
∫ fr(ωi, ωo) · cosθ dωi ≤ 1

验证方法：
1. 白炉测试 (White Furnace Test)：
   - 设置 base_color = (1,1,1), metallic = 0
   - 用均匀环境光照射
   - 理想情况下输出亮度 ≤ 入射亮度
2. 多粗糙度球体渲染 → 总亮度不应超过入射光
```

> **注意**：标准 Schlick-GGX 在高粗糙度 + 掠射角时可能轻微违反能量守恒。
> 如果需要严格守恒，可后续加入多散射补偿项（Kulla-Conty）。

## 集成到现有 Forward Shader

### 替换 `vert_buffer.slang`

在 SRT pipeline 完成前，先在现有 forward pass 中替换 Blinn-Phong：

```slang
// vert_buffer.slang 修改

#include "brdf_common.slang"
#include "material_types.slang"

struct UniformBufferObject {
    float4x4 model, view, proj, normal_matrix;
    MaterialParams material;     // 替换原 base_color
    PointLight lights[4];
    float3 camera_world_pos;
    uint light_count;
};

float4 frag_main(VSOutput input) : SV_Target {
    float3 N = normalize(input.world_normal);
    float3 V = normalize(ubo.camera_world_pos - input.world_pos);

    MaterialInput mat;
    mat.base_color = ubo.material.base_color.xyz;
    mat.metallic = ubo.material.metallic;
    mat.roughness = ubo.material.roughness;
    mat.ao = ubo.material.ao;

    float3 Lo = float3(0);

    for (uint i = 0; i < ubo.light_count; i++) {
        PointLight light = ubo.lights[i];
        float3 L_vec = light.position - input.world_pos;
        float dist = length(L_vec);
        float3 L = L_vec / dist;

        float3 radiance = light.color * light.intensity / (dist * dist);

        BRDFResult brdf = evaluate_brdf(N, V, L, mat);
        Lo += brdf.total * radiance;
    }

    // 环境项（简单版，IBL 前的占位）
    float3 ambient = float3(0.03) * mat.base_color * mat.ao;

    return float4(Lo + ambient, 1.0);
}
```

### 材质参数 Shader 结构 — `shaders/material_types.slang`

```slang
struct MaterialParams {
    float4 base_color_factor;    // xyz = color, w = alpha
    float  metallic_factor;
    float  roughness_factor;
    float  ao_strength;
    float  emissive_strength;
    float4 emissive_factor;      // xyz = emissive color
    float  alpha_cutoff;
    float  ior;
    uint   texture_flags;
    uint   _pad;
};
```

## Phase C 完成标准

- [ ] `D_GGX()` 实现正确 — roughness=0 极窄峰值，roughness=1 均匀
- [ ] `G_Smith()` 实现正确 — 掠射角明显衰减
- [ ] `F_Schlick()` 实现正确 — cosθ=0 时返回 F0，cosθ≈0 时趋近 1
- [ ] `evaluate_brdf()` 完整组装 Cook-Torrance
- [ ] metallic=0 时漫反射主导，metallic=1 时无漫反射
- [ ] roughness=0 时锐利高光，roughness=1 时高光消失
- [ ] 能量守恒：白炉测试输出 ≤ 输入
- [ ] Forward shader 替换 Blinn-Phong 后视觉正确
- [ ] 材质球阵列渲染（roughness × metallic 网格）视觉正确

## 关键参考

- "Microfacet Models for Refraction through Rough Surfaces" (Walter et al. 2007) — GGX NDF 原始论文
- "Real Shading in Unreal Engine 4" (Karis 2013) — 工业标准 PBR 概述
- "Physically Based Shading at Disney" (Burley 2012) — Disney BRDF 参数化
- Google Filament PBR 文档 — 工程实现参考
- LearnOpenGL PBR 章节 — 教学参考
