# RTR2 材质系统升级计划：标准 Microfacet PBR

## 现状

```
当前材质能力：
- 每 mesh 一个 Vec4 base_color，无其他参数
- Blinn-Phong 着色（shininess / specular_strength 绑定在光源上）
- 纹理系统存在（Image + Sampler + ResourceManager）但从未绑定到材质
- OBJ 加载器忽略 .mtl 材质文件
- 顶点格式：position + uv + normal，无 tangent
- 描述符集只绑定 UBO，无纹理采样器
```

## 目标

构建**标准 microfacet PBR 材质系统**：

1. **Cook-Torrance microfacet BRDF** — GGX NDF + Smith geometry + Fresnel-Schlick
2. **Metallic-Roughness 工作流** — 与 glTF 2.0 / Unreal / Unity 标准对齐
3. **完整纹理管线** — albedo / normal / metallic-roughness / AO / emissive
4. **切线空间** — tangent + bitangent 用于法线贴图
5. **IBL 基础** — 环境光预过滤 + BRDF LUT（可选）

## Microfacet 理论基础

### 渲染方程中的 BRDF

```
Lo(p, ωo) = ∫ fr(p, ωi, ωo) · Li(p, ωi) · (n · ωi) dωi
```

对于 microfacet 模型，BRDF 分为漫反射和镜面反射两部分：

```
fr = kd · f_diffuse + ks · f_specular
```

其中 `kd + ks ≤ 1`（能量守恒），由 metallic 参数控制分配。

### Cook-Torrance Specular BRDF

```
f_specular(ωi, ωo) = D(h) · F(ωo, h) · G(ωi, ωo, h)
                      ─────────────────────────────────
                           4 · (n · ωi) · (n · ωo)
```

三个组件：

| 组件 | 含义 | 选用模型 |
|------|------|----------|
| **D(h)** — Normal Distribution Function | 微表面法线分布 | GGX (Trowbridge-Reitz) |
| **F(ωo, h)** — Fresnel | 反射率随角度变化 | Schlick 近似 |
| **G(ωi, ωo, h)** — Geometry / Masking-Shadowing | 微表面自遮挡 | Smith + GGX (height-correlated) |

### 各组件数学公式

**GGX NDF:**
```
D_GGX(h, α) = α²
              ─────────────────────────
              π · ((n·h)² · (α²-1) + 1)²

α = roughness²   (Disney 重映射：感知线性)
```

**Schlick-Fresnel:**
```
F_Schlick(cosθ, F0) = F0 + (1 - F0) · (1 - cosθ)⁵

F0 = lerp(0.04, albedo, metallic)   (电介质 F0 ≈ 0.04)
```

**Smith-GGX Geometry:**
```
G_Smith(ωi, ωo) = G1(ωi) · G1(ωo)

G1_SchlickGGX(v, α) =        n · v
                       ────────────────────
                       (n·v) · (1-k) + k

k = (roughness + 1)² / 8   (直接光照)
k = roughness² / 2          (IBL)
```

### 漫反射项

**Lambert:**
```
f_diffuse = albedo / π
```

可选升级：Disney diffuse（更物理准确的掠射角暗化），但 Lambert 足够初期使用。

### 金属与非金属

```
metallic = 0 (电介质):
  kd = 1 - F    → 漫反射为主
  ks = F         → F0 = 0.04（固定）
  albedo 用于漫反射颜色

metallic = 1 (金属):
  kd = 0         → 无漫反射
  ks = F         → F0 = albedo（金属颜色决定反射色）
  albedo 用于镜面反射颜色
```

## 总体策略

```
Phase A: 材质数据结构与加载   (~2 天)  — PBRMaterial + 纹理绑定 + .mtl/.gltf 加载
Phase B: 切线空间与法线贴图   (~1 天)  — tangent 计算 + TBN 矩阵
Phase C: Microfacet BRDF      (~1.5 天) — GGX + Smith + Fresnel shader 实现
Phase D: 纹理采样管线         (~1.5 天) — 多纹理描述符集 + 采样
Phase E: IBL 环境光           (~1 天)  — 预过滤环境贴图 + BRDF LUT（可选）
Phase F: 集成与验证           (~1 天)  — 材质 demo + 参考对比
```

总计约 8 个工作日。

## 不做什么

- 完整的 Disney Principled BRDF（clearcoat / sheen / anisotropy / subsurface / transmission）
- 实时材质编辑器 UI（未来可通过 ImGui 加）
- 虚拟纹理 / 纹理流式加载
- 着色器变体 / uber-shader 编译系统
- 视差贴图 / 置换贴图
- 多层材质 / 材质混合

## 目录结构

```
src/rtr/system/render/
└── pipeline/
    └── srt/
        └── material/
            ├── pbr_material.hpp         # PBR 材质参数定义
            ├── material_instance.hpp     # 运行时材质实例（参数 + 纹理句柄）
            ├── material_defaults.hpp     # 默认 1x1 纹理（白 / 法线蓝 / 黑）
            └── brdf_lut.hpp             # BRDF 积分 LUT（IBL 用）

src/rtr/utils/
├── obj_io.hpp                           # 修改：加载 .mtl 材质
├── obj_types.hpp                        # 修改：顶点增加 tangent
├── gltf_io.hpp                          # 新建：glTF 2.0 加载器（可选）
└── tangent_compute.hpp                  # 新建：MikkTSpace 切线计算

src/rtr/rhi/
├── texture.hpp                          # 现有，无需大改
├── mesh.hpp                             # 修改：顶点属性增加 tangent
└── descriptor.hpp                       # 现有，无需大改

src/rtr/resource/
└── resource_kinds.hpp                   # 修改：新增 MaterialResourceKind

src/rtr/framework/component/material/
└── mesh_component.hpp                   # 修改：增加 material handle

shaders/
├── brdf_common.slang                    # 新建：microfacet BRDF 函数库
├── material_types.slang                 # 新建：材质参数结构定义
└── normal_mapping.slang                 # 新建：TBN + 法线贴图采样
```

## 文件依赖图

```
pbr_material.hpp
  <- material_instance.hpp
       <- material_defaults.hpp
  <- mesh_component.hpp (stores MaterialHandle)
  <- forward_scene_view.hpp / gbuffer_pass (binds material)

tangent_compute.hpp
  <- obj_io.hpp (计算切线后写入顶点)
  <- gltf_io.hpp

brdf_common.slang
  <- material_types.slang
  <- normal_mapping.slang
  <- vert_buffer.slang / gbuffer.slang / pbr_lighting.slang (引用 BRDF)

brdf_lut.hpp
  <- pbr_lighting.slang (IBL 采样)
```
