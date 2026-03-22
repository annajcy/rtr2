# RTR2 软光追升级计划：SDFGI + ReSTIR

## 背景

当前渲染系统：
- Vulkan 后端（1.2/1.3），纯前向渲染
- Blinn-Phong 着色，最多 4 个点光源
- 无阴影、无 GI、无 PBR
- Slang 着色语言 → SPIR-V
- 两条 pipeline：ForwardPipeline / ShaderToyPipeline
- 无硬件光追依赖（不使用 VK_KHR_ray_tracing_pipeline）

## 目标

构建**纯 compute shader 软光追**管线，不依赖硬件 RT core：

1. **SDFGI**（Signed Distance Field Global Illumination）— 基于 SDF 级联体素的间接光照
2. **ReSTIR DI**（Reservoir-based Spatiotemporal Importance Resampling）— 多光源直接光照
3. **面光源**（Area Lights）— 矩形 / 圆盘 / 球体面光源，LTC 解析着色 + ReSTIR 随机采样
4. 可选扩展：ReSTIR GI — 间接光照增强

## 为什么选 SDFGI + ReSTIR

| 方案 | 优点 | 缺点 |
|------|------|------|
| 硬件 RT (VK_KHR_ray_tracing) | 精确、标准 | 需要 RT core，macOS 不支持 |
| Screen-space GI (SSGI/SSAO) | 简单 | 只能看到屏幕内信息 |
| **SDFGI** | 全场景 GI，无需 RT core，compute shader 实现 | SDF 精度有限，动态物体需更新 |
| Light Probes (DDGI) | 效果好 | 需要硬件 RT 或大量 probe |
| **ReSTIR** | 支持数万光源，O(1) 采样 | 需要 G-Buffer |

SDFGI + ReSTIR 是**无硬件 RT 依赖**的最佳组合：
- SDFGI 解决间接光照（漫反射 GI + 粗糙镜面反射）
- ReSTIR 解决多光源直接光照
- 全部 compute shader 实现，跨平台

## 总体策略

```
Phase 0: 基础设施         (~3 天)  — G-Buffer + PBR + 阴影 + 面光源基础
Phase 1: SDF 基础设施     (~3 天)  — 网格 → SDF，级联体素，SDF ray march
Phase 2: SDFGI            (~4 天)  — probe 布局，GI 计算，滤波
Phase 3: ReSTIR DI        (~4 天)  — reservoir 采样，时空复用，面光源采样
Phase 4: 降噪与集成       (~2 天)  — denoiser，TAA，tone mapping
```

总计约 16 个工作日。

## 不做什么

- 硬件光追 (VK_KHR_ray_tracing_pipeline)
- 体积光 / 体积雾（可后续扩展）
- 大规模动态场景 SDF 增量更新
- 完整的 subsurface scattering
- 生产级 LOD / 虚拟纹理

## 架构总览

```
                    ┌─────────────┐
                    │  Scene Data │
                    └──────┬──────┘
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
        ┌──────────┐ ┌──────────┐ ┌──────────┐
        │ G-Buffer │ │   SDF    │ │  Light   │
        │   Pass   │ │ Builder  │ │  Buffer  │
        └────┬─────┘ └────┬─────┘ └────┬─────┘
             │            │            │
             ▼            ▼            │
        ┌──────────┐ ┌──────────┐     │
        │  Shadow  │ │  SDFGI   │     │
        │   Pass   │ │  Probes  │     │
        └────┬─────┘ └────┬─────┘     │
             │            │            │
             └────────────┼────────────┘
                          ▼
                   ┌─────────────┐
                   │  ReSTIR DI  │
                   │  Sampling   │
                   └──────┬──────┘
                          ▼
                   ┌─────────────┐
                   │  Composite  │
                   │  + Denoise  │
                   └──────┬──────┘
                          ▼
                   ┌─────────────┐
                   │  TAA + Tone │
                   │   Mapping   │
                   └──────┬──────┘
                          ▼
                      Present
```

## 目录结构

```
src/rtr/system/render/
├── pipeline/
│   ├── forward/              # 现有
│   ├── shadertoy/            # 现有
│   └── srt/                  # 新建：软光追管线
│       ├── srt_pipeline.hpp          # 主管线编排
│       ├── gbuffer/
│       │   ├── gbuffer_pass.hpp      # G-Buffer 生成
│       │   └── gbuffer_data.hpp      # G-Buffer 纹理布局
│       ├── shadow/
│       │   ├── shadow_pass.hpp       # 阴影贴图
│       │   └── shadow_data.hpp       # 级联阴影数据
│       ├── sdf/
│       │   ├── sdf_builder.hpp       # 网格 → SDF 体素化
│       │   ├── sdf_cascade.hpp       # 级联 SDF 管理
│       │   └── sdf_ray_march.hpp     # SDF ray marching compute
│       ├── sdfgi/
│       │   ├── sdfgi_probe.hpp       # probe 布局与数据
│       │   ├── sdfgi_trace.hpp       # probe ray tracing via SDF
│       │   ├── sdfgi_update.hpp      # probe 辐照度更新
│       │   └── sdfgi_sample.hpp      # 从 probe 采样 GI
│       ├── light/
│       │   ├── area_light.hpp        # 面光源定义与采样
│       │   ├── ltc.hpp               # LTC 解析面光源着色
│       │   └── ltc_lut.hpp           # LTC 预计算查找表
│       ├── restir/
│       │   ├── reservoir.hpp         # reservoir 数据结构
│       │   ├── restir_di.hpp         # 直接光照 ReSTIR
│       │   ├── restir_spatial.hpp    # 空间复用
│       │   └── restir_temporal.hpp   # 时间复用
│       ├── post/
│       │   ├── denoiser.hpp          # SVGF / A-trous 降噪
│       │   ├── taa.hpp               # 时间抗锯齿
│       │   └── tone_mapping.hpp      # ACES / Filmic
│       └── material/
│           └── pbr_material.hpp      # PBR 材质定义

shaders/srt/
├── gbuffer.slang
├── gbuffer_emissive.slang
├── shadow.slang
├── ltc_area_light.slang
├── sdf_voxelize.slang
├── sdf_ray_march.slang
├── sdfgi_probe_trace.slang
├── sdfgi_probe_update.slang
├── sdfgi_sample.slang
├── restir_initial.slang
├── restir_temporal.slang
├── restir_spatial.slang
├── restir_shade.slang
├── denoise_svgf.slang
├── taa.slang
└── tone_mapping.slang
```

## 文件依赖图

```
pbr_material.hpp
  <- gbuffer_data.hpp
       <- gbuffer_pass.hpp
            <- shadow_pass.hpp
                 <- srt_pipeline.hpp

sdf_cascade.hpp
  <- sdf_builder.hpp
  <- sdf_ray_march.hpp
       <- sdfgi_trace.hpp
            <- sdfgi_probe.hpp
            <- sdfgi_update.hpp
                 <- sdfgi_sample.hpp
                      <- srt_pipeline.hpp

area_light.hpp
  <- ltc.hpp
       <- ltc_lut.hpp

reservoir.hpp
  <- restir_di.hpp (uses area_light.hpp for sampling)
       <- restir_temporal.hpp
       <- restir_spatial.hpp
            <- srt_pipeline.hpp

denoiser.hpp <- taa.hpp <- tone_mapping.hpp <- srt_pipeline.hpp
```

## 成功标准

- G-Buffer 正确输出 albedo / normal / depth / roughness / metallic
- SDF 体素化结果与原始几何一致（视觉对比）
- SDFGI probe 在 Cornell Box 场景中产生可见的颜色渗透（color bleeding）
- 面光源（矩形 / 圆盘）LTC 解析着色无噪点，视觉正确
- 面光源在 ReSTIR 中正确采样，产生物理正确的软阴影
- ReSTIR 在 1000+ 光源（含面光源）场景中保持 60fps@1080p
- 整体管线在 Cornell Box + 多光源 + 面光源场景中视觉正确
- macOS (MoltenVK) 与 Linux/Windows 均可运行
