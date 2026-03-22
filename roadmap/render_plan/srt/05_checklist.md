# SRT 实施检查清单

## Phase 0: Deferred + PBR + Shadow（Day 0-2）

### 前置条件
- [ ] 确认 VK_KHR_dynamic_rendering 支持 MRT（macOS MoltenVK 验证）
- [ ] 确认 Slang 编译器支持 MRT 输出语法

### G-Buffer
- [ ] `pbr_material.hpp` — PBR 材质结构
- [ ] `gbuffer_data.hpp` — 4 个 RT + depth 的创建与管理
- [ ] `gbuffer_pass.hpp` — MRT 动态渲染 pass
- [ ] `gbuffer.slang` — vertex + fragment shader
- [ ] Debug 可视化（每通道独立输出）
- [ ] Motion vector 输出（TAA/temporal reuse 用）

### PBR Lighting
- [ ] `deferred_lighting_pass.hpp` — 全屏 compute lighting
- [ ] `pbr_lighting.slang` — Cook-Torrance BRDF
- [ ] Light buffer SSBO — 不限数量光源
- [ ] `GPULight` 结构支持 6 种光源类型（point/dir/spot/rect/disk/sphere）

### 面光源 (Area Lights)
- [ ] `area_light.hpp` — 面光源定义（center, normal, right, size）
- [ ] `ltc_lut.hpp` — LTC 64×64 查找表加载
- [ ] `ltc.hpp` — LTC 解析着色框架
- [ ] `ltc_area_light.slang` — 矩形面光源 LTC shader
- [ ] `polygon_irradiance()` — 球面多边形积分
- [ ] LTC 集成到 deferred lighting pass
- [ ] G-Buffer emissive 通道（面光源几何体自发光）
- [ ] `gbuffer_emissive.slang` — 自发光着色

### Shadow
- [ ] `shadow_data.hpp` — CSM 数据管理
- [ ] `shadow_pass.hpp` — depth-only pass
- [ ] `shadow.slang` — shadow shader + PCF
- [ ] CSM cascade 之间无跳变

### Pipeline
- [ ] `srt_pipeline.hpp` — 基础骨架可运行

## Phase 1: SDF Infrastructure（Day 3-5）

### 体素化
- [ ] `sdf_builder.hpp` — mesh → SDF 管线
- [ ] `sdf_voxelize.slang` — GPU 三角形体素化
- [ ] `sdf_jfa.slang` — Jump Flooding Algorithm
- [ ] `sdf_sign.slang` — 符号判定
- [ ] SDF slice 可视化验证

### 级联
- [ ] `sdf_cascade.hpp` — 4 级级联管理
- [ ] Toroidal addressing — 增量更新
- [ ] 级联间 SDF 连续性验证

### Ray March
- [ ] `sdf_ray_march.slang` — sphere tracing
- [ ] 软阴影函数
- [ ] Debug 全屏 ray march 可视化
- [ ] 平均步数 < 30

## Phase 2: SDFGI（Day 6-9）

### Probe 系统
- [ ] `sdfgi_probe.hpp` — probe grid + atlas
- [ ] 八面体辐照度存储
- [ ] Probe active/inactive 管理
- [ ] Depth atlas（Chebyshev visibility 用）

### Probe Tracing
- [ ] `sdfgi_trace.hpp` — probe ray tracing dispatch
- [ ] `sdfgi_probe_trace.slang` — SDF march 射线
- [ ] Fibonacci 球面采样 + 帧间旋转
- [ ] 命中点直接光照 + 间接光照采样

### Probe Update
- [ ] `sdfgi_update.hpp` — 辐照度更新
- [ ] `sdfgi_probe_update.slang` — cosine 加权 + EMA
- [ ] Hysteresis 收敛平滑

### GI 采样
- [ ] `sdfgi_sample.hpp` / `sdfgi_sample.slang`
- [ ] 三线性 probe 插值
- [ ] Normal 权重
- [ ] Chebyshev visibility 权重
- [ ] Cornell Box color bleeding 验证

## Phase 3: ReSTIR DI + 面光源采样（Day 10-13）

### Reservoir
- [ ] `reservoir.hpp` — WRS 数据结构（含 sample_point 字段）
- [ ] 双缓冲 reservoir SSBO

### 面光源采样
- [ ] `area_light_sampling.slang` — 面光源表面采样函数
- [ ] `sample_rect_area_light()` — 矩形均匀采样
- [ ] `sample_disk_area_light()` — 圆盘 concentric mapping
- [ ] `sample_sphere_area_light()` — 球面均匀采样
- [ ] `area_light_geometry_term()` — 含光源端 cosine 的 geometry term
- [ ] 面光源 source PDF 正确（1/light_count × 1/area）

### 采样流程
- [ ] `restir_initial.slang` — M=32 候选采样（含面光源面采样）
- [ ] `restir_temporal.slang` — 时间复用 + 几何一致性
- [ ] `restir_spatial.slang` — 空间复用 k=5
- [ ] `restir_shade.slang` — 最终着色 + SDF visibility（使用 sample_point）

### 验证
- [ ] 1000+ 光源（含面光源）不崩溃
- [ ] 面光源产生物理正确的软阴影（半影宽度 ∝ 面积）
- [ ] 大面光源 vs 小面光源软阴影差异明显
- [ ] Temporal reuse 噪点低于无 temporal
- [ ] Spatial reuse 进一步降低噪点
- [ ] 无明显偏差（能量守恒）
- [ ] LTC 与 ReSTIR 协作策略验证

## Phase 4: Post-Processing（Day 13-14）

### Denoise
- [ ] `denoiser.hpp` — SVGF 降噪器
- [ ] `denoise_svgf.slang` — temporal + 5 iter a-trous

### TAA
- [ ] `taa.hpp` + `taa.slang`
- [ ] Halton jitter 在 G-Buffer pass
- [ ] Neighborhood clamping 无 ghosting

### Tone Mapping
- [ ] `tone_mapping.slang` — ACES filmic
- [ ] Exposure 控制

### 集成
- [ ] 完整 SRT Pipeline 端到端
- [ ] 帧时间 < 16.6ms @ 1080p
- [ ] 跨平台验证

## Demo 场景

- [ ] **Cornell Box**：经典 GI 测试（color bleeding）
- [ ] **Cornell Box + 矩形面光源天花板**：面光源 GI + 软阴影
- [ ] **1000 光源场景**（含面光源）：ReSTIR 压力测试
- [ ] **面光源软阴影对比**：不同面积面光源的半影效果对比
- [ ] **室外场景**：SDFGI 级联 + 天空光测试
- [ ] **动态物体**：SDF 更新 + temporal stability
