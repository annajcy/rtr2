# 材质系统实施检查清单

## Phase A: 材质数据结构与加载（Day 0-1）

### 数据结构
- [ ] `PBRMaterialParams` — base_color, metallic, roughness, ao, emissive, ior
- [ ] `MaterialTextureSlot` — 5 个纹理槽位枚举
- [ ] `MaterialInstance` — 参数 + 纹理句柄管理
- [ ] `GPUMaterialParams` — std140 对齐的 GPU 结构
- [ ] `MaterialDefaults` — 4 个 1×1 默认纹理（白/蓝法线/黑/默认MR）

### 加载
- [ ] OBJ .mtl 材质参数提取（Kd, Ks, Ns, Ke, 纹理路径）
- [ ] Phong → PBR 参数转换（shininess → roughness, specular → metallic）
- [ ] 纹理加载色空间正确（albedo/emissive = sRGB, 其余 = linear）
- [ ] `MaterialResourceKind` 注册到 ResourceManager
- [ ] `MeshComponent` 持有 `MaterialHandle`

### 描述符
- [ ] 描述符集分层设计（Set 0 frame / Set 1 material / Set 2 object）

## Phase B: 切线空间（Day 2）

### 顶点格式
- [ ] `ObjVertex` 扩展：新增 `Vec4 tangent`（48 bytes/vertex）
- [ ] 顶点属性描述新增 location 3（R32G32B32A32Sfloat）

### 切线计算
- [ ] MikkTSpace 集成 + `compute_tangents()` 函数
- [ ] Lengyel fallback 处理退化 UV
- [ ] `load_obj()` 自动计算 tangent
- [ ] tangent.w handedness 正确（镜像 UV 为 -1）

### Shader
- [ ] `normal_mapping.slang` — TBN 矩阵构建
- [ ] `get_normal_from_map()` — 法线贴图采样 + 空间变换
- [ ] Gram-Schmidt 正交化
- [ ] 默认法线纹理 (0.5, 0.5, 1.0) → 无扰动回退正确

## Phase C: Microfacet BRDF（Day 2-3）

### BRDF 组件
- [ ] `brdf_common.slang` — 函数库文件
- [ ] `D_GGX()` — GGX Normal Distribution Function
- [ ] `G_Smith()` — Smith Geometry Function（直接光照 k 值）
- [ ] `F_Schlick()` — Fresnel-Schlick 近似
- [ ] `evaluate_brdf()` — 完整 Cook-Torrance 组装

### 正确性
- [ ] roughness → alpha 重映射（α = roughness²）
- [ ] F0 = lerp(0.04, base_color, metallic) 正确
- [ ] metallic=1 时 kd=0（无漫反射）
- [ ] 漫反射项 = kd * base_color / π
- [ ] 能量守恒：白炉测试通过
- [ ] roughness=0 无 NaN / Inf（EPSILON 保护）

### 集成
- [ ] `material_types.slang` — GPU 材质参数结构
- [ ] `vert_buffer.slang` 替换 Blinn-Phong 为 Cook-Torrance
- [ ] 材质球阵列 5×5 渲染验证

## Phase D: 纹理采样管线（Day 3-4）

### 描述符绑定
- [ ] `MaterialDescriptorManager` — per-material 描述符集创建
- [ ] Material UBO (binding 0) 正确绑定
- [ ] 5 个纹理 (binding 1-5) 正确绑定
- [ ] 无纹理时绑定默认纹理
- [ ] Pipeline layout 支持 3 个描述符集

### Shader 采样
- [ ] `sample_material()` — albedo × factor
- [ ] MetallicRoughness: G=roughness, B=metallic (glTF 约定)
- [ ] AO: R 通道 × ao_strength
- [ ] `sample_emissive()` — emissive × factor × strength
- [ ] Alpha test — discard when alpha < cutoff
- [ ] roughness 下限 clamp 0.04

### 渲染优化
- [ ] Renderable 按材质排序
- [ ] 描述符集切换最小化

## Phase E: IBL 环境光（Day 5）

### 预计算
- [ ] HDR equirectangular 加载
- [ ] Equirectangular → Cubemap 转换 (compute shader)
- [ ] Irradiance cubemap 预卷积（32×32 per face）
- [ ] Pre-filtered env map（128×128, 5 mip levels）
- [ ] BRDF Integration LUT（512×512 RG16F）

### 着色
- [ ] `evaluate_ibl()` — 漫反射 IBL + 镜面 IBL
- [ ] `F_SchlickRoughness()` — 粗糙度感知 Fresnel
- [ ] 镜面 IBL: mip level 由 roughness 选择
- [ ] AO 正确影响环境光项

### 天空
- [ ] Skybox 渲染（cubemap 采样）

## Phase F: 集成与验证（Day 6-7）

### 验证场景
- [ ] 材质球阵列 5×5 视觉正确
- [ ] 5 种纹理类型全部采样验证
- [ ] glTF 测试模型（DamagedHelmet / FlightHelmet）
- [ ] 白炉测试（能量守恒）
- [ ] 边界条件无伪影
- [ ] 掠射角 Fresnel 效应

### 集成
- [ ] Forward pipeline 使用新材质系统
- [ ] G-Buffer pass 输出完整材质参数
- [ ] SRT pipeline 对接验证

## 与 SRT Plan 的交叉依赖

```
材质系统 Phase A-D  →  SRT Phase 0 (G-Buffer + PBR lighting)
材质系统 Phase C    →  SRT Phase 2 (SDFGI probe 命中点光照评估)
材质系统 Phase C    →  SRT Phase 3 (ReSTIR shade 完整 BRDF)
材质系统 Phase B    →  SRT Phase 0 (G-Buffer 需要 TBN 输出 world normal)
```

**推荐执行顺序**：先完成材质系统 Phase A-D，再开始 SRT Phase 0。
IBL (Phase E) 可与 SRT 并行，或在 SDFGI 完成后降级为天空光。
