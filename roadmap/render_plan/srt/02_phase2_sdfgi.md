# Phase 2: SDFGI — SDF 全局光照（~4 天）

## 目标

基于 Phase 1 的级联 SDF，实现 probe-based 全局光照系统：
- 漫反射 GI（color bleeding）
- 粗糙镜面反射（glossy reflection via SDF）
- 多次弹射近似

## 核心算法概述

SDFGI 的核心思路（参考 Godot 4）：

```
1. 在级联体素中均匀放置 light probes
2. 每帧从每个 probe 发射少量射线（用 SDF ray march）
3. 射线命中后，从 G-Buffer/材质获取命中点的颜色
4. 更新 probe 的辐照度球谐（SH）/ 八面体编码
5. 渲染时，每个像素从附近 probes 插值获取间接光照
```

关键创新点：**用 SDF ray march 代替硬件光追做 probe tracing**，这是"软光追"的核心。

## Day 6-7: Probe 系统

### Probe 布局

```
每个级联体素中放置一个 probe
Cascade 0: 128³ probes, 间距 = voxel_size_0 (~6.25cm)
Cascade 1: 128³ probes, 间距 = voxel_size_1 (~25cm)
...

优化：不是每个体素都需要 probe
- 只在表面附近（|SDF| < threshold）放置 active probes
- 用 indirect dispatch 只处理 active probes
```

### 辐照度存储

两种主流方案：

| 方案 | 存储 | 精度 | 采样速度 |
|------|------|------|----------|
| SH (L2) | 9 × float3 = 108 bytes/probe | 低频好 | 快 |
| **八面体编码** | 8×8 texel octahedral map | 方向分辨率高 | 需要纹理采样 |

选择**八面体编码**（Octahedral encoding）：
- 每个 probe 存储一个 8×8 的辐照度图（八面体映射）
- 用 3D texture atlas 打包所有 probe 的辐照度图
- 支持硬件纹理过滤

### 任务

1. **Probe 数据结构** — `sdfgi_probe.hpp`
   ```cpp
   struct SDFGIProbeGrid {
       // 每个 cascade 的 probe atlas
       // Atlas 布局: (probe_x * PROBE_SIZE, probe_y * PROBE_SIZE)
       // 第三维用 3D texture 的 z 轴
       vk::raii::Image     irradiance_atlas;   // RGBA16F
       vk::raii::Image     depth_atlas;        // RG16F (mean, variance)
       vk::raii::ImageView irradiance_view;
       vk::raii::ImageView depth_view;

       static constexpr int PROBE_TEXELS = 8;  // 每个 probe 的八面体分辨率
       // 加 1 texel border 用于过滤 → 实际 10×10

       glm::ivec3 probe_count;   // 每轴 probe 数量
       float      probe_spacing; // probe 间距
       glm::vec3  grid_origin;   // grid 世界坐标原点
   };
   ```

2. **Probe 状态管理**
   - Active/inactive 标记（基于 SDF 距离）
   - Probe age tracking（新激活的 probe 需要更多射线初始化）
   - Hysteresis：probe 不因相机移动频繁开关

## Day 7-8: Probe Ray Tracing

### 射线策略

每帧每个 probe 发射 **64-128 条射线**（分摊到多帧）：
- 方向：均匀球面采样（Fibonacci 球面 + 帧间旋转）
- 用 SDF ray march（Phase 1）执行 tracing
- 命中后采样该点的直接光照（来自 Phase 0 lighting pass 或 light buffer）

### 多弹射近似

```
Bounce 0: probe 射线命中 → 采样命中点的直接光照
Bounce 1: 命中点再查询最近 probe 的辐照度 → 间接光照
```

不需要递归 tracing：第 N 帧的 probe 辐照度自然包含了 N-1 帧的间接光照信息。随时间收敛到多弹射。

### 任务

1. **Probe Trace Shader** — `shaders/srt/sdfgi_probe_trace.slang`
   ```slang
   // 每个线程处理一个 probe 的一条射线
   // workgroup size: (RAYS_PER_BATCH, 1, 1)
   // dispatch: (active_probe_count, 1, 1)

   [numthreads(64, 1, 1)]
   void main(uint3 tid : SV_DispatchThreadID) {
       uint probe_idx = tid.y;
       uint ray_idx = tid.x;

       ProbeInfo probe = load_probe(probe_idx);
       float3 ray_dir = fibonacci_sphere(ray_idx + frame_offset, TOTAL_RAYS);

       SDFHit hit = sdf_ray_march(probe.position, ray_dir, MAX_TRACE_DIST);

       float3 radiance = float3(0);
       if (hit.hit) {
           // 采样命中点的直接光照
           radiance = evaluate_direct_lighting(hit.position, hit.normal);

           // 采样命中点附近 probe 的辐照度（多弹射）
           radiance += sample_probe_irradiance(hit.position, hit.normal);
       } else {
           // 天空光
           radiance = sample_sky(ray_dir);
       }

       // 写入 probe 射线结果缓冲
       ray_results[probe_idx * RAYS_PER_BATCH + ray_idx] = float4(radiance, hit.distance);
   }
   ```

2. **命中点光照评估**
   - 快速版：查询 light buffer，计算命中点的 Lambert 漫反射
   - 不需要完整 PBR（probe GI 主要贡献低频漫反射）
   - **面光源处理**：probe 射线直接命中面光源几何体时，返回面光源的 emission
     - 在 SDF 中，面光源几何体标记为 emissive（SDF 值 ≤ 0 的区域带 emission 属性）
     - 或者：维护一个 emissive voxel atlas，体素化时同时写入 emission 信息
     - 这是面光源在 GI 中产生 color bleeding 的关键路径
   - 可选：从上一帧的 GI buffer 获取 indirect（多弹射递归的关键）

## Day 8-9: Probe 更新与采样

### Probe 辐照度更新

1. **Probe Update Shader** — `shaders/srt/sdfgi_probe_update.slang`
   ```slang
   // 每个线程处理一个 probe atlas 的一个 texel
   // 输入: ray_results buffer
   // 输出: irradiance_atlas (八面体映射)

   [numthreads(PROBE_TEXELS+2, PROBE_TEXELS+2, 1)]
   void main(uint3 tid : SV_DispatchThreadID) {
       uint2 texel = tid.xy;
       uint probe_idx = tid.z;

       float3 texel_dir = octahedral_decode(texel_to_uv(texel));

       // 加权平均所有射线对这个方向的贡献
       float3 irradiance = float3(0);
       float total_weight = 0;

       for (int r = 0; r < RAYS_PER_BATCH; r++) {
           float3 ray_dir = fibonacci_sphere(r + frame_offset, TOTAL_RAYS);
           float weight = max(0, dot(ray_dir, texel_dir));  // cosine 权重

           float4 ray_result = ray_results[probe_idx * RAYS_PER_BATCH + r];
           irradiance += ray_result.rgb * weight;
           total_weight += weight;
       }

       if (total_weight > 0)
           irradiance /= total_weight;

       // 指数移动平均，避免闪烁
       float3 prev = load_probe_texel(probe_idx, texel);
       float hysteresis = 0.97;  // 新 probe 用更低值加速收敛
       irradiance = lerp(irradiance, prev, hysteresis);

       store_probe_texel(probe_idx, texel, irradiance);
   }
   ```

2. **深度更新**（Chebyshev visibility test 用）
   - 同步更新 depth atlas：存储命中距离的 mean 和 mean²（用于 variance）
   - 采样时做 Chebyshev 可见性测试，避免 light leaking

### 从 Probe 采样 GI

3. **GI 采样 Shader** — `shaders/srt/sdfgi_sample.slang`
   ```slang
   // 全屏 compute shader，逐像素采样间接光照
   float3 sample_sdfgi(float3 world_pos, float3 normal) {
       // 1. 确定所在级联
       int cascade = select_cascade(world_pos);

       // 2. 找到周围 8 个 probe（三线性插值）
       int3 base_probe = floor((world_pos - grid_origin) / probe_spacing);

       float3 gi = float3(0);
       float total_weight = 0;

       for (int i = 0; i < 8; i++) {  // 2×2×2 邻近 probe
           int3 offset = int3(i&1, (i>>1)&1, (i>>2)&1);
           int3 probe_coord = base_probe + offset;

           float3 probe_pos = grid_origin + float3(probe_coord) * probe_spacing;

           // Trilinear 权重
           float3 alpha = frac((world_pos - grid_origin) / probe_spacing);
           float w = lerp3(1-alpha, alpha, offset);

           // Normal 权重：背面 probe 权重降低
           float3 dir_to_probe = normalize(probe_pos - world_pos);
           w *= max(0.0001, dot(normal, dir_to_probe));

           // Chebyshev visibility 权重（防 light leak）
           w *= chebyshev_test(probe_coord, world_pos);

           // 采样 probe 辐照度
           float2 uv = octahedral_encode(normal);
           float3 probe_irradiance = sample_probe_atlas(probe_coord, uv);

           gi += probe_irradiance * w;
           total_weight += w;
       }

       return gi / max(total_weight, 0.0001);
   }
   ```

### 集成到 SRT Pipeline

```cpp
void SRTPipeline::render(FrameTicket& ticket) {
    shadow_pass_.execute(ticket);
    gbuffer_pass_.execute(ticket);
    sdfgi_trace_.execute(ticket);     // probe ray tracing
    sdfgi_update_.execute(ticket);    // probe irradiance 更新
    lighting_pass_.execute(ticket);   // 直接光照 + 采样 SDFGI 间接光照
    present_pass_.execute(ticket);
}
```

## Day 9: 调试与调优

### 常见问题及解决

| 问题 | 原因 | 解决 |
|------|------|------|
| Light leaking | 墙后亮光穿透 | Chebyshev visibility test / 增大 normal bias |
| 闪烁 | hysteresis 太低 | 提高 hysteresis（0.95-0.98）|
| 暗角 | probe 密度不够 | 降低级联范围 / 提高分辨率 |
| 色偏 | 某些方向射线不够 | 增加 rays_per_batch / 用更好的采样序列 |
| 能量不守恒 | 采样未除以 PDF | 确保 cosine 加权正确 |

### Debug 可视化

- Probe 位置绘制（小球体）
- Probe 辐照度球（每个 probe 渲染八面体 unwrap）
- GI only 通道（关闭直接光照）
- Probe trace ray 可视化（line rendering）

## Phase 2 完成标准

- [ ] Probe 系统正确布局在级联体素中
- [ ] Probe ray tracing 通过 SDF ray march 执行
- [ ] Probe 辐照度八面体图正确更新
- [ ] 多弹射通过帧间递归自然收敛
- [ ] Cornell Box 中可见明显的 color bleeding
- [ ] Chebyshev visibility test 有效减少 light leaking
- [ ] GI 贡献在 1-2 秒内视觉收敛
- [ ] 性能：SDFGI 全流程 < 4ms @ 1080p

## 关键参考

- Juan Linietsky, "SDFGI in Godot 4" — 整体架构参考
- "Dynamic Diffuse Global Illumination with Ray-Traced Irradiance Fields" (Majercik et al. 2019) — DDGI probe 更新
- "Octahedral Normal Vector Encoding" — 八面体编码
- "Scalable Real-Time Global Illumination for Large Scenes" (Crassin et al.) — 级联设计
