# Phase 3: ReSTIR DI — 多光源直接光照 + 面光源采样（~4 天）

## 目标

实现 ReSTIR (Reservoir-based Spatiotemporal Importance Resampling) 直接光照：
- 支持数千到数万个光源（含面光源）
- O(1) per-pixel 采样复杂度
- 时间和空间复用提高采样质量
- 面光源通过随机面采样产生物理正确的软阴影

## 核心算法

### 为什么需要 ReSTIR

传统直接光照：每像素遍历所有光源 → O(N) 复杂度，N = 1000 时不可行。

ReSTIR 核心思想：
1. 每像素维护一个 **reservoir**（蓄水池采样器）
2. 通过 Weighted Reservoir Sampling (WRS) 从所有光源中采样 1 个
3. 通过**时间复用**（temporal reuse）和**空间复用**（spatial reuse）有效增加采样数
4. 最终每像素只评估 1 个光源的完整 BRDF + visibility

### 算法流程

```
Per-pixel pipeline:
1. Initial Candidates:   从 light buffer 随机抽 M 个候选光源 (M ≈ 32)
                          用 target PDF ∝ Le * G * f_brdf 做 WRS → 选出 1 个
2. Temporal Reuse:        与上一帧同像素的 reservoir 合并
3. Spatial Reuse:         与邻近像素的 reservoir 合并 (k ≈ 5 个邻居)
4. Shade:                 用最终选中的光源计算完整光照
```

## Day 10: Reservoir 数据结构 + 面光源采样

### 面光源与 ReSTIR 的集成策略

面光源在 ReSTIR 中的处理核心：**将面光源的面积采样转化为点样本**。

```
面光源 → 在光源表面随机采样一个点 → 变成一个"虚拟点光源"
                                    → 进入标准 ReSTIR reservoir 流程
```

这意味着每次评估面光源时，需要：
1. 在面光源表面采样一个点 (sample_point)
2. 计算该点的 radiance 和 PDF
3. 将 (light_index, sample_point) 作为一个完整的样本存入 reservoir

### 任务

1. **Reservoir** — `reservoir.hpp` + buffer 定义
   ```cpp
   struct Reservoir {
       uint32_t light_index;    // 当前选中的光源
       float3   sample_point;   // 面光源：采样点位置；点光源：= light.position
       float    weight_sum;     // 权重累计和 (w_sum)
       uint32_t sample_count;   // 已见样本数 (M)
       float    W;              // 1 / (p_hat(y) * M) * w_sum — 无偏权重
   };
   ```

   > **关键变化**：相比纯点光源版本，新增 `sample_point` 字段。面光源每次被选中时，对应的采样点不同，所以 reservoir 必须同时记录采样点位置。

2. **Reservoir Buffer** — 每帧两个 buffer（current + previous）
   ```cpp
   // SSBO: width × height × sizeof(Reservoir)
   // 双缓冲用于时间复用
   Buffer reservoir_current_;
   Buffer reservoir_previous_;
   ```

3. **Reservoir GLSL/Slang 工具函数**
   ```slang
   // Weighted Reservoir Sampling
   bool reservoir_update(inout Reservoir r, uint light_idx, float weight, inout uint rng_state) {
       r.weight_sum += weight;
       r.sample_count += 1;
       if (random_float(rng_state) < weight / r.weight_sum) {
           r.light_index = light_idx;
           return true;
       }
       return false;
   }

   // 合并两个 reservoir
   bool reservoir_merge(inout Reservoir dst, Reservoir src, float target_pdf_at_src_sample, inout uint rng_state) {
       float weight = target_pdf_at_src_sample * src.W * src.sample_count;
       return reservoir_update(dst, src.light_index, weight, rng_state);
   }
   ```

## Day 10-11: 面光源采样函数 + Initial Sampling

### 面光源表面采样

在 ReSTIR 候选生成之前，需要定义面光源的采样和 PDF 计算函数。

**面光源采样工具** — `shaders/srt/area_light_sampling.slang`
```slang
struct LightSample {
    float3 point;     // 面光源表面采样点（世界空间）
    float3 normal;    // 采样点法线
    float3 emission;  // 采样点辐射度
    float  pdf;       // 采样概率密度 (1/area for uniform)
};

// 矩形面光源均匀采样
LightSample sample_rect_area_light(GPULight light, float2 xi) {
    LightSample s;
    float half_w = light.params.x;
    float half_h = light.params.y;
    float3 right = light.right.xyz;
    float3 up = cross(light.direction.xyz, right);

    // xi ∈ [0,1)² → 矩形表面上的均匀点
    float u = (xi.x - 0.5) * 2.0 * half_w;
    float v = (xi.y - 0.5) * 2.0 * half_h;
    s.point = light.position.xyz + right * u + up * v;
    s.normal = light.direction.xyz;
    s.emission = light.color.xyz * light.color.w;
    s.pdf = 1.0 / (4.0 * half_w * half_h);  // 1 / area
    return s;
}

// 圆盘面光源均匀采样（concentric mapping）
LightSample sample_disk_area_light(GPULight light, float2 xi) {
    LightSample s;
    float radius = light.params.x;
    float3 right = light.right.xyz;
    float3 up = cross(light.direction.xyz, right);

    // Concentric disk mapping (Shirley)
    float2 disk = concentric_sample_disk(xi) * radius;
    s.point = light.position.xyz + right * disk.x + up * disk.y;
    s.normal = light.direction.xyz;
    s.emission = light.color.xyz * light.color.w;
    s.pdf = 1.0 / (PI * radius * radius);
    return s;
}

// 球形面光源均匀采样
LightSample sample_sphere_area_light(GPULight light, float2 xi) {
    LightSample s;
    float radius = light.params.x;

    // 均匀球面采样
    float z = 1.0 - 2.0 * xi.x;
    float r = sqrt(max(0, 1.0 - z * z));
    float phi = 2.0 * PI * xi.y;
    float3 local_normal = float3(r * cos(phi), r * sin(phi), z);

    s.point = light.position.xyz + local_normal * radius;
    s.normal = local_normal;
    s.emission = light.color.xyz * light.color.w;
    s.pdf = 1.0 / (4.0 * PI * radius * radius);
    return s;
}

// 统一采样入口
LightSample sample_light_surface(GPULight light, float2 xi) {
    uint type = uint(light.position.w);
    if (type == 3) return sample_rect_area_light(light, xi);
    if (type == 4) return sample_disk_area_light(light, xi);
    if (type == 5) return sample_sphere_area_light(light, xi);

    // 点/方向/聚光灯：退化为点采样
    LightSample s;
    s.point = light.position.xyz;
    s.normal = -normalize(light.position.xyz);  // placeholder
    s.emission = light.color.xyz * light.color.w;
    s.pdf = 1.0;  // delta distribution
    return s;
}

// 面光源 → 着色点的 geometry term
// 包含面光源法线的 cosine 衰减
float area_light_geometry_term(float3 shading_pos, float3 shading_normal,
                                LightSample ls) {
    float3 wi = ls.point - shading_pos;
    float dist2 = dot(wi, wi);
    wi = normalize(wi);

    float cos_shading = max(0, dot(shading_normal, wi));
    float cos_light = max(0, dot(ls.normal, -wi));  // 面光源朝向着色点

    return cos_shading * cos_light / dist2;
}
```

### Initial Candidate Generation（含面光源）

1. **Initial Sampling Shader** — `shaders/srt/restir_initial.slang`
   ```slang
   [numthreads(8, 8, 1)]
   void main(uint3 tid : SV_DispatchThreadID) {
       uint2 pixel = tid.xy;
       if (any(pixel >= resolution)) return;

       // 从 G-Buffer 读取几何信息
       float3 pos = load_gbuffer_position(pixel);
       float3 normal = load_gbuffer_normal(pixel);
       float3 albedo = load_gbuffer_albedo(pixel);
       float roughness = load_gbuffer_roughness(pixel);
       float metallic = load_gbuffer_metallic(pixel);

       uint rng = init_rng(pixel, frame_count);
       Reservoir r = reservoir_empty();

       // 从所有光源中随机抽 M 个候选
       uint M = min(32, light_count);
       for (uint i = 0; i < M; i++) {
           uint light_idx = random_uint(rng) % light_count;
           GPULight light = lights[light_idx];

           // 在光源表面采样一个点
           float2 xi = float2(random_float(rng), random_float(rng));
           LightSample ls = sample_light_surface(light, xi);

           // target PDF: p_hat ∝ Le * geometry_term * brdf_approx
           float3 wi = normalize(ls.point - pos);
           float NdotL = max(0, dot(normal, wi));
           float dist2 = dot(ls.point - pos, ls.point - pos);

           // 面光源的 geometry term 包含光源端 cosine
           float cos_light = is_area_light(light)
               ? max(0, dot(ls.normal, -wi))
               : 1.0;

           float3 Le = ls.emission;
           float brdf_approx = NdotL;
           float p_hat = luminance(Le) * brdf_approx * cos_light / dist2;

           // source PDF: 均匀选光源 × 面采样 PDF
           float p_source = (1.0 / float(light_count)) * ls.pdf;
           float weight = (p_source > 0) ? p_hat / p_source : 0;

           reservoir_update(r, light_idx, ls.point, weight, rng);
       }

       // 计算 W
       float p_hat_selected = evaluate_target_pdf(r.light_index, r.sample_point, pos, normal);
       r.W = (p_hat_selected > 0) ? (r.weight_sum / (r.sample_count * p_hat_selected)) : 0;

       store_reservoir(pixel, r);
   }
   ```

### Temporal Reuse

2. **Temporal Reuse Shader** — `shaders/srt/restir_temporal.slang`
   ```slang
   [numthreads(8, 8, 1)]
   void main(uint3 tid : SV_DispatchThreadID) {
       uint2 pixel = tid.xy;

       Reservoir current = load_reservoir_current(pixel);
       float3 pos = load_gbuffer_position(pixel);
       float3 normal = load_gbuffer_normal(pixel);

       // Motion vector → 找到上一帧对应像素
       float2 motion = load_motion_vector(pixel);
       int2 prev_pixel = int2(pixel) + int2(round(motion));

       if (all(prev_pixel >= 0) && all(prev_pixel < resolution)) {
           // 几何一致性检查
           float3 prev_pos = load_prev_gbuffer_position(prev_pixel);
           float3 prev_normal = load_prev_gbuffer_normal(prev_pixel);

           bool geometry_similar =
               length(prev_pos - pos) < 0.1 &&
               dot(prev_normal, normal) > 0.9;

           if (geometry_similar) {
               Reservoir prev = load_reservoir_previous(prev_pixel);
               // 限制历史帧权重（防止 stale sample）
               prev.sample_count = min(prev.sample_count, 20 * current.sample_count);

               // 合并
               uint rng = init_rng(pixel, frame_count);
               Reservoir combined = reservoir_empty();
               float p_hat_curr = evaluate_target_pdf(current.light_index, pos, normal);
               float p_hat_prev = evaluate_target_pdf(prev.light_index, pos, normal);

               reservoir_merge(combined, current, p_hat_curr, rng);
               reservoir_merge(combined, prev, p_hat_prev, rng);

               float p_hat_final = evaluate_target_pdf(combined.light_index, pos, normal);
               combined.W = (p_hat_final > 0)
                   ? combined.weight_sum / (combined.sample_count * p_hat_final)
                   : 0;

               store_reservoir(pixel, combined);
               return;
           }
       }

       store_reservoir(pixel, current);
   }
   ```

### Motion Vector

需要在 G-Buffer pass 中额外输出 motion vector：
- 当前帧 clip space position - 上一帧 clip space position
- 用于 temporal reuse 的像素对应和 TAA

## Day 11-12: Spatial Reuse + Final Shading

### Spatial Reuse

1. **Spatial Reuse Shader** — `shaders/srt/restir_spatial.slang`
   ```slang
   [numthreads(8, 8, 1)]
   void main(uint3 tid : SV_DispatchThreadID) {
       uint2 pixel = tid.xy;

       Reservoir center = load_reservoir(pixel);
       float3 pos = load_gbuffer_position(pixel);
       float3 normal = load_gbuffer_normal(pixel);
       float depth = load_gbuffer_depth(pixel);

       uint rng = init_rng(pixel, frame_count + 1);
       Reservoir combined = reservoir_empty();

       // 先加入自己
       float p_hat_self = evaluate_target_pdf(center.light_index, pos, normal);
       reservoir_merge(combined, center, p_hat_self, rng);

       // 随机选 k 个邻居
       uint k = 5;
       float search_radius = 30.0;  // pixels

       for (uint i = 0; i < k; i++) {
           float angle = random_float(rng) * 2.0 * PI;
           float radius = random_float(rng) * search_radius;
           int2 neighbor = int2(pixel) + int2(cos(angle) * radius, sin(angle) * radius);

           if (any(neighbor < 0) || any(neighbor >= resolution)) continue;

           // 几何相似性检查（防止跨表面复用）
           float3 n_pos = load_gbuffer_position(neighbor);
           float3 n_normal = load_gbuffer_normal(neighbor);
           float n_depth = load_gbuffer_depth(neighbor);

           if (abs(n_depth - depth) / depth > 0.1) continue;
           if (dot(n_normal, normal) < 0.9) continue;

           Reservoir n_reservoir = load_reservoir(neighbor);
           float p_hat_n = evaluate_target_pdf(n_reservoir.light_index, pos, normal);
           reservoir_merge(combined, n_reservoir, p_hat_n, rng);
       }

       float p_hat_final = evaluate_target_pdf(combined.light_index, pos, normal);
       combined.W = (p_hat_final > 0)
           ? combined.weight_sum / (combined.sample_count * p_hat_final)
           : 0;

       store_reservoir(pixel, combined);
   }
   ```

### Final Shading

2. **Shade Shader** — `shaders/srt/restir_shade.slang`
   ```slang
   [numthreads(8, 8, 1)]
   void main(uint3 tid : SV_DispatchThreadID) {
       uint2 pixel = tid.xy;

       Reservoir r = load_reservoir(pixel);
       float3 pos = load_gbuffer_position(pixel);
       float3 normal = load_gbuffer_normal(pixel);
       float3 albedo = load_gbuffer_albedo(pixel);
       float roughness = load_gbuffer_roughness(pixel);
       float metallic = load_gbuffer_metallic(pixel);
       float3 V = normalize(camera_pos - pos);

       float3 direct = float3(0);

       if (r.light_index != INVALID_LIGHT && r.W > 0) {
           GPULight light = lights[r.light_index];

           // 关键：使用 reservoir 中存储的 sample_point，而非光源中心
           // 对面光源，这是表面上的随机采样点
           // 对点光源，这就是光源位置
           float3 light_pos = r.sample_point;
           float3 to_light = light_pos - pos;
           float dist2 = dot(to_light, to_light);
           float3 L = normalize(to_light);

           float3 Le = light.color.xyz * light.color.w;

           // 面光源的光源端 cosine 衰减
           float cos_light = 1.0;
           if (is_area_light(light)) {
               cos_light = max(0, dot(light.direction.xyz, -L));
               // 背面采样点贡献为 0
               if (cos_light <= 0) { store_hdr_color(pixel, float3(0)); return; }
           }

           // 可见性：SDF ray march 到采样点
           // 面光源的不同采样点产生不同的遮挡 → 自然形成软阴影
           float visibility = sdf_ray_march_visibility(
               pos + normal * 0.01, L, sqrt(dist2) - 0.02);

           // 完整 PBR BRDF
           float3 brdf = cook_torrance_brdf(normal, V, L, albedo, metallic, roughness);
           float NdotL = max(0, dot(normal, L));

           direct = Le * brdf * NdotL * cos_light * visibility / dist2 * r.W;
       }

       // 加上 SDFGI 间接光照
       float3 indirect = sample_sdfgi(pos, normal);

       float3 final_color = direct + indirect * albedo;
       store_hdr_color(pixel, final_color);
   }
   ```

   > **面光源软阴影原理**：ReSTIR temporal/spatial reuse 后，同一像素在不同帧/邻居中采样到面光源的不同表面点。每个采样点的 SDF visibility 不同（部分被遮挡，部分可见），经过降噪器平滑后自然呈现出与面光源面积成比例的半影。这是物理正确的软阴影，不需要额外处理。

## 可见性处理策略

ReSTIR 的可见性处理是一个关键设计选择：

| 策略 | 优点 | 缺点 |
|------|------|------|
| **Shade 时检查** | 简单 | 可能采样被遮挡的光源 |
| Initial sampling 时检查 | 更好的样本 | 每候选一次 shadow test → 慢 |
| 混合：temporal 时不检查，shade 时检查 | 平衡 | 推荐方案 |

我们选择**混合方案**：
- Initial sampling 和 reuse 阶段不做 visibility → 速度快
- Final shade 阶段做一次 SDF ray march visibility → 正确性

## Pipeline 集成

```cpp
void SRTPipeline::render(FrameTicket& ticket) {
    shadow_pass_.execute(ticket);
    gbuffer_pass_.execute(ticket);

    // SDFGI
    sdfgi_trace_.execute(ticket);
    sdfgi_update_.execute(ticket);

    // ReSTIR DI
    restir_initial_.execute(ticket);   // compute: 初始候选采样
    restir_temporal_.execute(ticket);  // compute: 时间复用
    restir_spatial_.execute(ticket);   // compute: 空间复用
    restir_shade_.execute(ticket);     // compute: 最终着色

    // Post
    present_pass_.execute(ticket);
}
```

## 面光源与 LTC 的协作

Phase 0 的 LTC 和 Phase 3 的 ReSTIR 面光源采样在最终 pipeline 中如何协作：

```
策略 A（简单）: 完全用 ReSTIR 采样替代 LTC
  - 优点：统一管线，阴影自动正确
  - 缺点：面光源直视时可能有轻微噪点

策略 B（推荐）: LTC 用于面光源自身可见区域的无阴影 specular highlight
                 ReSTIR 用于面光源的阴影和 diffuse 贡献
  - 在 restir_shade 中：
    - diffuse + shadow → ReSTIR 路径
    - specular highlight（光滑表面正对面光源时）→ LTC 路径
  - 通过 roughness 阈值混合：
    roughness < 0.3 → 更多 LTC weight（specular 主导，需要无噪点）
    roughness > 0.7 → 完全 ReSTIR（diffuse 主导，噪点不敏感）
```

初期推荐策略 A（简单），验证正确后可切换到策略 B 提升高光质量。

## Phase 3 完成标准

- [ ] Reservoir 数据结构正确实现 WRS（含 sample_point 字段）
- [ ] 面光源表面采样函数正确（rect / disk / sphere）
- [ ] Initial candidate sampling 从 1000+ 光源（含面光源）中有效采样
- [ ] 面光源的 source PDF 正确计算（= 1/light_count × 1/area）
- [ ] Temporal reuse 显著降低噪点（对比无 temporal）
- [ ] Spatial reuse 进一步平滑（对比无 spatial）
- [ ] 面光源产生物理正确的软阴影（半影宽度与面积成比例）
- [ ] Motion vector 正确驱动 temporal reuse
- [ ] 几何一致性检查有效防止跨表面复用
- [ ] 1000 光源（含面光源）场景下 ReSTIR 全流程 < 4ms @ 1080p
- [ ] 与 SDFGI 间接光照正确合成

## 关键参考

- "Spatiotemporal Reservoir Resampling for Real-Time Ray Tracing with Dynamic Direct Lighting" (Bitterli et al. 2020) — ReSTIR DI 原始论文
- "Rearchitecting Spatiotemporal Resampling for Production" (Wyman & Panteleev 2021) — 工程化改进
- "A Gentle Introduction to ReSTIR" — 教学资源
- "Real-Time Polygonal-Light Shading with Linearly Transformed Cosines" (Heitz et al. 2016) — LTC 面光源
- "Real-Time Area Lighting: a Journey from Research to Production" (Heitz et al. 2016, Advances in Real-Time Rendering) — 工程实践
