# Phase 4: 降噪、后处理与集成（~2 天）

## 目标

将 SDFGI + ReSTIR 的输出进行降噪和后处理，产出最终高质量画面。

## Day 13: 降噪器

### 为什么需要降噪

- ReSTIR 每像素最终只评估 1 个光源 → 仍有残留噪点
- SDFGI probe 分辨率有限 → 间接光照有块状伪影
- 降噪器是从"能用"到"好看"的关键一步

### 方案：SVGF (Spatiotemporal Variance-Guided Filtering)

| 方案 | 优点 | 缺点 |
|------|------|------|
| Bilateral filter | 简单 | 无时间稳定性 |
| **SVGF** | 时空稳定，方差自适应 | 实现中等 |
| NRD (NVIDIA) | 最佳质量 | 闭源 / 重度依赖 |

选择 SVGF：开源、学术友好、compute shader 实现。

### SVGF 流程

```
1. 时间累积：用 motion vector 混合当前帧和历史帧
2. 方差估计：计算局部亮度方差
3. A-trous 小波滤波：5 次迭代，步长 1→2→4→8→16
   - edge-stopping: normal / depth / luminance 引导
   - 方差高的区域滤波更强
```

### 任务

1. **Denoiser** — `denoiser.hpp` + `shaders/srt/denoise_svgf.slang`

   ```slang
   // Pass 1: Temporal accumulation
   [numthreads(8, 8, 1)]
   void temporal_accumulation(uint3 tid : SV_DispatchThreadID) {
       uint2 pixel = tid.xy;
       float3 color = load_hdr_color(pixel);
       float3 normal = load_gbuffer_normal(pixel);
       float depth = load_gbuffer_depth(pixel);

       // Reproject
       float2 motion = load_motion_vector(pixel);
       int2 prev_pixel = int2(pixel) + int2(round(motion));

       float alpha = 0.2;  // 混合系数

       if (valid_reprojection(prev_pixel, normal, depth)) {
           float3 prev_color = load_history(prev_pixel);
           color = lerp(prev_color, color, alpha);
       }

       store_accumulated(pixel, color);
   }

   // Pass 2-6: A-trous wavelet filter (5 iterations)
   [numthreads(8, 8, 1)]
   void atrous_filter(uint3 tid : SV_DispatchThreadID) {
       uint2 pixel = tid.xy;
       int step_size = 1 << iteration;  // 1, 2, 4, 8, 16

       float3 center_color = load_filtered(pixel);
       float3 center_normal = load_gbuffer_normal(pixel);
       float center_depth = load_gbuffer_depth(pixel);
       float center_lum = luminance(center_color);

       // 5×5 a-trous kernel
       float kernel[3] = {1.0, 2.0/3.0, 1.0/6.0};

       float3 sum = float3(0);
       float weight_sum = 0;

       for (int dy = -2; dy <= 2; dy++) {
           for (int dx = -2; dx <= 2; dx++) {
               int2 p = int2(pixel) + int2(dx, dy) * step_size;
               if (any(p < 0) || any(p >= resolution)) continue;

               float3 s_color = load_filtered(p);
               float3 s_normal = load_gbuffer_normal(p);
               float s_depth = load_gbuffer_depth(p);

               // Edge-stopping weights
               float w_normal = pow(max(0, dot(center_normal, s_normal)), 128.0);
               float w_depth = exp(-abs(center_depth - s_depth) / (sigma_depth * abs(float(dx + dy) * step_size) + 1e-6));
               float w_lum = exp(-abs(center_lum - luminance(s_color)) / (sigma_lum + 1e-6));

               float w = kernel[abs(dx)] * kernel[abs(dy)] * w_normal * w_depth * w_lum;
               sum += s_color * w;
               weight_sum += w;
           }
       }

       store_filtered(pixel, sum / max(weight_sum, 1e-6));
   }
   ```

## Day 13-14: TAA + Tone Mapping

### TAA（Temporal Anti-Aliasing）

1. **TAA Shader** — `shaders/srt/taa.slang`
   ```slang
   [numthreads(8, 8, 1)]
   void main(uint3 tid : SV_DispatchThreadID) {
       uint2 pixel = tid.xy;

       float3 current = load_denoised(pixel);
       float2 motion = load_motion_vector(pixel);
       float2 prev_uv = (float2(pixel) + 0.5 + motion) / float2(resolution);

       float3 history = sample_bilinear(taa_history, prev_uv);

       // Neighborhood clamping (3×3)
       float3 near_min = float3(1e10);
       float3 near_max = float3(-1e10);
       for (int dy = -1; dy <= 1; dy++) {
           for (int dx = -1; dx <= 1; dx++) {
               float3 s = load_denoised(pixel + int2(dx, dy));
               near_min = min(near_min, s);
               near_max = max(near_max, s);
           }
       }
       history = clamp(history, near_min, near_max);

       float3 result = lerp(history, current, 0.1);
       store_taa_output(pixel, result);
       store_taa_history(pixel, result);
   }
   ```

   需要在 G-Buffer pass 中加入 **sub-pixel jitter**：
   - 每帧用 Halton(2,3) 序列偏移投影矩阵
   - 偏移量: ±0.5 pixel

### Tone Mapping

2. **Tone Mapping Shader** — `shaders/srt/tone_mapping.slang`
   ```slang
   // ACES Filmic Tone Mapping
   float3 aces_film(float3 x) {
       float a = 2.51;
       float b = 0.03;
       float c = 2.43;
       float d = 0.59;
       float e = 0.14;
       return saturate((x * (a * x + b)) / (x * (c * x + d) + e));
   }

   [numthreads(8, 8, 1)]
   void main(uint3 tid : SV_DispatchThreadID) {
       uint2 pixel = tid.xy;
       float3 hdr = load_taa_output(pixel);

       // Exposure
       hdr *= exposure;

       // Tone map
       float3 ldr = aces_film(hdr);

       // Gamma correction (sRGB)
       ldr = pow(ldr, 1.0 / 2.2);

       store_final(pixel, float4(ldr, 1.0));
   }
   ```

## Day 14: 完整 Pipeline 集成

### 最终 SRT Pipeline

```cpp
class SRTPipeline : public RenderPipeline {
    // Phase 0
    GBufferPass        gbuffer_pass_;
    ShadowPass         shadow_pass_;

    // Phase 1
    SDFBuilder         sdf_builder_;
    SDFCascade         sdf_cascade_;

    // Phase 2
    SDFGITracePass     sdfgi_trace_;
    SDFGIUpdatePass    sdfgi_update_;

    // Phase 3
    ReSTIRInitialPass  restir_initial_;
    ReSTIRTemporalPass restir_temporal_;
    ReSTIRSpatialPass  restir_spatial_;
    ReSTIRShadePass    restir_shade_;

    // Phase 4
    DenoisePass        denoise_pass_;      // SVGF (6 sub-passes)
    TAAPass            taa_pass_;
    ToneMappingPass    tone_mapping_pass_;
    PresentPass        present_pass_;

    void render(FrameTicket& ticket) override {
        // 几何
        gbuffer_pass_.execute(ticket);
        shadow_pass_.execute(ticket);

        // SDF 更新（按需，不是每帧）
        if (sdf_dirty_) {
            sdf_builder_.rebuild(ticket);
            sdf_dirty_ = false;
        }
        sdf_cascade_.update(ticket, camera_pos_);

        // 间接光照
        sdfgi_trace_.execute(ticket);
        sdfgi_update_.execute(ticket);

        // 直接光照
        restir_initial_.execute(ticket);
        restir_temporal_.execute(ticket);
        restir_spatial_.execute(ticket);
        restir_shade_.execute(ticket);   // 合并直接 + 间接

        // 后处理
        denoise_pass_.execute(ticket);   // SVGF
        taa_pass_.execute(ticket);       // TAA
        tone_mapping_pass_.execute(ticket);

        // 呈现
        present_pass_.execute(ticket);
    }
};
```

### 资源总览

```
GPU 资源估算 @ 1080p:

G-Buffer:
  RT0 (RGBA16F):           1920×1080×8  = 16 MB
  RT1 (RGBA16F):           1920×1080×8  = 16 MB
  RT2 (RGBA32F):           1920×1080×16 = 32 MB
  Depth (D32F):            1920×1080×4  = 8 MB
  Motion Vector (RG16F):   1920×1080×4  = 8 MB

Shadow Maps:
  4 cascade × 2048²×4:     64 MB

SDF Cascades:
  4 × 128³ × 2 (R16F):     32 MB

SDFGI Probes:
  4 cascade × 128³ × 10×10 × 8 (RGBA16F):
  ~300 MB（可优化：只激活表面附近 probe）

Reservoir Buffer:
  2 × 1920×1080 × 16:      64 MB

Denoise/TAA/HDR buffers:
  ~100 MB

总计: ~640 MB（可接受，优化后可降至 ~400 MB）
```

### 性能预算

```
@ 1080p, 目标 16.6ms (60fps)

G-Buffer Pass:         1.0 ms
Shadow Pass:           1.0 ms
SDF Cascade Update:    0.5 ms (amortized)
SDFGI Trace:           2.0 ms
SDFGI Update:          0.5 ms
ReSTIR Initial:        0.5 ms
ReSTIR Temporal:       0.3 ms
ReSTIR Spatial:        0.5 ms
ReSTIR Shade:          0.5 ms
SVGF Denoise (6 pass): 2.0 ms
TAA:                   0.3 ms
Tone Mapping:          0.1 ms
Present:               0.1 ms
─────────────────────────────
Total:                ~8.8 ms → ~114 fps headroom
```

## Phase 4 完成标准

- [ ] SVGF 降噪有效消除 ReSTIR 残留噪点
- [ ] TAA 消除锯齿，无明显 ghosting
- [ ] Tone mapping 正确处理 HDR → LDR
- [ ] 完整 SRT Pipeline 端到端运行
- [ ] Cornell Box + 1000 光源场景视觉正确：
  - 直接光照清晰
  - 间接光照有 color bleeding
  - 阴影柔和
  - 无明显伪影
- [ ] 整体帧时间 < 16.6ms @ 1080p (60fps)
- [ ] macOS / Linux / Windows 可运行

## 后续扩展（不在本计划内）

- **ReSTIR GI**：将 ReSTIR 扩展到间接光照采样，替代或补充 SDFGI
- **半透明 / 体积**：SDF 体积光、大气散射
- **动态 SDF 更新**：骨骼动画物体的增量 SDF 更新
- **Specular GI**：SDFGI glossy reflection 改进
- **可变速率着色**：VRS 降低边缘区域计算量
- **异步 compute**：SDF 更新和 probe tracing 在异步 compute queue 执行
