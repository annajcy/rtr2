# Phase 1: SDF 基础设施（~3 天）

## 目标

构建从网格到 SDF 体素的完整管线，实现 GPU 上的 SDF ray marching，为 SDFGI probe tracing 提供加速结构。

## 核心概念

### 什么是 SDF

Signed Distance Field 是一个 3D 标量场，每个体素存储到最近表面的**带符号距离**：
- 正值 = 在物体外部
- 负值 = 在物体内部
- 零 = 恰好在表面上

### 为什么用 SDF 做光追

- **Sphere tracing**：每步可以安全前进 |SDF(p)| 的距离，通常 10-30 步就能命中
- **不需要 BVH / 加速结构**：SDF 本身就是加速结构
- **Compute shader 友好**：只需要 3D texture 采样
- **天然支持软阴影**：沿射线记录最小 SDF/距离比即可

### 级联 SDF（Cascaded SDF）

单一分辨率的 SDF 无法兼顾精度和覆盖范围。参考 Godot 4 SDFGI 方案：

```
Cascade 0:  高分辨率，小范围（相机附近 8m，128³）
Cascade 1:  中分辨率，中范围（32m，128³）
Cascade 2:  低分辨率，大范围（128m，128³）
Cascade 3:  最低分辨率，最大范围（512m，128³）
```

每级覆盖范围是上一级的 4 倍，体素分辨率相同（128³），但 voxel size 增大 4 倍。

## Day 3-4: 网格 → SDF 体素化

### 方案选择

| 方案 | 优点 | 缺点 |
|------|------|------|
| CPU jump flooding | 简单 | 慢，不适合实时更新 |
| **GPU 体素化 + JFA** | 快，适合静态场景 | 实现复杂度中等 |
| 保守光栅化 | 硬件加速 | 需要 VK_EXT_conservative_rasterization |

选择 **GPU 体素化 + 3D Jump Flooding Algorithm (JFA)**：

### 任务

1. **SDF Builder** — `sdf_builder.hpp`
   ```cpp
   class SDFBuilder {
       // Step 1: 三角形 → 体素化（compute shader）
       //   每个三角形在 3D grid 中标记占据的体素
       // Step 2: JFA（compute shader，log2(N) 次 dispatch）
       //   从种子体素向外传播最近距离
       // Step 3: 符号判定（compute shader）
       //   用 ray casting 奇偶规则确定内外

       void build(const Mesh& mesh, SDFCascade& cascade);
       void rebuild_cascade(uint32_t cascade_index);
   };
   ```

2. **体素化 Shader** — `shaders/srt/sdf_voxelize.slang`
   - 输入：三角形 SSBO
   - 输出：3D image（R32F），种子体素标记为 0，其余为 MAX_FLOAT
   - 每个线程组处理一个三角形，遍历其 AABB 内的体素

3. **JFA Shader** — `shaders/srt/sdf_jfa.slang`
   - 经典 3D JFA：step size 从 N/2 递减到 1
   - 每个体素检查 26 个邻居方向（3³-1）
   - 传播最近种子位置
   - 最后一步计算 distance = length(pos - nearest_seed)

4. **符号判定 Shader** — `shaders/srt/sdf_sign.slang`
   - 每个体素发射一条 +X 方向射线
   - 统计与三角形的交点数（奇数 = 内部）
   - 也可用多方向投票提高鲁棒性

### 验证

- 用 3D slice 可视化 SDF（取 z = 0 平面，绘制等值线）
- Sphere mesh → SDF 应接近解析球 SDF
- 内外符号正确（bunny 内部为负值）

## Day 4-5: 级联 SDF 管理

### 任务

1. **SDF Cascade** — `sdf_cascade.hpp`
   ```cpp
   struct SDFCascadeLevel {
       vk::raii::Image     volume;        // 3D texture (128³ R16F)
       vk::raii::ImageView volume_view;
       glm::vec3           center;        // 级联中心（跟随相机）
       float               voxel_size;    // 每体素世界尺寸
       float               extent;        // 级联覆盖半径
   };

   class SDFCascade {
       static constexpr uint32_t NUM_CASCADES = 4;
       static constexpr uint32_t RESOLUTION = 128;

       std::array<SDFCascadeLevel, NUM_CASCADES> levels_;

       // 相机移动时，只滚动更新变化的 slice（toroidal addressing）
       void update(const glm::vec3& camera_pos);
   };
   ```

2. **Toroidal Addressing**
   - 级联跟随相机移动时，不要整体重建
   - 用环形寻址：当相机移动超过一个 voxel size 时，只更新边缘的新 slice
   - 这是 SDFGI 实时性能的关键

3. **级联间混合**
   - ray march 时，在级联边界处平滑过渡
   - 使用级联索引的 LOD 选择：`cascade = clamp(log4(distance / base_range), 0, 3)`

### 验证

- 4 个级联 3D texture 正确分配
- 相机移动时 slice 滚动更新正确（无撕裂）
- 级联间 SDF 值连续

## Day 5: SDF Ray Marching

### 任务

1. **SDF Ray March** — `sdf_ray_march.hpp` + `shaders/srt/sdf_ray_march.slang`
   ```slang
   struct SDFHit {
       float3 position;
       float3 normal;    // 通过 SDF 梯度计算
       float  distance;
       bool   hit;
       int    cascade_level;
   };

   SDFHit sdf_ray_march(float3 origin, float3 direction, float max_dist) {
       float t = 0;
       for (int i = 0; i < MAX_STEPS; i++) {
           float3 p = origin + direction * t;
           int cascade = select_cascade(p);
           float d = sample_sdf(p, cascade);
           if (d < EPSILON) {
               // 命中：用中心差分计算 normal
               return hit(p, sdf_gradient(p, cascade), t);
           }
           t += d * STEP_SCALE;  // STEP_SCALE < 1.0 提高稳定性
           if (t > max_dist) break;
       }
       return miss();
   }
   ```

2. **软阴影 via SDF**
   ```slang
   float sdf_soft_shadow(float3 origin, float3 light_dir, float max_dist, float k) {
       float shadow = 1.0;
       float t = MIN_T;
       for (int i = 0; i < MAX_STEPS; i++) {
           float d = sample_sdf(origin + light_dir * t, select_cascade(origin + light_dir * t));
           if (d < EPSILON) return 0.0;
           shadow = min(shadow, k * d / t);  // Inigo Quilez 软阴影技巧
           t += d;
           if (t > max_dist) break;
       }
       return shadow;
   }
   ```

3. **Debug 可视化**
   - 全屏 compute shader：对每个像素从相机发射射线，用 SDF ray march 渲染场景
   - 用于验证 SDF 质量（应与光栅化结果近似）
   - 可视化 ray march 步数（越少越好，理想 < 30 步）

### 验证

- SDF ray march 渲染结果与光栅化 G-Buffer 视觉对比一致
- 软阴影产生平滑的半影
- 平均 ray march 步数 < 30（日志输出统计）
- 在 128³×4 cascade 配置下 ray march 10 万条射线 < 2ms

## Phase 1 完成标准

- [ ] GPU 体素化 + JFA 正确生成 SDF
- [ ] SDF 符号内外正确
- [ ] 4 级级联 SDF 正确分配和管理
- [ ] Toroidal addressing 在相机移动时正常工作
- [ ] SDF ray march 在 debug 模式下渲染结果正确
- [ ] 软阴影通过 SDF 实现并视觉正确
- [ ] 性能：ray march 10 万条射线 < 2ms @ 1080p

## 关键参考

- Inigo Quilez, "Distance Functions" — SDF 基础
- Godot 4 SDFGI 实现 — 级联设计参考
- "Jump Flooding in GPU with Applications to Voronoi Diagram and Distance Transform" (Rong & Tan 2006)
