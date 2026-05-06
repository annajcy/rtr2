# Phase B: 切线空间与法线贴图（~1 天）

## 目标

为法线贴图计算逐顶点 tangent 向量，构建 TBN 矩阵，在 shader 中实现切线空间法线采样。

## 为什么需要切线空间

法线贴图存储的是**切线空间**中的法线扰动：
- R = tangent 方向偏移
- G = bitangent 方向偏移
- B = 法线方向（几何法线方向）

要将采样的切线空间法线变换到世界空间，需要 TBN 矩阵：

```
TBN = [T | B | N]   (3×3，列向量)
world_normal = normalize(TBN * tangent_space_normal)
```

其中 T = tangent，B = bitangent，N = geometric normal。

## 顶点格式扩展

### 修改 `obj_types.hpp`

```cpp
struct ObjVertex {
    pbpt::math::Vec3 position;   // 12 bytes
    pbpt::math::Vec2 uv;         // 8 bytes
    pbpt::math::Vec3 normal;     // 12 bytes
    pbpt::math::Vec4 tangent;    // 16 bytes — 新增
    // tangent.xyz = tangent 方向
    // tangent.w   = handedness (±1)，用于计算 bitangent：
    //               B = cross(N, T.xyz) * T.w
};
// Total: 48 bytes per vertex (was 32)
```

> **为什么 tangent 是 Vec4？**
> bitangent 可以从 `cross(N, T) * handedness` 重建，不需要显式存储。
> `handedness = ±1` 编码 UV 空间的朝向（镜像 UV 时为 -1）。
> 这是 glTF 2.0 和 MikkTSpace 的标准约定。

### 修改 `mesh.hpp` — 顶点属性

```cpp
// 新增 attribute description
static std::vector<vk::VertexInputAttributeDescription> attribute_descriptions() {
    return {
        {0, 0, vk::Format::eR32G32B32Sfloat, 0},                    // position
        {1, 1, vk::Format::eR32G32Sfloat, 0},                       // uv
        {2, 2, vk::Format::eR32G32B32Sfloat, 0},                    // normal
        {3, 3, vk::Format::eR32G32B32A32Sfloat, 0},                 // tangent (新增)
    };
}
```

## 切线计算

### 方案选择

| 方案 | 优点 | 缺点 |
|------|------|------|
| **MikkTSpace** | 业界标准，与大多数 DCC 工具匹配 | 需要集成第三方库 |
| 手写 Lengyel 方法 | 简单，无依赖 | 不保证与 DCC 工具一致 |
| glTF 自带 tangent | 最准确 | 仅限 glTF 格式 |

**推荐**：MikkTSpace（header-only，~500 行），fallback 用 Lengyel 方法。

### MikkTSpace 集成

`tangent_compute.hpp`:
```cpp
#include <mikktspace.h>  // header-only

// MikkTSpace 回调适配器
class MikkTSpaceAdapter {
    std::vector<ObjVertex>& vertices_;
    const std::vector<uint32_t>& indices_;

    // 实现 SMikkTSpaceInterface 回调：
    // getNumFaces, getNumVerticesOfFace, getPosition, getNormal, getTexCoord
    // setTSpaceBasic (接收计算出的 tangent + sign)
};

void compute_tangents(std::vector<ObjVertex>& vertices,
                      const std::vector<uint32_t>& indices) {
    SMikkTSpaceInterface iface{};
    iface.m_getNumFaces = [](const SMikkTSpaceContext* ctx) -> int { ... };
    iface.m_getNumVerticesOfFace = [](const SMikkTSpaceContext* ctx, int face) -> int { return 3; };
    iface.m_getPosition = [](const SMikkTSpaceContext* ctx, float out[], int face, int vert) { ... };
    iface.m_getNormal = [...];
    iface.m_getTexCoord = [...];
    iface.m_setTSpaceBasic = [](const SMikkTSpaceContext* ctx, const float tangent[],
                                 float sign, int face, int vert) {
        // 写入 vertex.tangent = {tangent[0], tangent[1], tangent[2], sign}
    };

    SMikkTSpaceContext ctx{};
    ctx.m_pInterface = &iface;
    ctx.m_pUserData = /* adapter */;

    genTangSpaceDefault(&ctx);
}
```

### Fallback: Lengyel 方法

当 UV 退化或三角形退化时 MikkTSpace 可能失败，需要 fallback：

```cpp
// Lengyel's tangent calculation
// 对每个三角形：
//   edge1 = v1 - v0, edge2 = v2 - v0
//   duv1 = uv1 - uv0, duv2 = uv2 - uv0
//   r = 1 / (duv1.x * duv2.y - duv2.x * duv1.y)
//   T = (edge1 * duv2.y - edge2 * duv1.y) * r
//   B = (edge2 * duv1.x - edge1 * duv2.x) * r
// 然后逐顶点平均 + 正交化（Gram-Schmidt）：
//   T' = normalize(T - N * dot(N, T))
//   handedness = sign(dot(cross(N, T), B))

void compute_tangents_fallback(std::vector<ObjVertex>& vertices,
                                const std::vector<uint32_t>& indices);
```

### 集成到加载流程

修改 `obj_io.hpp` 的 `load_obj()`:
```cpp
ObjLoadResult load_obj(const std::string& path) {
    // ... 现有加载逻辑 ...

    // 新增：计算切线
    compute_tangents(result.vertices, result.indices);

    return result;
}
```

## Shader 端法线贴图

### TBN 矩阵构建 — `shaders/normal_mapping.slang`

```slang
// 顶点 shader 输出
struct VSOutput {
    float4 position : SV_Position;
    float2 uv       : TEXCOORD0;
    float3 world_pos : TEXCOORD1;
    float3 world_normal  : TEXCOORD2;
    float4 world_tangent : TEXCOORD3;   // xyz = tangent, w = handedness
};

// 在顶点 shader 中
VSOutput vert_main(VSInput input, uniform PerObjectUBO ubo) {
    VSOutput o;
    // ... position transform ...

    o.world_normal = normalize((ubo.normal_matrix * float4(input.normal, 0)).xyz);

    // tangent 同样用 normal matrix 变换（只旋转+缩放，不平移）
    o.world_tangent.xyz = normalize((ubo.normal_matrix * float4(input.tangent.xyz, 0)).xyz);
    o.world_tangent.w = input.tangent.w;  // 保持 handedness

    return o;
}

// 在 fragment shader 中获取扰动法线
float3 get_normal_from_map(float2 uv, float3 world_normal, float4 world_tangent,
                            Texture2D normal_map, SamplerState samp) {
    // 采样法线贴图
    float3 tangent_normal = normal_map.Sample(samp, uv).xyz;
    tangent_normal = tangent_normal * 2.0 - 1.0;  // [0,1] → [-1,1]

    // 构建 TBN
    float3 N = normalize(world_normal);
    float3 T = normalize(world_tangent.xyz);

    // Gram-Schmidt 正交化（插值后 T 和 N 可能不完全正交）
    T = normalize(T - N * dot(N, T));

    float3 B = cross(N, T) * world_tangent.w;  // handedness

    float3x3 TBN = float3x3(T, B, N);

    return normalize(mul(tangent_normal, TBN));
}
```

### 无法线贴图时的行为

当绑定默认法线纹理 `(0.5, 0.5, 1.0)`：
```
tangent_normal = (0.5, 0.5, 1.0) * 2 - 1 = (0, 0, 1)
TBN * (0, 0, 1) = N   → 几何法线，无扰动
```

完美回退，无需 shader 分支。

## DynamicMesh 的切线处理

`DeformableMeshComponent` 的顶点在 CPU 端动态更新。切线需要在变形后重新计算：

```cpp
// 选项 1: 每帧 CPU 重算（简单但慢）
// 选项 2: 只在拓扑变化时重算（deformable mesh 拓扑通常不变）
// 选项 3: 在 compute shader 中重算（最快）

// 推荐选项 2：拓扑不变时，可以在初始化时计算 tangent，
// 变形后 tangent 的误差通常可接受（除非极端变形）
```

## Phase B 完成标准

- [ ] `ObjVertex` 扩展为 48 bytes（新增 Vec4 tangent）
- [ ] 顶点属性描述新增 location 3（tangent）
- [ ] MikkTSpace 集成，`compute_tangents()` 正确运行
- [ ] Lengyel fallback 处理退化 UV 三角形
- [ ] `load_obj()` 自动计算 tangent
- [ ] Shader TBN 矩阵构建正确
- [ ] 默认法线纹理时 TBN * (0,0,1) = 几何法线（无扰动）
- [ ] 法线贴图采样后世界空间法线方向正确
- [ ] 验证：平面 + checker normal map → 可见凹凸细节
