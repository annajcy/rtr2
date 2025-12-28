struct VSInput {
    float3 in_position : POSITION0;
    float2 in_uv : TEXCOORD0;
    float3 in_normal : NORMAL0;
};

struct PSInput {
    float4 pos : SV_Position;
    float3 color : COLOR0;
    float2 uv : TEXCOORD0;
    float3 normal : NORMAL0;
};

struct UniformBufferObject {
    float4x4 model;
    float4x4 view;
    float4x4 proj;
    float4x4 normal; // inverse-transpose of model
};

[[vk_binding(0, 0)]]
ConstantBuffer<UniformBufferObject> ubo;

[shader("vertex")]
PSInput vertMain(VSInput input) {
    PSInput output;
    float4 world_pos = mul(float4(input.in_position, 1.0), ubo.model);
    float4 view_pos = mul(world_pos, ubo.view);
    output.pos = mul(view_pos, ubo.proj);
    // Proper normal transform uses inverse-transpose of the upper-left 3x3.
    float3x3 normal_mat = (float3x3)ubo.normal;
    float3 world_normal = normalize(mul(normal_mat, input.in_normal));
    // Map [-1,1] to [0,1] for visualization.
    output.color = world_normal * 0.5 + 0.5;
    output.uv = input.in_uv;
    output.normal = world_normal;
    return output;
}

[shader("fragment")]
float4 fragMain(PSInput input) : SV_Target {
    return float4(input.color, 1.0);
}
