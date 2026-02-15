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
    row_major float4x4 model;
    row_major float4x4 view;
    row_major float4x4 proj;
    row_major float4x4 normal; // inverse-transpose of model
    float4 base_color;
};

[[vk_binding(0, 0)]]
ConstantBuffer<UniformBufferObject> ubo;

[shader("vertex")]
PSInput vertMain(VSInput input) {
    PSInput output;
    float4 world_pos = mul(ubo.model, float4(input.in_position, 1.0));
    float4 view_pos = mul(ubo.view, world_pos);
    output.pos = mul(ubo.proj, view_pos);
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
    return ubo.base_color;
}
