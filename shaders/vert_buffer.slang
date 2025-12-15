struct VSInput {
    float2 in_position : POSITION0;
    float3 in_color : COLOR0;
};

struct PSInput {
    float4 pos : SV_Position;
    float3 color : COLOR0;
};

struct UniformBufferObject {
    float4x4 model;
    float4x4 view;
    float4x4 proj;
};

[[vk_binding(0, 0)]]
ConstantBuffer<UniformBufferObject> ubo;

[shader("vertex")]
PSInput vertMain(VSInput input) {
    PSInput output;
    float4 world_pos = mul(float4(input.in_position, 0.0, 1.0), ubo.model);
    float4 view_pos = mul(world_pos, ubo.view);
    output.pos = mul(view_pos, ubo.proj);
    output.color = input.in_color;
    return output;
}

[shader("fragment")]
float4 fragMain(PSInput input) : SV_Target {
    return float4(input.color, 1.0);
}



