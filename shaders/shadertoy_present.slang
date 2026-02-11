[[vk_binding(0, 1)]]
Sampler2D<float4> offscreenSampler;

struct VSOut {
    float4 pos : SV_Position;
    float2 uv : TEXCOORD0;
};

[shader("vertex")]
VSOut vertMain(uint vertexID : SV_VertexID) {
    VSOut outv;
    float2 pos;
    switch (vertexID) {
    case 0:
        pos = float2(-1.0, -1.0);
        break;
    case 1:
        pos = float2(3.0, -1.0);
        break;
    default:
        pos = float2(-1.0, 3.0);
        break;
    }

    outv.pos = float4(pos, 0.0, 1.0);
    outv.uv = (pos + 1.0) * 0.5;
    return outv;
}

[shader("fragment")]
float4 fragMain(VSOut input) : SV_Target {
    float2 uv = float2(input.uv.x, 1.0 - input.uv.y);
    return offscreenSampler.Sample(uv);
}
