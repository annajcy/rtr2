struct ShaderToyParams {
    float4 iResolution;
    float4 iTime;
};

[[vk_binding(0, 0)]]
ConstantBuffer<ShaderToyParams> params;

[[vk_binding(1, 0)]]
RWTexture2D<float4> outColor;

[[vk_binding(0, 1)]]
Sampler2D<float4> offscreenSampler;

[numthreads(8, 8, 1)]
[shader("compute")]
void compMain(uint3 tid : SV_DispatchThreadID) {
    uint width = (uint)params.iResolution.x;
    uint height = (uint)params.iResolution.y;
    if (tid.x >= width || tid.y >= height) {
        return;
    }

    float2 uv = (float2(tid.xy) + 0.5) / params.iResolution.xy;
    float time = params.iTime.x;

    float2 p = uv * 2.0 - 1.0;
    p.x *= params.iResolution.x / max(params.iResolution.y, 1.0);
    float r = length(p);
    float a = atan2(p.y, p.x);

    float wave = 0.5 + 0.5 * cos(8.0 * r - 2.5 * time + a * 2.0);
    float3 base = 0.5 + 0.5 * cos(time + float3(0.0, 2.0, 4.0) + uv.xyx * 6.0);
    float3 color = lerp(base, base.zyx, wave);

    outColor[tid.xy] = float4(color, 1.0);
}

struct VSOut {
    float4 pos : SV_Position;
    float2 uv : TEXCOORD0;
};

[shader("vertex")]
VSOut vertMain(uint vertexID : SV_VertexID) {
    VSOut outv;
    float2 corner = float2((vertexID << 1) & 2, vertexID & 2);
    outv.pos = float4(corner * 2.0 + float2(-1.0, -1.0), 0.0, 1.0);
    outv.uv = corner * 0.5;
    return outv;
}

[shader("fragment")]
float4 fragMain(VSOut input) : SV_Target {
    float2 uv = float2(input.uv.x, 1.0 - input.uv.y);
    return offscreenSampler.Sample(uv);
}
