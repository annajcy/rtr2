struct ShaderToyParams {
    float4 iResolution;
    float4 iTime;
    float4 iParams;
};

[[vk_binding(0, 0)]]
ConstantBuffer<ShaderToyParams> params;

[[vk_binding(1, 0)]]
RWTexture2D<float4> outColor;

float3 palette(float t) {
    float3 a = float3(0.45, 0.35, 0.20);
    float3 b = float3(0.55, 0.40, 0.25);
    float3 c = float3(0.90, 0.70, 0.35);
    float3 d = float3(0.15, 0.10, 0.05);
    return a + b * cos(6.28318 * (c * t + d));
}

[numthreads(8, 8, 1)]
[shader("compute")]
void compMain(uint3 tid : SV_DispatchThreadID) {
    uint width = (uint)params.iResolution.x;
    uint height = (uint)params.iResolution.y;
    if (tid.x >= width || tid.y >= height) {
        return;
    }

    float2 uv = ((float2)tid.xy + 0.5) / params.iResolution.xy;
    float2 p = uv * 2.0 - 1.0;
    p.x *= params.iResolution.x / max(params.iResolution.y, 1.0);

    float time = params.iTime.x * (0.5 + params.iParams.y);
    float radius = 1 + 0.25 * sin(time * 0.7 + params.iParams.z);
    float ripple = sin(18.0 * length(p) - time * (1.5 + params.iParams.x)) * 0.5 + 0.5;
    float ring = smoothstep(radius + 0.04, radius - 0.04, abs(length(p) - radius));

    float glow = exp(-4.5 * length(p)) * (0.35 + 0.65 * ripple);
    float hue = ripple + ring * 0.5 + uv.x * 0.2;
    float3 color = palette(hue) * (glow + ring);

    outColor[tid.xy] = float4(color, 1.0);
}
