struct ShaderToyParams {
    float4 iResolution;
    float4 iTime;
    float4 iParams; // iParams.x = scale, iParams.y = speed, iParams.z = offset
};

[[vk_binding(0, 0)]]
ConstantBuffer<ShaderToyParams> params;

[[vk_binding(1, 0)]]
RWTexture2D<float4> outColor;

[numthreads(8, 8, 1)]
[shader("compute")]
void compMain(uint3 tid : SV_DispatchThreadID) {
    uint width = (uint)params.iResolution.x;
    uint height = (uint)params.iResolution.y;
    if (tid.x >= width || tid.y >= height) {
        return;
    }

    float2 uv = (float2(tid.xy) + 0.5) / params.iResolution.xy;
    float time = params.iTime.x * params.iParams.y; // param.y as speed
    float2 p = uv * params.iParams.x - 0.5 * params.iParams.x; // param.x as scale

    float3 red = float3(1.0, 0.0, 0.0);
    float3 green = float3(0.0, 1.0, 0.0);
    float3 blue = float3(0.0, 0.0, 1.0);
    
    float pulse = sin(time + length(p) * 5.0) * 0.5 + 0.5;
    float3 color = (uv.x * red + uv.y * blue + (2.0 - uv.x - uv.y) * green) * pulse;
    outColor[tid.xy] = float4(color, 1.0);
}
