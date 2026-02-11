struct ShaderToyParams {
    float4 iResolution;
    float4 iTime;
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
    float time = params.iTime.x;
    float2 p = uv * 2.0 - 1.0;

    float3 red = float3(1.0, 0.0, 0.0);
    float3 green = float3(0.0, 1.0, 0.0);
    float3 blue = float3(0.0, 0.0, 1.0);
    float3 color = uv.x * red + uv.y * blue + (2.0 - uv.x - uv.y) * green;
    outColor[tid.xy] = float4(color, 1.0);
}
