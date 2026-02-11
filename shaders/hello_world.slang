[[vk_binding(0, 0)]]
StructuredBuffer<float> buffer0;

[[vk_binding(1, 0)]]
StructuredBuffer<float> buffer1;

[[vk_binding(2, 0)]]
RWStructuredBuffer<float> result;

[shader("compute")]
[numthreads(1, 1, 1)]
void computeMain(uint3 threadId : SV_DispatchThreadID)
{
    uint index = threadId.x;
    result[index] = buffer0[index] + buffer1[index];
}
