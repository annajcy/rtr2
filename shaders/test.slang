//test.slang

float2 getPosition(uint index) {
    switch (index) {
    case 0: return float2(0.0, -0.5);
    case 1: return float2(0.5, 0.5);
    default: return float2(-0.5, 0.5);
    }
}

float3 getColor(uint index) {
    switch (index) {
    case 0: return float3(1.0, 0.0, 0.0);
    case 1: return float3(0.0, 1.0, 0.0);
    default: return float3(0.0, 0.0, 1.0);
    }
}

struct VSInput {
    uint vid : SV_VertexID;
};

struct PSInput {
    float4 pos : SV_Position;
    float3 color : COLOR0;
};

[shader("vertex")]
PSInput vertMain(VSInput input) {
    PSInput output;
    float2 pos = getPosition(input.vid);
    output.pos = float4(pos, 0.0, 1.0);
    output.color = getColor(input.vid);
    return output;
}

[shader("fragment")]
float4 fragMain(PSInput input) : SV_Target {
    return float4(input.color, 1.0);
}
