struct VSInput {
    float3 in_position : POSITION0;
    float2 in_uv : TEXCOORD0;
    float3 in_normal : NORMAL0;
};

struct PointLight {
    float3 position;
    float intensity;
    float3 color;
    float range;
    float specular_strength;
    float shininess;
    float2 padding;
};

struct PSInput {
    float4 pos : SV_Position;
    float3 color : COLOR0;
    float2 uv : TEXCOORD0;
    float3 normal : NORMAL0;
    float3 world_pos : POSITION1;
};

struct UniformBufferObject {
    row_major float4x4 model;
    row_major float4x4 view;
    row_major float4x4 proj;
    row_major float4x4 normal; // inverse-transpose of model
    float4 base_color;
    PointLight point_lights[4];
    float3 camera_world_pos;
    uint point_light_count;
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
    // Map [-1,1] to [0,1] for visualization (not actively used in blinn-phong, kept for fallback if desired).
    output.color = world_normal * 0.5 + 0.5;
    output.uv = input.in_uv;
    output.normal = world_normal;
    output.world_pos = world_pos.xyz;
    return output;
}

[shader("fragment")]
float4 fragMain(PSInput input) : SV_Target {
    float3 N = normalize(input.normal);
    float3 V = normalize(ubo.camera_world_pos - input.world_pos);
    
    float3 ambient = float3(0.1, 0.1, 0.1);
    float3 lighting = ambient;
    
    for (uint i = 0; i < ubo.point_light_count; ++i) {
        PointLight light = ubo.point_lights[i];
        
        float3 L = light.position - input.world_pos;
        float dist = length(L);
        float3 Ldir = normalize(L);
        
        // Attenuation
        float attenuation = saturate(1.0 - dist / light.range);
        attenuation = (attenuation * attenuation) / (1.0 + dist * dist);
        
        // Diffuse
        float diff = max(dot(N, Ldir), 0.0);
        float3 diffuse = diff * light.color * light.intensity;
        
        // Specular (Blinn-Phong)
        float3 H = normalize(Ldir + V);
        float spec = pow(max(dot(N, H), 0.0), light.shininess);
        float3 specular = light.specular_strength * spec * light.color * light.intensity;
        
        lighting += (diffuse + specular) * attenuation;
    }
    
    float3 final_color = lighting * ubo.base_color.rgb;
    return float4(final_color, ubo.base_color.a);
}
