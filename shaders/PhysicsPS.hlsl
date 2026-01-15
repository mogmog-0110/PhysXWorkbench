// Pixel Shader for PhysX Visualization

cbuffer ConstantBuffer : register(b0)
{
    matrix worldViewProj;
    matrix world;
    float4 lightDir;
    float4 cameraPos;
};

struct PS_INPUT
{
    float4 position : SV_POSITION;
    float3 worldPos : POSITION;
    float3 normal : NORMAL;
    float3 color : COLOR;
};

float4 main(PS_INPUT input) : SV_TARGET
{
    // Normalize interpolated normal
    float3 normal = normalize(input.normal);

    // Calculate diffuse lighting
    float3 lightDirection = normalize(-lightDir.xyz);
    float diffuse = max(dot(normal, lightDirection), 0.0f);

    // Calculate specular lighting (Blinn-Phong)
    float3 viewDir = normalize(cameraPos.xyz - input.worldPos);
    float3 halfDir = normalize(lightDirection + viewDir);
    float specular = pow(max(dot(normal, halfDir), 0.0f), 32.0f);

    // Ambient lighting
    float3 ambient = float3(0.2f, 0.2f, 0.2f);

    // Combine lighting
    float3 finalColor = input.color * (ambient + diffuse * 0.7f) + specular * 0.3f;

    return float4(finalColor, 1.0f);
}
