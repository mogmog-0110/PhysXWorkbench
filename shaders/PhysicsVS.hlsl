// Vertex Shader for PhysX Visualization

cbuffer ConstantBuffer : register(b0)
{
    matrix worldViewProj;
    matrix world;
    float4 lightDir;
    float4 cameraPos;
};

struct VS_INPUT
{
    float3 position : POSITION;
    float3 normal : NORMAL;
    float3 color : COLOR;
};

struct PS_INPUT
{
    float4 position : SV_POSITION;
    float3 worldPos : POSITION;
    float3 normal : NORMAL;
    float3 color : COLOR;
};

PS_INPUT main(VS_INPUT input)
{
    PS_INPUT output;

    // Transform position to clip space
    output.position = mul(float4(input.position, 1.0f), worldViewProj);

    // Transform position to world space for lighting
    output.worldPos = mul(float4(input.position, 1.0f), world).xyz;

    // Transform normal to world space
    output.normal = mul(input.normal, (float3x3)world);

    // Pass through color
    output.color = input.color;

    return output;
}
