// PBR Vertex Shader

cbuffer ConstantBuffer : register(b0)
{
    matrix worldViewProj;
    matrix world;
    matrix worldInvTranspose;
    float4 cameraPos;
};

struct VS_INPUT
{
    float3 position : POSITION;
    float3 normal : NORMAL;
    float2 texCoord : TEXCOORD;
    float3 tangent : TANGENT;
};

struct PS_INPUT
{
    float4 position : SV_POSITION;
    float3 worldPos : POSITION;
    float3 normal : NORMAL;
    float2 texCoord : TEXCOORD;
    float3 tangent : TANGENT;
    float3 bitangent : BITANGENT;
};

PS_INPUT main(VS_INPUT input)
{
    PS_INPUT output;

    // Transform to clip space
    output.position = mul(float4(input.position, 1.0f), worldViewProj);

    // World position
    output.worldPos = mul(float4(input.position, 1.0f), world).xyz;

    // Transform normal to world space
    output.normal = mul(input.normal, (float3x3)worldInvTranspose);

    // Transform tangent to world space
    output.tangent = mul(input.tangent, (float3x3)world);

    // Calculate bitangent
    output.bitangent = cross(output.normal, output.tangent);

    // Pass through texture coordinates
    output.texCoord = input.texCoord;

    return output;
}
