// PBR Pixel Shader with Cook-Torrance BRDF

cbuffer MaterialBuffer : register(b1)
{
    float3 albedo;
    float metallic;
    float roughness;
    float ao;
    float2 padding;
};

cbuffer LightBuffer : register(b2)
{
    float3 lightPositions[4];
    float3 lightColors[4];
    float4 cameraPos;
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

static const float PI = 3.14159265359;

// Normal Distribution Function (GGX/Trowbridge-Reitz)
float DistributionGGX(float3 N, float3 H, float roughness)
{
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float nom = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return nom / denom;
}

// Geometry Function (Schlick-GGX)
float GeometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r * r) / 8.0;

    float nom = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return nom / denom;
}

float GeometrySmith(float3 N, float3 V, float3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = GeometrySchlickGGX(NdotV, roughness);
    float ggx1 = GeometrySchlickGGX(NdotL, roughness);

    return ggx1 * ggx2;
}

// Fresnel-Schlick Approximation
float3 FresnelSchlick(float cosTheta, float3 F0)
{
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

float4 main(PS_INPUT input) : SV_TARGET
{
    float3 N = normalize(input.normal);
    float3 V = normalize(cameraPos.xyz - input.worldPos);

    // Calculate reflectance at normal incidence
    // For dielectrics F0 = 0.04, for metals use albedo
    float3 F0 = float3(0.04, 0.04, 0.04);
    F0 = lerp(F0, albedo, metallic);

    // Reflectance equation
    float3 Lo = float3(0.0, 0.0, 0.0);
    for (int i = 0; i < 4; ++i)
    {
        // Calculate per-light radiance
        float3 L = normalize(lightPositions[i] - input.worldPos);
        float3 H = normalize(V + L);
        float distance = length(lightPositions[i] - input.worldPos);
        float attenuation = 1.0 / (distance * distance);
        float3 radiance = lightColors[i] * attenuation;

        // Cook-Torrance BRDF
        float NDF = DistributionGGX(N, H, roughness);
        float G = GeometrySmith(N, V, L, roughness);
        float3 F = FresnelSchlick(max(dot(H, V), 0.0), F0);

        float3 numerator = NDF * G * F;
        float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001; // Prevent divide by zero
        float3 specular = numerator / denominator;

        // kS is equal to Fresnel
        float3 kS = F;
        // For energy conservation, diffuse and specular can't be above 1.0
        float3 kD = float3(1.0, 1.0, 1.0) - kS;
        // Metallic surfaces don't have diffuse lighting
        kD *= 1.0 - metallic;

        float NdotL = max(dot(N, L), 0.0);
        Lo += (kD * albedo / PI + specular) * radiance * NdotL;
    }

    // Ambient lighting (simplified IBL)
    float3 ambient = float3(0.03, 0.03, 0.03) * albedo * ao;

    float3 color = ambient + Lo;

    // HDR tonemapping (Reinhard)
    color = color / (color + float3(1.0, 1.0, 1.0));

    // Gamma correction
    color = pow(color, float3(1.0 / 2.2, 1.0 / 2.2, 1.0 / 2.2));

    return float4(color, 1.0);
}
