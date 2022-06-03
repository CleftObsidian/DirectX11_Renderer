//--------------------------------------------------------------------------------------
// File: PhongShaders.fx
//
// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License (MIT).
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// Constant Buffer Variables
//--------------------------------------------------------------------------------------
#define NUM_LIGHTS (1)
#define NEAR_PLANE (0.01f)
#define FAR_PLANE (1000.0f)

Texture2D diffuseTexture : register(t0);
SamplerState diffuseSamplers : register(s0);

Texture2D normalTexture : register(t1);
SamplerState normalSamplers : register(s1);

Texture2D shadowMapTexture : register(t2);
SamplerState shadowMapSampler : register(s2);

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Cbuffer:  cbChangeOnCameraMovement

  Summary:  Constant buffer used for view transformation and shading
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
cbuffer cbChangeOnCameraMovement : register(b0)
{
    matrix View;
    float4 CameraPosition;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Cbuffer:  cbChangeOnResize

  Summary:  Constant buffer used for projection transformation
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
cbuffer cbChangeOnResize : register(b1)
{
    matrix Projection;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Cbuffer:  cbChangesEveryFrame

  Summary:  Constant buffer used for world transformation
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
cbuffer cbChangesEveryFrame : register(b2)
{
    matrix World;
    float4 OutputColor;
    bool HasNormalMap;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Cbuffer:  cbLights

  Summary:  Constant buffer used for shading
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
cbuffer cbLights : register(b3)
{
    float4 LightPositions[NUM_LIGHTS];
    float4 LightColors[NUM_LIGHTS];
	matrix LightViews[NUM_LIGHTS];
	matrix LightProjections[NUM_LIGHTS];
};

//--------------------------------------------------------------------------------------
/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Struct:   VS_PHONG_INPUT

  Summary:  Used as the input to the vertex shader
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
struct VS_PHONG_INPUT
{
    float4 Position : POSITION;
    float2 TexCoord : TEXCOORD0;
    float3 Normal : NORMAL;
    float3 Tangent : TANGENT;
    float3 Bitangent : BITANGENT;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Struct:   PS_PHONG_INPUT

  Summary:  Used as the input to the pixel shader, output of the 
            vertex shader
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
struct PS_PHONG_INPUT
{
    float4 Position : SV_POSITION;
    float2 TexCoord : TEXCOORD0;
    float3 Normal : NORMAL;
    float3 WorldPosition : WORLDPOS;
    float3 Tangent : TANGENT;
    float3 Bitangent : BITANGENT;
    float4 LightViewPosition : TEXCOORD1;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Struct:   PS_LIGHT_CUBE_INPUT

  Summary:  Used as the input to the pixel shader, output of the 
            vertex shader
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
struct PS_LIGHT_CUBE_INPUT
{
    float4 Position : SV_POSITION;
};

//--------------------------------------------------------------------------------------
// Vertex Shader
//--------------------------------------------------------------------------------------
PS_PHONG_INPUT VSPhong(VS_PHONG_INPUT input)
{
	/*--------------------------------------------------------------------
	  TODO: Vertex shader code (remove the comment)
	--------------------------------------------------------------------*/
    PS_PHONG_INPUT output = (PS_PHONG_INPUT) 0;
    output.Position = mul(input.Position, World);
    output.WorldPosition = output.Position;
    output.LightViewPosition = output.Position;
    output.Position = mul(output.Position, View);
    output.Position = mul(output.Position, Projection);

    output.TexCoord = input.TexCoord;
    
    output.Normal = mul(float4(input.Normal, 0.0f), World).xyz;
    
    if (HasNormalMap)
    {
        output.Tangent = normalize(mul(float4(input.Tangent, 0.0f), World).xyz);
        output.Bitangent = normalize(mul(float4(input.Bitangent, 0.0f), World).xyz);
    }
    
    output.LightViewPosition = mul(output.LightViewPosition, LightViews[0]);
    output.LightViewPosition = mul(output.LightViewPosition, LightProjections[0]);

    return output;
}

PS_LIGHT_CUBE_INPUT VSLightCube(VS_PHONG_INPUT input)
{
    PS_LIGHT_CUBE_INPUT output = (PS_LIGHT_CUBE_INPUT) 0;
    output.Position = mul(input.Position, World);
    output.Position = mul(output.Position, View);
    output.Position = mul(output.Position, Projection);

    return output;
}

float LinearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0;
    return ((2.0 * NEAR_PLANE * FAR_PLANE) / (FAR_PLANE + NEAR_PLANE - z * (FAR_PLANE - NEAR_PLANE))) / FAR_PLANE;
}

//--------------------------------------------------------------------------------------
// Pixel Shader
//--------------------------------------------------------------------------------------
float4 PSPhong(PS_PHONG_INPUT input) : SV_Target
{
	/*--------------------------------------------------------------------
	  TODO: Pixel shader code (remove the comment)
	--------------------------------------------------------------------*/
    float2 depthTexCoord = float2(0.0f, 0.0f);
    depthTexCoord.x = input.LightViewPosition.x / input.LightViewPosition.w / 2.0f + 0.5f;
    depthTexCoord.y = -input.LightViewPosition.y / input.LightViewPosition.w / 2.0f + 0.5f;
    
    float closestDepth = shadowMapTexture.Sample(shadowMapSampler, depthTexCoord).r;
    float currentDepth = input.LightViewPosition.z / input.LightViewPosition.w;
    
    closestDepth = LinearizeDepth(closestDepth);
    currentDepth = LinearizeDepth(currentDepth);
    return float4(closestDepth, closestDepth, closestDepth, 1.0f);
    if (currentDepth > closestDepth + 0.001f)
    {
        float3 ambient = float3(0.0f, 0.0f, 0.0f);
        for (uint i = 0u; i < NUM_LIGHTS; ++i)
        {
            ambient += ambient += float3(0.1f, 0.1f, 0.1f) * LightColors[i].xyz;
        }
        return float4(ambient, 1.0f) * diffuseTexture.Sample(diffuseSamplers, input.TexCoord);
    }
    else // Phong Shading
    {
        float3 normal = normalize(input.Normal);
    
        if (HasNormalMap)
        {
            // Sample the pixel in the normal map
            float4 bumpMap = normalTexture.Sample(normalSamplers, input.TexCoord);
        
            // Expend the range of the normal value from (0, +1) to (-1, +1)
            bumpMap = (bumpMap * 2.0f) - 1.0f;
        
            // Calculate the normal from the data in the normal map
            float3 bumpNormal = (bumpMap.x * input.Tangent) + (bumpMap.y * input.Bitangent) + (bumpMap.z * normal);
        
            // Normalize the resulting bump normal and replace existing normal
            normal = normalize(bumpNormal);
        }
    
        // ambient
        float3 ambient = float3(0.0f, 0.0f, 0.0f);
        for (uint i = 0u; i < NUM_LIGHTS; ++i)
        {
            ambient += ambient += float3(0.1f, 0.1f, 0.1f) * LightColors[i].xyz;
        }

        // diffuse
        float3 lightDirection = float3(0.0f, 0.0f, 0.0f);
        float3 diffuse = float3(0.0f, 0.0f, 0.0f);
        for (uint j = 0u; j < NUM_LIGHTS; ++j)
        {
            lightDirection = normalize(LightPositions[j].xyz - input.WorldPosition);
            diffuse += saturate(dot(normal, lightDirection)) * LightColors[j];
        }

        // specular
        float3 viewDirection = normalize(CameraPosition.xyz - input.WorldPosition);
        float3 specular = float3(0.0f, 0.0f, 0.0f);
        float3 reflectDirection = float3(0.0f, 0.0f, 0.0f);
        float shiness = 20.0f;
        for (uint k = 0; k < NUM_LIGHTS; ++k)
        {
            lightDirection = normalize(LightPositions[k].xyz - input.WorldPosition);
            reflectDirection = reflect(-lightDirection, normal);
            specular += pow(saturate(dot(reflectDirection, viewDirection)), shiness) * LightColors[k];
        }
        
        return float4(ambient + diffuse + specular, 1.0f) * diffuseTexture.Sample(diffuseSamplers, input.TexCoord);
    }
}

float4 PSLightCube(PS_LIGHT_CUBE_INPUT input) : SV_TARGET
{
    return OutputColor;
}