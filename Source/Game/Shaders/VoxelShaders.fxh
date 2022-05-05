//--------------------------------------------------------------------------------------
// File: VoxelShaders.fx
//
// Copyright (c) Kyung Hee University.
//--------------------------------------------------------------------------------------

#define NUM_LIGHTS (2)

//--------------------------------------------------------------------------------------
// Global Variables
//--------------------------------------------------------------------------------------
/*--------------------------------------------------------------------
  TODO: Declare a diffuse texture and a sampler state (remove the comment)
--------------------------------------------------------------------*/
Texture2D diffuseTexture : register(t0);
SamplerState diffuseSampler : register(s0);

//--------------------------------------------------------------------------------------
// Constant Buffer Variables
//--------------------------------------------------------------------------------------
/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Cbuffer:  cbChangeOnCameraMovement

  Summary:  Constant buffer used for view transformation and shading
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
/*--------------------------------------------------------------------
  TODO: cbChangeOnCameraMovement definition (remove the comment)
--------------------------------------------------------------------*/
cbuffer cbChangeOnCameraMovement : register(b0)
{
    matrix View;
    float4 CameraPosition;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Cbuffer:  cbChangeOnResize

  Summary:  Constant buffer used for projection transformation
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
/*--------------------------------------------------------------------
  TODO: cbChangeOnResize definition (remove the comment)
--------------------------------------------------------------------*/
cbuffer cbChangeOnResize : register(b1)
{
    matrix Projection;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Cbuffer:  cbChangesEveryFrame

  Summary:  Constant buffer used for world transformation, and the 
            color of the voxel
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
/*--------------------------------------------------------------------
  TODO: cbChangesEveryFrame definition (remove the comment)
--------------------------------------------------------------------*/
cbuffer cbChangesEveryFrame : register(b2)
{
    matrix World;
    float4 OutputColor;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Cbuffer:  cbLights

  Summary:  Constant buffer used for shading
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
/*--------------------------------------------------------------------
  TODO: cbLights definition (remove the comment)
--------------------------------------------------------------------*/
cbuffer cbLights : register(b3)
{
    float4 LightPositions[NUM_LIGHTS];
    float4 LightColors[NUM_LIGHTS];
};

//--------------------------------------------------------------------------------------
/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Struct:   VS_INPUT

  Summary:  Used as the input to the vertex shader, 
            instance data included
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
/*--------------------------------------------------------------------
  TODO: VS_INPUT definition (remove the comment)
--------------------------------------------------------------------*/
struct VS_INPUT
{
    float4 Position : POSITION;
    float3 Normal : NORMAL;
    row_major float4x4 Transform : INSTANCE_TRANSFORM;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Struct:   PS_INPUT

  Summary:  Used as the input to the pixel shader, output of the 
            vertex shader
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
/*--------------------------------------------------------------------
  TODO: PS_INPUT definition (remove the comment)
--------------------------------------------------------------------*/
struct PS_INPUT
{
    float4 Position : SV_POSITION;
    float3 Normal : NORMAL;
    float3 WorldPosition : WORLDPOS;
};

//--------------------------------------------------------------------------------------
// Vertex Shader
//--------------------------------------------------------------------------------------
/*--------------------------------------------------------------------
  TODO: Vertex Shader function VSVoxel definition (remove the comment)
--------------------------------------------------------------------*/
PS_INPUT VSVoxel(VS_INPUT input)
{
    PS_INPUT output = (PS_INPUT) 0;
    output.Position = mul(input.Position, input.Transform);
    output.Position = mul(output.Position, World);
    output.WorldPosition = output.Position;
    output.Position = mul(output.Position, View);
    output.Position = mul(output.Position, Projection);
    
    output.Normal = mul(float4(input.Normal, 0.0f), input.Transform).xyz;
    output.Normal = mul(float4(output.Normal, 0.0f), World).xyz;
    
    return output;
}

//--------------------------------------------------------------------------------------
// Pixel Shader
//--------------------------------------------------------------------------------------
/*--------------------------------------------------------------------
  TODO: Pixel Shader function PSVoxel definition (remove the comment)
--------------------------------------------------------------------*/
float4 PSVoxel(PS_INPUT input) : SV_Target
{
    // ambient
    float3 ambient = float3(0.0f, 0.0f, 0.0f);
    for (uint i = 0; i < NUM_LIGHTS; ++i)
    {
        ambient += float4(float3(0.1f, 0.1f, 0.1f) * LightColors[i].xyz, 1.0f);
    }

    // diffuse
    float3 lightDirection = float3(0.0f, 0.0f, 0.0f);
    float3 diffuse = float3(0.0f, 0.0f, 0.0f);
    for (uint j = 0; j < NUM_LIGHTS; ++j)
    {
        lightDirection = normalize(LightPositions[j].xyz - input.WorldPosition);
        diffuse += saturate(dot(normalize(input.Normal), lightDirection)) * LightColors[j];
    }

    // specular
    float3 viewDirection = normalize(CameraPosition.xyz - input.WorldPosition);
    float3 specular = float3(0.0f, 0.0f, 0.0f);
    float3 reflectDirection = float3(0.0f, 0.0f, 0.0f);
    float shiness = 20.0f;
    for (uint k = 0; k < NUM_LIGHTS; ++k)
    {
        lightDirection = normalize(LightPositions[k].xyz - input.WorldPosition);
        reflectDirection = reflect(-lightDirection, normalize(input.Normal));
        specular += pow(saturate(dot(reflectDirection, viewDirection)), shiness) * LightColors[k];
    }
    
    return float4(ambient + diffuse + specular, 1.0f) * OutputColor;
}