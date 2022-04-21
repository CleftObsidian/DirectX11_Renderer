//--------------------------------------------------------------------------------------
// File: PhongShaders.fx
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
Texture2D txDiffuse : register(t0);
SamplerState samLinear : register(s0);

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
}

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

  Summary:  Constant buffer used for world transformation
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
/*--------------------------------------------------------------------
  TODO: cbChangesEveryFrame definition (remove the comment)
--------------------------------------------------------------------*/
cbuffer cbChangesEveryFrame : register(b2)
{
    matrix World;
    float4 OutputColor;
}

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
  Struct:   VS_PHONG_INPUT

  Summary:  Used as the input to the vertex shader
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
/*--------------------------------------------------------------------
  TODO: VS_PHONG_INPUT definition (remove the comment)
--------------------------------------------------------------------*/
struct VS_PHONG_INPUT
{
    float4 Position : POSITION;
    float2 TexCoord : TEXCOORD0;
    float3 Normal : NORMAL;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Struct:   PS_PHONG_INPUT

  Summary:  Used as the input to the pixel shader, output of the 
            vertex shader
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
/*--------------------------------------------------------------------
  TODO: PS_PHONG_INPUT definition (remove the comment)
--------------------------------------------------------------------*/
struct PS_PHONG_INPUT
{
    float4 Position : SV_POSITION;
    float3 Normal : NORMAL;
    float3 WorldPosition : WORLDPOS;
    float2 TexCoord : TEXCOORD0;
};

/*C+C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C+++C
  Struct:   PS_LIGHT_CUBE_INPUT

  Summary:  Used as the input to the pixel shader, output of the 
            vertex shader
C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C---C-C*/
/*--------------------------------------------------------------------
  TODO: PS_LIGHT_CUBE_INPUT definition (remove the comment)
--------------------------------------------------------------------*/
struct PS_LIGHT_CUBE_INPUT
{
    float4 Position : SV_POSITION;
};

//--------------------------------------------------------------------------------------
// Vertex Shader
//--------------------------------------------------------------------------------------
/*--------------------------------------------------------------------
  TODO: Vertex Shader function VSPhong definition (remove the comment)
--------------------------------------------------------------------*/
PS_PHONG_INPUT VSPhong( VS_PHONG_INPUT input )
{
    PS_PHONG_INPUT output = (PS_PHONG_INPUT)0;
    output.Position = mul( input.Position, World );
    output.Position = mul( output.Position, View );
    output.Position = mul( output.Position, Projection );

    output.Normal = normalize(mul( float4( input.Normal, 1 ), World ).xyz);

    output.WorldPosition = mul( input.Position, World );

    output.TexCoord = input.TexCoord;

    return output;
}

/*--------------------------------------------------------------------
  TODO: Vertex Shader function VSLightCube definition (remove the comment)
--------------------------------------------------------------------*/
PS_LIGHT_CUBE_INPUT VSLightCube( VS_PHONG_INPUT input )
{
    PS_LIGHT_CUBE_INPUT output = (PS_LIGHT_CUBE_INPUT)0;
    output.Position = mul( input.Position, World );
    output.Position = mul( output.Position, View );
    output.Position = mul( output.Position, Projection );

    return output;
}

//--------------------------------------------------------------------------------------
// Pixel Shader
//--------------------------------------------------------------------------------------
/*--------------------------------------------------------------------
  TODO: Pixel Shader function PSPhong definition (remove the comment)
--------------------------------------------------------------------*/
float4 PSPhong( PS_PHONG_INPUT input ) : SV_TARGET
{
    // ambient
    float3 ambient = float3(0.0f, 0.0f, 0.0f);
    for (uint i = 0; i < NUM_LIGHTS; ++i)
    {
        ambient += float4(float3(0.1f, 0.1f, 0.1f) * LightColors[i].xyz, 1.0f);
    }
    ambient *= txDiffuse.Sample(samLinear, input.TexCoord);

    // diffuse
    float3 diffuse = float3(0.0f, 0.0f, 0.0f);
    for (uint j = 0; j < NUM_LIGHTS; ++j)
    {
        float3 lightDirection = normalize(LightPositions[j].xyz - input.WorldPosition);
        diffuse += saturate(dot(normalize(input.Normal), lightDirection)) * LightColors[j];
    }
    diffuse *= txDiffuse.Sample(samLinear, input.TexCoord);

    // specular
    float3 viewDirection = normalize(CameraPosition.xyz - input.WorldPosition);
    float3 specular = float3(0.0f, 0.0f, 0.0f);
    for (uint k = 0; k < NUM_LIGHTS; ++k)
    {
        float3 lightDirection = normalize(LightPositions[k].xyz - input.WorldPosition);
        float3 reflectDirection = reflect(-lightDirection, normalize(input.Normal));
        specular += pow(saturate(dot(reflectDirection, viewDirection)), 4.0f) * LightColors[k];
    }
    specular *= txDiffuse.Sample(samLinear, input.TexCoord);

    return float4(specular, 1.0f);
}

/*--------------------------------------------------------------------
  TODO: Pixel Shader function PSLightCube definition (remove the comment)
--------------------------------------------------------------------*/
float4 PSLightCube( PS_LIGHT_CUBE_INPUT input ) : SV_TARGET
{
    return OutputColor;
}