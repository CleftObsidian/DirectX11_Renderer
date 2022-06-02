#include "RenderTexture.h"

namespace library
{
	/*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
	  Method:   RenderTexture::RenderTexture

	  Summary:  Constructor

	  Modifies: [m_uWidth, m_uHeight, m_texture2D, m_renderTargetView,
				 m_shaderResourceView, m_samplerClamp].
	M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
	/*--------------------------------------------------------------------
	  TODO: RenderTexture::RenderTexture definition (remove the comment)
	--------------------------------------------------------------------*/
	RenderTexture::RenderTexture(_In_ UINT uWidth, _In_ UINT uHeight)
		: m_uWidth(uWidth)
		, m_uHeight(uHeight)
		, m_texture2D()
		, m_renderTargetView()
		, m_shaderResourceView()
		, m_samplerClamp()
	{
		// empty
	}

	/*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
	  Method:   RenderTexture::Initialize

	  Summary:  Initialize

	  Args:     ID3D11Device* pDevice
				ID3D11DeviceContext* pImmediateContext

	  Modifies: [m_texture2D, m_renderTargetView, m_shaderResourceView,
				 m_samplerClamp].

	  Returns:  HRESULT
				  Status code
	M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
	/*--------------------------------------------------------------------
	  TODO: RenderTexture::Initialize definition (remove the comment)
	--------------------------------------------------------------------*/
	HRESULT RenderTexture::Initialize(_In_ ID3D11Device* pDevice, _In_ ID3D11DeviceContext* pImmediateContext)
	{
		HRESULT hr = S_OK;

		// Create a texture2D used as the shadow map
		D3D11_TEXTURE2D_DESC descShadowTexture =
		{
			.Width = m_uWidth,
			.Height = m_uHeight,
			.MipLevels = 1u,
			.ArraySize = 1u,
			.Format = DXGI_FORMAT_R32G32B32A32_FLOAT,
			.SampleDesc = {.Count = 1u},
			.Usage = D3D11_USAGE_DEFAULT,
			.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE,
			.CPUAccessFlags = 0u,
			.MiscFlags = 0u
		};
		hr = pDevice->CreateTexture2D(&descShadowTexture, nullptr, m_texture2D.GetAddressOf());
		if (FAILED(hr))
		{
			MessageBox(
				nullptr,
				L"Call to CreateShadowMapTexture2D failed!",
				L"Game Graphics Programming",
				NULL
			);
			return hr;
		}

		// Create a render target view
		D3D11_RENDER_TARGET_VIEW_DESC descShadowRenderTargetView =
		{
			.Format = descShadowTexture.Format,
			.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D,
			.Texture2D = {.MipSlice = 0}
		};
		hr = pDevice->CreateRenderTargetView(m_texture2D.Get(), &descShadowRenderTargetView, m_renderTargetView.GetAddressOf());
		if (FAILED(hr))
		{
			MessageBox(
				nullptr,
				L"Call to CreateShadowRenderTargetView failed!",
				L"Game Graphics Programming",
				NULL
			);
			return hr;
		}

		// Create a shader resource view
		D3D11_SHADER_RESOURCE_VIEW_DESC descShadowShaderResourceView =
		{
			.Format = descShadowTexture.Format,
			.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D,
			.Texture2D = {.MostDetailedMip = 0,
						  .MipLevels = 1}
		};
		hr = pDevice->CreateShaderResourceView(m_texture2D.Get(), &descShadowShaderResourceView, m_shaderResourceView.GetAddressOf());
		if (FAILED(hr))
		{
			MessageBox(
				nullptr,
				L"Call to CreateShadowShaderResourceView failed!",
				L"Game Graphics Programming",
				NULL
			);
			return hr;
		}

		// Create a sampler state with clamp
		D3D11_SAMPLER_DESC descShadonSamplerState =
		{
			.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR,
			.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP,
			.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP,
			.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP,
			.ComparisonFunc = D3D11_COMPARISON_ALWAYS,
			.MinLOD = 0,
			.MaxLOD = D3D11_FLOAT32_MAX
		};
		hr = pDevice->CreateSamplerState(&descShadonSamplerState, m_samplerClamp.GetAddressOf());
		if (FAILED(hr))
		{
			MessageBox(
				nullptr,
				L"Call to CreateShadowSamplerState failed!",
				L"Game Graphics Programming",
				NULL
			);
			return hr;
		}

		return hr;
	}

	ComPtr<ID3D11Texture2D>& RenderTexture::GetTexture2D()
	{
		return m_texture2D;
	}

	ComPtr<ID3D11RenderTargetView>& RenderTexture::GetRenderTargetView()
	{
		return m_renderTargetView;
	}

	ComPtr<ID3D11ShaderResourceView>& RenderTexture::GetShaderResourceView()
	{
		return m_shaderResourceView;
	}

	ComPtr<ID3D11SamplerState>& RenderTexture::GetSamplerState()
	{
		return m_samplerClamp;
	}
}