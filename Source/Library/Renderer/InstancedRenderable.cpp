#include "Renderer/InstancedRenderable.h"

namespace library
{
    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   InstancedRenderable::InstancedRenderable

      Summary:  Constructor

      Args:     const XMFLOAT4& outputColor
                  Default color of the renderable
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    /*--------------------------------------------------------------------
      TODO: InstancedRenderable::InstancedRenderable definition (remove the comment)
    --------------------------------------------------------------------*/
    InstancedRenderable::InstancedRenderable(_In_ const XMFLOAT4& outputColor)
        : Renderable(outputColor)
        , m_instanceBuffer(nullptr)
        , m_aInstanceData(std::vector<InstanceData>())
        , m_padding()
    {
        // empty
    }

    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   InstancedRenderable::InstancedRenderable

      Summary:  Constructor

      Args:     std::vector<InstanceData>&& aInstanceData
                  An instance data
                const XMFLOAT4& outputColor
                  Default color of the renderable

      Modifies: [m_instanceBuffer, m_aInstanceData].
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    /*--------------------------------------------------------------------
      TODO: InstancedRenderable::InstancedRenderable definition (remove the comment)
    --------------------------------------------------------------------*/
    InstancedRenderable::InstancedRenderable(_In_ std::vector<InstanceData>&& aInstanceData, _In_ const XMFLOAT4& outputColor)
        : Renderable(outputColor)
        , m_instanceBuffer(nullptr)
        , m_aInstanceData(std::move(aInstanceData))
        , m_padding()
    {
        // empty
    }

    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   InstancedRenderable::SetInstanceData

      Summary:  Sets the instance data

      Args:     std::vector<InstanceData>&& aInstanceData
                  Instance data

      Modifies: [m_aInstanceData].
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    /*--------------------------------------------------------------------
      TODO: InstancedRenderable::SetInstanceData definition (remove the comment)
    --------------------------------------------------------------------*/
    void InstancedRenderable::SetInstanceData(std::vector<InstanceData>&& aInstanceData)
    {
        m_aInstanceData = std::move(aInstanceData);
    }

    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   InstancedRenderable::GetInstanceBuffer

      Summary:  Returns the instance buffer

      Returns:  ComPtr<ID3D11Buffer>&
                  Instance buffer
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    /*--------------------------------------------------------------------
      TODO: InstancedRenderable::GetInstanceBuffer definition (remove the comment)
    --------------------------------------------------------------------*/
    ComPtr<ID3D11Buffer>& InstancedRenderable::GetInstanceBuffer()
    {
        return m_instanceBuffer;
    }

    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   InstancedRenderable::GetNumInstances

      Summary:  Returns the number of instances

      Returns:  UINT
                  Number of instances
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    /*--------------------------------------------------------------------
      TODO: InstancedRenderable::GetNumInstances definition (remove the comment)
    --------------------------------------------------------------------*/
    UINT InstancedRenderable::GetNumInstances() const
    {
        return static_cast<UINT>(m_aInstanceData.size());
    }

    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   InstancedRenderable::initializeInstance

      Summary:  Creates an instance buffer

      Args:     ID3D11Device* pDevice
                  Pointer to a Direct3D 11 device

      Modifies: [m_instanceBuffer].

      Returns:  HRESULT
                  Status code
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    /*--------------------------------------------------------------------
      TODO: InstancedRenderable::initializeInstance definition (remove the comment)
    --------------------------------------------------------------------*/
    HRESULT InstancedRenderable::initializeInstance(_In_ ID3D11Device* pDevice)
    {
        HRESULT hr = S_OK;

        // Create the instance buffer
        D3D11_BUFFER_DESC iBufferDesc =
        {
            .ByteWidth = sizeof(InstanceData) * m_aInstanceData.size(),
            .Usage = D3D11_USAGE_DEFAULT,
            .BindFlags = D3D11_BIND_VERTEX_BUFFER,
            .CPUAccessFlags = 0
        };
        D3D11_SUBRESOURCE_DATA iInitData =
        {
            .pSysMem = &m_aInstanceData
        };
        hr = pDevice->CreateBuffer(&iBufferDesc, &iInitData, &m_instanceBuffer);
        if (FAILED(hr))
        {
            MessageBox(
                nullptr,
                L"Call to CreateInstanceBuffer failed!",
                L"Game Graphics Programming",
                NULL
            );
            return hr;
        }

        return hr;
    }
}