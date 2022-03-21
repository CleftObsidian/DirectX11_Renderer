#include "Game/Game.h"

namespace library
{
    /*--------------------------------------------------------------------
      Global Variables
    --------------------------------------------------------------------*/

    HINSTANCE                                       g_hInst = nullptr;
    HWND                                            g_hWnd = nullptr;
    D3D_DRIVER_TYPE                                 g_driverType = D3D_DRIVER_TYPE_NULL;
    D3D_FEATURE_LEVEL                               g_featureLevel = D3D_FEATURE_LEVEL_11_0;
    Microsoft::WRL::ComPtr<ID3D11Device>            g_pD3dDevice = nullptr;
    Microsoft::WRL::ComPtr<ID3D11Device1>           g_pD3dDevice1 = nullptr;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext>     g_pImmediateContext = nullptr;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext1>    g_pImmediateContext1 = nullptr;
    Microsoft::WRL::ComPtr<IDXGISwapChain>          g_pSwapChain = nullptr;
    Microsoft::WRL::ComPtr<IDXGISwapChain1>         g_pSwapChain1 = nullptr;
    Microsoft::WRL::ComPtr<ID3D11RenderTargetView>  g_pRenderTargetView = nullptr;

    /*--------------------------------------------------------------------
      Forward declarations
    --------------------------------------------------------------------*/

    /*F+F+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      Function: WindowProc

      Summary:  Defines the behavior of the window—its appearance, how
                it interacts with the user, and so forth

      Args:     HWND hWnd
                  Handle to the window
                UINT uMsg
                  Message code
                WPARAM wParam
                  Additional data that pertains to the message
                LPARAM lParam
                  Additional data that pertains to the message

      Returns:  LRESULT
                  Integer value that your program returns to Windows
    -----------------------------------------------------------------F-F*/
    LRESULT CALLBACK WindowProc(_In_ HWND hWnd, _In_ UINT uMsg, _In_ WPARAM wParam, _In_ LPARAM lParam)
    {
        PAINTSTRUCT ps;
        HDC hdc;

        switch (uMsg)
        {
        case WM_PAINT:
            hdc = BeginPaint(hWnd, &ps);
            EndPaint(hWnd, &ps);
            break;

        case WM_DESTROY:
            PostQuitMessage(0);
            break;

        case WM_CLOSE:
            if (MessageBox(hWnd,
                L"Really quit?",
                L"Game Graphics Programming",
                MB_OKCANCEL) == IDOK)
            {
                DestroyWindow(hWnd);
            }
            // Else: user canceled. Do nothing.
            return S_OK;

        default:
            return DefWindowProc(hWnd, uMsg, wParam, lParam);
        }

        return S_OK;
    }

    HRESULT InitWindow(_In_ HINSTANCE hInstance, _In_ INT nCmdShow)
    {
        // Register class
        WNDCLASSEX wcex;

        wcex.cbSize = static_cast<UINT> (sizeof(WNDCLASSEX));
        wcex.style = static_cast<UINT> (CS_HREDRAW | CS_VREDRAW);
        wcex.lpfnWndProc = WindowProc;
        wcex.cbClsExtra = 0;
        wcex.cbWndExtra = 0;
        wcex.hInstance = hInstance;
        wcex.hIcon = LoadIcon(hInstance, reinterpret_cast<LPCTSTR> (IDI_TUTORIAL1));
        wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
        wcex.hbrBackground = reinterpret_cast<HBRUSH> (COLOR_WINDOW + 1);
        wcex.lpszMenuName = nullptr;
        wcex.lpszClassName = L"GameGraphicsProgramming";
        wcex.hIconSm = LoadIcon(wcex.hInstance, reinterpret_cast<LPCTSTR> (IDI_TUTORIAL1));

        if (!RegisterClassEx(&wcex))
        {
            DWORD dwError = GetLastError();

            MessageBox(
                nullptr,
                L"Call to RegisterClassEx failed!",
                L"Game Graphics Programming",
                NULL
            );

            if (dwError != ERROR_CLASS_ALREADY_EXISTS)
            {
                return HRESULT_FROM_WIN32(dwError);
            }

            return E_FAIL;
        }

        // Create window
        g_hInst = hInstance;
        RECT rc = { 0, 0, 800, 600 };
        AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
        g_hWnd = CreateWindow(L"GameGraphicsProgramming", L"Game Graphics Programming Lab 01: Direct3D 11 Basics",
                              WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
                              CW_USEDEFAULT, CW_USEDEFAULT, rc.right - rc.left, rc.bottom - rc.top, nullptr, nullptr, hInstance,
                              nullptr);
        if (!g_hWnd)
        {
            DWORD dwError = GetLastError();

            MessageBox(
                nullptr,
                L"Call to CreateWindowEx failed!",
                L"Game Graphics Programming",
                NULL
            );

            if (dwError != ERROR_CLASS_ALREADY_EXISTS)
            {
                return HRESULT_FROM_WIN32(dwError);
            }

            return E_FAIL;
        }

        ShowWindow(g_hWnd, nCmdShow);

        return S_OK;
    }

    HRESULT InitDevice()
    {
        HRESULT hr = S_OK;

        RECT rc;
        GetClientRect(g_hWnd, &rc);
        UINT width = static_cast<UINT> (rc.right - rc.left);
        UINT height = static_cast<UINT> (rc.bottom - rc.top);

        UINT createDeviceFlags = 0u;
#ifdef _DEBUG
        createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif // _DEBUG

        D3D_DRIVER_TYPE driverTypes[] =
        {
            D3D_DRIVER_TYPE_HARDWARE,
            D3D_DRIVER_TYPE_WARP,
            D3D_DRIVER_TYPE_REFERENCE,
        };
        UINT numDriverTypes = ARRAYSIZE(driverTypes);

        D3D_FEATURE_LEVEL featureLevels[] =
        {
            D3D_FEATURE_LEVEL_11_1,
            D3D_FEATURE_LEVEL_11_0,
            D3D_FEATURE_LEVEL_10_1,
            D3D_FEATURE_LEVEL_10_0,
        };
        UINT numFeatureLevels = ARRAYSIZE(featureLevels);

        for (UINT driverTypeIndex = 0u; driverTypeIndex < numDriverTypes; ++driverTypeIndex)
        {
            g_driverType = driverTypes[driverTypeIndex];
            hr = D3D11CreateDevice(nullptr, g_driverType, nullptr, createDeviceFlags, featureLevels, numFeatureLevels,
                D3D11_SDK_VERSION, g_pD3dDevice.GetAddressOf(), &g_featureLevel, g_pImmediateContext.GetAddressOf());

            if (hr == E_INVALIDARG)
            {
                // DirectX 11.0 platforms will not recognize D3D_FEATURE_LEVEL_11_1 so we need to retry without it
                hr = D3D11CreateDevice(nullptr, g_driverType, nullptr, createDeviceFlags, &featureLevels[1], numFeatureLevels - 1,
                    D3D11_SDK_VERSION, g_pD3dDevice.GetAddressOf(), &g_featureLevel, g_pImmediateContext.GetAddressOf());
            }

            if (SUCCEEDED(hr))
            {
                break;
            }
        }
        if (FAILED(hr))
        {
            MessageBox(
                nullptr,
                L"Call to D3D11CreateDevice failed!",
                L"Game Graphics Programming",
                NULL
            );
            return hr;
        }

        // Obtain DXGI factory from device
        Microsoft::WRL::ComPtr<IDXGIFactory1> dxgiFactory = nullptr;
        {
            Microsoft::WRL::ComPtr<IDXGIDevice> dxgiDevice = nullptr;
            if (SUCCEEDED(g_pD3dDevice.As(&dxgiDevice)))
            {
                Microsoft::WRL::ComPtr<IDXGIAdapter> adapter = nullptr;
                if (SUCCEEDED(dxgiDevice->GetAdapter(&adapter)))
                {
                    hr = adapter->GetParent(IID_PPV_ARGS(dxgiFactory.GetAddressOf()));
                    // adapter->Release();
                }
                // dxgiDevice->Release();
            }
        }
        if (FAILED(hr))
        {
            MessageBox(
                nullptr,
                L"Call to DXGI factory setting failed!",
                L"Game Graphics Programming",
                NULL
            );
            return hr;
        }

        // Create swap chain
        Microsoft::WRL::ComPtr<IDXGIFactory2> dxgiFactory2 = nullptr;
        if (SUCCEEDED(dxgiFactory.As(&dxgiFactory2)))
        {
            // DirectX 11.1 or later
            if (SUCCEEDED(g_pD3dDevice.As(&g_pD3dDevice1)))
            {
                (void)g_pImmediateContext.As(&g_pImmediateContext1);
            }

            DXGI_SWAP_CHAIN_DESC1 sd = {};
            sd.Width = width;
            sd.Height = height;
            sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
            sd.SampleDesc.Count = 1u;
            sd.SampleDesc.Quality = 0u;
            sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
            sd.BufferCount = 1;

            hr = dxgiFactory2->CreateSwapChainForHwnd(g_pD3dDevice.Get(), g_hWnd, &sd, nullptr, nullptr, &g_pSwapChain1);
            if (SUCCEEDED(hr))
            {
                hr = g_pSwapChain1.As(&g_pSwapChain);
            }

            // dxgiFactory2->Release();
        }
        else
        {
            // DirectX 11.0 systems
            DXGI_SWAP_CHAIN_DESC sd = {};
            sd.BufferCount = 1u;
            sd.BufferDesc.Width = width;
            sd.BufferDesc.Height = height;
            sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
            sd.BufferDesc.RefreshRate.Numerator = 60u;
            sd.BufferDesc.RefreshRate.Denominator = 1u;
            sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
            sd.OutputWindow = g_hWnd;
            sd.SampleDesc.Count = 1u;
            sd.SampleDesc.Quality = 0u;
            sd.Windowed = TRUE;

            hr = dxgiFactory->CreateSwapChain(g_pD3dDevice.Get(), &sd, &g_pSwapChain);
        }

        // Note this tutorial doesn't handle full-screen swapchains so we block the ALT+ENTER shortcut
        dxgiFactory->MakeWindowAssociation(g_hWnd, DXGI_MWA_NO_ALT_ENTER);

        // dxgiFactory->Release();

        if (FAILED(hr))
        {
            MessageBox(
                nullptr,
                L"Call to CreateSwapChain failed!",
                L"Game Graphics Programming",
                NULL
            );
            return hr;
        }

        // Create a render target view
        Microsoft::WRL::ComPtr<ID3D11Texture2D> pBackBuffer = nullptr;
        hr = g_pSwapChain->GetBuffer(0u, IID_PPV_ARGS(&pBackBuffer));
        if (FAILED(hr))
        {
            MessageBox(
                nullptr,
                L"Call to GetBuffer failed!",
                L"Game Graphics Programming",
                NULL
            );
            return hr;
        }

        hr = g_pD3dDevice->CreateRenderTargetView(pBackBuffer.Get(), nullptr, g_pRenderTargetView.GetAddressOf());
        // pBackBuffer->Release();
        if (FAILED(hr))
        {
            MessageBox(
                nullptr,
                L"Call to CreateRenderTargetView failed!",
                L"Game Graphics Programming",
                NULL
            );
            return hr;
        }

        g_pImmediateContext->OMSetRenderTargets(1, g_pRenderTargetView.GetAddressOf(), nullptr);

        // Setup the viewport
        D3D11_VIEWPORT vp;
        vp.Width = static_cast<FLOAT> (width);
        vp.Height = static_cast<FLOAT> (height);
        vp.MinDepth = 0.0f;
        vp.MaxDepth = 1.0f;
        vp.TopLeftX = 0.0f;
        vp.TopLeftY = 0.0f;
        g_pImmediateContext->RSSetViewports(1u, &vp);

        return S_OK;
    }

    void CleanupDevice()
    {
        if (g_pImmediateContext) g_pImmediateContext->ClearState();

        /*if (g_pRenderTargetView) g_pRenderTargetView->Release();
        if (g_pSwapChain1) g_pSwapChain1->Release();
        if (g_pSwapChain) g_pSwapChain->Release();
        if (g_pImmediateContext1) g_pImmediateContext1->Release();
        if (g_pImmediateContext) g_pImmediateContext->Release();
        if (g_pD3dDevice1) g_pD3dDevice1->Release();
        if (g_pD3dDevice) g_pD3dDevice->Release();*/
    }

    void Render()
    {
        // Just clear the backbuffer
        g_pImmediateContext->ClearRenderTargetView(g_pRenderTargetView.Get(), Colors::MidnightBlue);
        g_pSwapChain->Present(0u, 0u);
    }
}