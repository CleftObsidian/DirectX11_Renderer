#include "Cube/DysonCube.h"

void DysonCube::Update(_In_ FLOAT deltaTime)
{
	m_distance = static_cast<FLOAT>(cos(deltaTime * XM_PI * 5.0f) + 3.0f);
	
	XMMATRIX spin = XMMatrixRotationX(-deltaTime * 20.0f);
	XMMATRIX orbit = XMMatrixRotationZ(-deltaTime * 2.0f);
	XMMATRIX translate = XMMatrixTranslation(m_distance, 0.0f, 0.0f);
	XMMATRIX scale = XMMatrixScaling(0.2f, 0.2f, 0.2f);

	m_world = scale * spin * translate * orbit;
}