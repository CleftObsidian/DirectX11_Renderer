#include "Cube/DysonCube.h"

void DysonCube::Update(_In_ FLOAT deltaTime)
{
	if (m_distance > 0.0f)
	{
		m_distance = -3.0f;
	}
	else
	{
		m_distance = 3.0f;
	}
	
	XMMATRIX spin = XMMatrixRotationX(-deltaTime * 20.0f);
	XMMATRIX orbit = XMMatrixRotationZ(-deltaTime * 8.0f);
	XMMATRIX translate = XMMatrixTranslation(m_distance, 0.0f, 0.0f);
	XMMATRIX scale = XMMatrixScaling(0.2f, 0.2f, 0.2f);

	m_world = scale * spin * translate * orbit;
}