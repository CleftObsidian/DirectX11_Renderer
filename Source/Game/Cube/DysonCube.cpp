#include "Cube/DysonCube.h"

void DysonCube::Update(_In_ FLOAT deltaTime)
{
	// Add time
	if (XMMatrixIsIdentity(m_world))
	{
		m_totalTime = 0.0f;
	}
	m_totalTime += deltaTime;

	// Rotate
	m_distance = static_cast<FLOAT>(cos(m_totalTime * 20.0f) + 3.0f);
	
	XMMATRIX spin = XMMatrixRotationX(-m_totalTime * 10.0f);
	XMMATRIX orbit = XMMatrixRotationZ(-m_totalTime * 3.0f);
	XMMATRIX translate = XMMatrixTranslation(m_distance, 0.0f, 0.0f);
	XMMATRIX scale = XMMatrixScaling(0.2f, 0.2f, 0.2f);

	m_world = scale * spin * translate * orbit;
}