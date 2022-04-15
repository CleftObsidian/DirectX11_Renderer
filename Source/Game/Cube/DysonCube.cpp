#include "Cube/DysonCube.h"

DysonCube::DysonCube(const std::filesystem::path& textureFilePath)
	: BaseCube(textureFilePath)
	, m_spin()
	, m_orbit()
	, m_translate()
	, m_scale(XMMatrixScaling(0.3f, 0.3f, 0.3f))
{
	// empty
}

void DysonCube::Update(_In_ FLOAT deltaTime)
{
	// Add time
	static FLOAT s_totalTime = 0.0f;
	s_totalTime += deltaTime;

	// Rotate
	m_spin = XMMatrixRotationX(-s_totalTime * 10.0f);
	m_orbit = XMMatrixRotationZ(-s_totalTime * 3.0f);
	m_translate = XMMatrixTranslation(XMScalarCos(s_totalTime * 20.0f) + 3.0f, XMScalarSin(s_totalTime), 0.0f);

	m_world = m_scale * m_spin * m_translate * m_orbit;
}