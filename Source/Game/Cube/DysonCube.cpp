#include "Cube/DysonCube.h"

DysonCube::DysonCube(const std::filesystem::path& textureFilePath)
	: BaseCube(textureFilePath)
{
	// empty
}

void DysonCube::Update(_In_ FLOAT deltaTime)
{
	// Add time
	static FLOAT s_totalTime = 0.0f;
	s_totalTime += deltaTime;

	// Rotate
	static XMMATRIX s_spin = XMMatrixIdentity();
	s_spin = XMMatrixRotationX(-s_totalTime * 10.0f);

	static XMMATRIX s_orbit = XMMatrixIdentity();
	s_orbit = XMMatrixRotationZ(-s_totalTime * 3.0f);

	static XMMATRIX s_translate = XMMatrixIdentity();
	s_translate = XMMatrixTranslation(XMScalarCos(s_totalTime * 20.0f) + 3.0f, XMScalarSin(s_totalTime), 0.0f);

	static XMMATRIX s_scale = XMMatrixScaling(0.3f, 0.3f, 0.3f);

	m_world = s_scale * s_spin * s_translate * s_orbit;
}