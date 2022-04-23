#include "Cube/SunCube.h"

SunCube::SunCube(const std::filesystem::path& textureFilePath)
	: BaseCube(textureFilePath)
{
	// empty
}

void SunCube::Update(_In_ FLOAT deltaTime)
{
	// Does nothing
}