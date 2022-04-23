#pragma once

#include "Cube/BaseCube.h"

class SunCube final : public BaseCube
{
public:
    SunCube() = default;
    SunCube(const std::filesystem::path& textureFilePath);
    SunCube(const SunCube& other) = delete;
    SunCube(SunCube&& other) = delete;
    SunCube& operator=(const SunCube& other) = delete;
    SunCube& operator=(SunCube&& other) = delete;
    ~SunCube() = default;

    void Update(_In_ FLOAT deltaTime) override;
};