#pragma once

#include "Cube/BaseCube.h"

class DysonCube final : public BaseCube
{
public:
    DysonCube() = default;
    DysonCube(const DysonCube & other) = delete;
    DysonCube(DysonCube && other) = delete;
    DysonCube& operator=(const DysonCube & other) = delete;
    DysonCube& operator=(DysonCube && other) = delete;
    ~DysonCube() = default;

    void Update(_In_ FLOAT deltaTime) override;

private:
    FLOAT m_distance;
};