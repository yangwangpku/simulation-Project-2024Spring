#pragma once

#include <fcl/narrowphase/collision.h>
#include "Labs/1-RigidBody/Box.h"
#include <iostream>

namespace VCX::Labs::RigidBody {
    void DetectCollision(Box &b0, Box &b1, fcl::CollisionResult<float> &collisionResult);

    void SolveCollision(Box &b0, Box &b1, fcl::CollisionResult<float> &collisionResult, float _restitution);

} // namespace VCX::Labs::RigidBody