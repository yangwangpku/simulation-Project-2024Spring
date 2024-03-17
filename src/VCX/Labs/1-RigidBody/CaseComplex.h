#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fcl/narrowphase/collision.h>


#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ForceManager.h"
#include "Labs/1-RigidBody/Box.h"

#define BOXNUM 10

namespace VCX::Labs::RigidBody {

    class CaseComplex : public Common::ICase {
    public:
        CaseComplex();
        void GenerateBox();

        virtual std::string_view const GetName() override { return "Complex Scene with multiple boxes"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(glm::vec3 mouseDelta);
        void Advance(float timeDelta);
        void DetectCollision();
        void SolveCollision();

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line on box
        int                                 _boxNum = BOXNUM;
        int                                 _fixedboxNum = 4;
        Box                                 _box[BOXNUM];   // box objects
        fcl::CollisionResult<float>         _collisionResult[BOXNUM][BOXNUM];
        float                               _restitution = 0.9f;
        float                               _gravity = 0.f;
    };
} // namespace VCX::Labs::RigidBody
