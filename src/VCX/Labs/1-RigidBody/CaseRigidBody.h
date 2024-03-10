#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::RigidBody {

    class CaseRigidBody : public Common::ICase {
    public:
        CaseRigidBody();

        virtual std::string_view const GetName() override { return "Simple RigidBody Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(glm::vec3 mourseDelta);
        
        void Advance(float timeDelta);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line on box
        glm::vec3                     _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };

        Eigen::Quaternionf                  _orientation {1.0f, 0.0f, 0.0f, 0.0f}; // Quaternion for orientation
        // Eigen::Quaternionf                  _orientation {0.9f, 0.3f, 0.3f, 0.1f}; // Quaternion for orientation
        // Eigen::Vector3f                     _angularVelocity {0.0f, 0.0f, 0.0f}; // Angular velocity
        Eigen::Vector3f                     _angularVelocity {1.0f, 0.0f, 0.0f}; // Angular velocity
        // Eigen::Vector3f                     _velocity { 0.1f, 0.1f, 0.1f };
        Eigen::Vector3f                     _velocity { 0.f, 0.f, 0.f };
        Eigen::Vector3f                     _center { 0.f, 0.f, 0.f };

        Eigen::Vector3f                     _dim { 1.f, 2.f, 3.f };
    };
} // namespace VCX::Labs::RigidBody
