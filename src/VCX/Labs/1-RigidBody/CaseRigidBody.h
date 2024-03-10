#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ForceManager.h"

namespace VCX::Labs::RigidBody {

    class CaseRigidBody : public Common::ICase {
    public:
        CaseRigidBody();

        virtual std::string_view const GetName() override { return "Simple RigidBody Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(std::pair<glm::vec3,glm::vec3> force);
        
        void Advance(float timeDelta);

        Eigen::Matrix3f GetInertiaMatrix() {
        // Using the formula from the image to calculate the inertia matrix Iref
        Eigen::Matrix3f Iref;
        float m = _mass;
        float w = _dim.x();
        float h = _dim.y();
        float d = _dim.z();
        
        Iref << 1.0f / 12.0f * m * (h * h + d * d), 0, 0,
                 0, 1.0f / 12.0f * m * (d * d + w * w), 0,
                 0, 0, 1.0f / 12.0f * m * (w * w + h * h);
        
        // Rotate the inertia matrix according to the current orientation of the box
        Eigen::Matrix3f R = _orientation.toRotationMatrix();
        Eigen::Matrix3f inertiaMatrix = R * Iref * R.transpose();

        return inertiaMatrix;
    }

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        Common::ForceManager                _forceManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line on box
        glm::vec3                           _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        Eigen::Vector3f                     _dim { 1.f, 2.f, 3.f };

        Eigen::Quaternionf                  _orientation {1.0f, 0.0f, 0.0f, 0.0f}; // Quaternion for orientation
        // Eigen::Quaternionf                  _orientation {0.9f, 0.3f, 0.3f, 0.1f}; // Quaternion for orientation
        Eigen::Vector3f                     _angularVelocity {0.0f, 0.0f, 0.0f}; // Angular velocity
        // Eigen::Vector3f                     _angularVelocity {1.0f, 0.0f, 0.0f}; // Angular velocity
        // Eigen::Vector3f                     _velocity { 0.1f, 0.1f, 0.1f };
        Eigen::Vector3f                     _velocity { 0.f, 0.f, 0.f };
        Eigen::Vector3f                     _center { 0.f, 0.f, 0.f };

        float                               _mass{1.f};



    };
} // namespace VCX::Labs::RigidBody
