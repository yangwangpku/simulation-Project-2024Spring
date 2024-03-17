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
#include "Labs/1-RigidBody/Box.h"

namespace VCX::Labs::RigidBody {

    class CaseSingleBody : public Common::ICase {
    public:
        CaseSingleBody();

        virtual std::string_view const GetName() override { return "Single Box"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(std::pair<glm::vec3,glm::vec3> force);
        
        void Advance(float timeDelta);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        Common::ForceManager                _forceManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line on box
        Box                                 _box;   // box object
    };
} // namespace VCX::Labs::RigidBody
