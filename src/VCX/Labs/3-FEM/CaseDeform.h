#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>


#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/3-FEM/TetSystem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ForceManager.h"

namespace VCX::Labs::FEM {

    class CaseDeform : public Common::ICase {
    public:
        CaseDeform();

        virtual std::string_view const GetName() override { return "Deformable Object"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(std::pair<glm::vec3,int> force);
        void ResetSystem();
        
        // void Advance(float timeDelta);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _tetItem;  // render the tet
        FEM::Simulator                      _tetSystem;
        Common::ForceManager                _forceManager;
    };
} // namespace VCX::Labs::RigidBody
