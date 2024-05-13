#include <spdlog/spdlog.h>
#include "Engine/app.h"
#include "Labs/2-FluidSimulation/CaseFluid.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>

namespace VCX::Labs::FluidSimulation {
    const std::vector<glm::vec3> vertex_pos = {
            glm::vec3(-0.5f, -0.5f, -0.5f),
            glm::vec3(0.5f, -0.5f, -0.5f),  
            glm::vec3(0.5f, 0.5f, -0.5f),  
            glm::vec3(-0.5f, 0.5f, -0.5f), 
            glm::vec3(-0.5f, -0.5f, 0.5f),  
            glm::vec3(0.5f, -0.5f, 0.5f),   
            glm::vec3(0.5f, 0.5f, 0.5f),   
            glm::vec3(-0.5f, 0.5f, 0.5f)
    };
    const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index

    CaseFluid::CaseFluid(std::initializer_list<Assets::ExampleScene> && scenes) :
        _scenes(scenes),
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/fluid.vert"),
                Engine::GL::SharedShader("assets/shaders/fluid.frag") })),
        _lineprogram(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _sceneObject(1),
        _BoundaryItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Lines){ 
        _cameraManager.AutoRotate = false;
        _program.BindUniformBlock("PassConstants", 1);
        _program.GetUniforms().SetByName("u_DiffuseMap" , 0);
        _program.GetUniforms().SetByName("u_SpecularMap", 1);
        _program.GetUniforms().SetByName("u_HeightMap"  , 2);

        _lineprogram.GetUniforms().SetByName("u_Color",  glm::vec3(1.0f));
        _BoundaryItem.UpdateElementBuffer(line_index);
        ResetSystem();
        _sphere = Engine::Model(Engine::Sphere(6,_r));
    }

    void CaseFluid::OnSetupPropsUI() {
        if(ImGui::Button("Reset System")) 
            ResetSystem();
        ImGui::SameLine();
        if(ImGui::Button(_stopped ? "Start Simulation":"Stop Simulation"))
            _stopped = ! _stopped;
        ImGui::Spacing();
        ImGui::Checkbox("separateParticles", &_simulation.separateParticles);
        ImGui::Checkbox("compensateDrift", &_simulation.compensateDrift);
        ImGui::SliderFloat("Flip Ratio", &_simulation.m_fRatio, 0.0f, 1.0f);
        ImGui::SliderFloat("compensateDriftWeight", &_simulation.compensateDriftWeight, 0.0f, 1.0f);
        ImGui::SliderFloat("overRelaxation", &_simulation.overRelaxation, 0.3f, 2.0f);
        ImGui::SliderInt("numPressureIters", &_simulation.numPressureIters, 5,1000);
        ImGui::SliderInt("numParticleIters", &_simulation.numParticleIters, 3,10);

        ImGui::SliderFloat("obstacleVel.x", &_simulation.obstacleVel.x, -0.5f, 0.5f);
        ImGui::SliderFloat("obstacleVel.y", &_simulation.obstacleVel.y, -0.5f, 0.5f);
        ImGui::SliderFloat("obstacleVel.z", &_simulation.obstacleVel.z, -0.5f, 0.5f);
        ImGui::SliderFloat("obstacleRadius", &_simulation.obstacleRadius, 0.0f, 0.5f);

    }


    Common::CaseRenderResult CaseFluid::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            _sceneObject.ReplaceScene(GetScene(_sceneIdx));
            _cameraManager.Save(_sceneObject.Camera);
        }
        OnProcessMouseControl(_cameraManager.getMouseMove());
        if (! _stopped) _simulation.SimulateTimestep(Engine::GetDeltaTime());
        
        _BoundaryItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(vertex_pos));
        _frame.Resize(desiredSize);

        _cameraManager.Update(_sceneObject.Camera);
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::Projection, _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::View, _sceneObject.Camera.GetViewMatrix());
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::ViewPosition, _sceneObject.Camera.Eye);
        _lineprogram.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _lineprogram.GetUniforms().SetByName("u_View"      , _sceneObject.Camera.GetViewMatrix());
        
        if (_uniformDirty) {
            _uniformDirty = false;
            _program.GetUniforms().SetByName("u_AmbientScale"      , _ambientScale);
            _program.GetUniforms().SetByName("u_UseBlinn"          , _useBlinn);
            _program.GetUniforms().SetByName("u_Shininess"         , _shininess);
            _program.GetUniforms().SetByName("u_UseGammaCorrection", int(_useGammaCorrection));
            _program.GetUniforms().SetByName("u_AttenuationOrder"  , _attenuationOrder);            
            _program.GetUniforms().SetByName("u_BumpMappingBlend"  , _bumpMappingPercent * .01f);            

        }
        
        gl_using(_frame);

        glEnable(GL_DEPTH_TEST);
        glLineWidth(_BndWidth);
        _BoundaryItem.Draw({ _lineprogram.Use() });
        glLineWidth(1.f);

        // Rendering::ModelObject m = Rendering::ModelObject(_sphere,_simulation.Positions);
        Rendering::ModelObject m = Rendering::ModelObject(_sphere,_simulation.m_particlePos,_simulation.m_particleColor);
        auto const & material    = _sceneObject.Materials[0];
        m.Mesh.Draw({ material.Albedo.Use(),  material.MetaSpec.Use(), material.Height.Use(),_program.Use() },
            _sphere.Mesh.Indices.size(), 0, _simulation.m_iNumSpheres);
        
        Engine::Model obstacle_sphere = Engine::Model(Engine::Sphere(6,_simulation.obstacleRadius));
        Rendering::ModelObject obstacle = Rendering::ModelObject(obstacle_sphere, {_simulation.obstaclePos}, {glm::vec3(0.0f,0.0f,1.0f)});

        obstacle.Mesh.Draw({ material.Albedo.Use(),  material.MetaSpec.Use(), material.Height.Use(),_program.Use() },
            obstacle_sphere.Mesh.Indices.size(), 0, 1);

        glDepthFunc(GL_LEQUAL);
        glDepthFunc(GL_LESS);
        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseFluid::OnProcessInput(ImVec2 const& pos) {
        _cameraManager.ProcessInput(_sceneObject.Camera, pos);
    }

    void CaseFluid::ResetSystem(){
        _simulation.setupScene(_res);
        numofSpheres = _simulation.m_iNumSpheres;
        _r = _simulation.m_particleRadius; //cell size
    }

    void CaseFluid::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 0.2f;
        _simulation.obstaclePos += mouseDelta * movingScale;
        _simulation.obstacleVel = (mouseDelta * movingScale) / Engine::GetDeltaTime();
    }
}