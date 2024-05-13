#include "Labs/3-FEM/CaseDeform.h"
#include "Labs/3-FEM/TetSystem.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"
#include <iostream>



namespace VCX::Labs::FEM {

    CaseDeform::CaseDeform():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _tetItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles)
         {

        _tetSystem.setupScene(8*2, 2*2, 2*2, 0.5f);
        // _tetSystem.setupSceneSimple();

        const std::vector<std::uint32_t> tri_index_tet = { 0, 1, 2, 0, 1, 3, 0, 2, 3, 1, 2, 3};
        std::vector<std::uint32_t> tri_index;

        for(int i=0; i < _tetSystem.tet.size(); i++)
        {
            glm::ivec4 curtet = _tetSystem.tet[i];
            for(int j=0; j<tri_index_tet.size(); j++)
            {
                tri_index.push_back(curtet[tri_index_tet[j]]);
            }
        }
        _tetItem.UpdateElementBuffer(tri_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

    }

    void CaseDeform::ResetSystem() {
        _tetSystem.particlePos = _tetSystem.particlePosRest;
        _tetSystem.particleVel = _tetSystem.particleVelRest;
    }

    void CaseDeform::OnSetupPropsUI() {
        if(ImGui::Button("Reset System")) 
            ResetSystem();

        ImGui::SliderFloat("Gravity", &_tetSystem.g, 0.0f, 1.0f);
        ImGui::SliderFloat("Young", &_tetSystem.young, 1000.0f, 100000.0f);
        ImGui::SliderFloat("Poison", &_tetSystem.poison, -1.0f, 0.5f);
        ImGui::SliderFloat("Friction", &_tetSystem.friction, 0.0f, 100.0f);
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseDeform::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        std::pair<glm::vec3,int> force =  _forceManager.getForce(_tetSystem.particlePos);

        OnProcessMouseControl(force);

        // Advance(Engine::GetDeltaTime());
        _tetSystem.SimulateTimestep(Engine::GetDeltaTime());

        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        auto span_bytes = Engine::make_span_bytes<glm::vec3>(_tetSystem.particlePos);

        _program.GetUniforms().SetByName("u_Color", glm::vec3{ 121.0f / 255, 207.0f / 255, 171.0f / 255 });
        _tetItem.UpdateVertexBuffer("position", span_bytes);
        _tetItem.Draw({ _program.Use() });

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseDeform::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        _forceManager.ProcessInput(_camera, pos);
    }

    void CaseDeform::OnProcessMouseControl(std::pair<glm::vec3,int> force) {
        glm::vec3 forceVec = force.first;
        int pointId = force.second;

        glm::vec3 pointPos = _tetSystem.particlePos[pointId];
        
        float forceScale = 10000.0f;
        float forceRange = 2;

        // apply force to the point around the mouse
        for(int i=0; i<_tetSystem.particlePos.size(); i++)
        {
            glm::vec3 pos = _tetSystem.particlePos[i];
            glm::vec3 diff = pos - pointPos;
            float dist = glm::length(diff);
            if(dist < forceRange)
            {
                _tetSystem.particleForce[i] += forceScale * forceVec * (1 - dist/forceRange);
            }
        }


    }

} // namespace VCX::Labs::GettingStarted
