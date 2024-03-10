#include "Labs/1-RigidBody/CaseRigidBody.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"
#include <iostream>

static glm::vec3 eigen2glm(const Eigen::Vector3f& eigenVec) {
    return glm::vec3(eigenVec.x(), eigenVec.y(), eigenVec.z());
}

static Eigen::Vector3f glm2eigen(const glm::vec3& glmVec) {
    return Eigen::Vector3f(glmVec.x, glmVec.y, glmVec.z);
}


namespace VCX::Labs::RigidBody {

    CaseRigidBody::CaseRigidBody():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
        _lineItem.UpdateElementBuffer(line_index);

        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7 };
        _boxItem.UpdateElementBuffer(tri_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseRigidBody::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
            ImGui::SliderFloat("x", &_dim[0], 0.5, 4);
            ImGui::SliderFloat("y", &_dim[1], 0.5, 4);
            ImGui::SliderFloat("z", &_dim[2], 0.5, 4);

            ImGui::InputFloat("pos_x", &_center[0]);
            ImGui::InputFloat("pos_y", &_center[1]);
            ImGui::InputFloat("pos_z", &_center[2]);

            ImGui::InputFloat("velocity_x", &_velocity[0]);
            ImGui::InputFloat("velocity_y", &_velocity[1]);
            ImGui::InputFloat("velocity_z", &_velocity[2]);
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseRigidBody::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());

        Advance(Engine::GetDeltaTime());

        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        std::vector<glm::vec3> VertsPosition;

        Eigen::Matrix3f orientationMatrix = _orientation.toRotationMatrix();
        Eigen::Vector3f new_x = (_dim[0] / 2) * orientationMatrix * Eigen::Vector3f(1.f, 0.f, 0.f);
        Eigen::Vector3f new_y = (_dim[1] / 2) * orientationMatrix * Eigen::Vector3f(0.f, 1.f, 0.f);
        Eigen::Vector3f new_z = (_dim[2] / 2) * orientationMatrix * Eigen::Vector3f(0.f, 0.f, 1.f);

        VertsPosition.resize(8);
        VertsPosition[0] = eigen2glm(_center - new_x + new_y + new_z);
        VertsPosition[1] = eigen2glm(_center + new_x + new_y + new_z);
        VertsPosition[2] = eigen2glm(_center + new_x + new_y - new_z);
        VertsPosition[3] = eigen2glm(_center - new_x + new_y - new_z);
        VertsPosition[4] = eigen2glm(_center - new_x - new_y + new_z);
        VertsPosition[5] = eigen2glm(_center + new_x - new_y + new_z);
        VertsPosition[6] = eigen2glm(_center + new_x - new_y - new_z);
        VertsPosition[7] = eigen2glm(_center - new_x - new_y - new_z);

        auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);

        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes);
        _boxItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
        _lineItem.Draw({ _program.Use() });

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

    void CaseRigidBody::Advance(float timeDelta) {
        _center += timeDelta * _velocity;   // update position

        Eigen::Quaternionf _angularVelocityQuaternion(0, _angularVelocity.x() * timeDelta * 0.5f, _angularVelocity.y() * timeDelta * 0.5f, _angularVelocity.z() * timeDelta * 0.5f);

        _orientation.coeffs() += (_angularVelocityQuaternion*_orientation).coeffs();
        _orientation.normalize();
    }

    void CaseRigidBody::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseRigidBody::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 1.0f;
        _velocity = glm2eigen(mouseDelta) * movingScale;
    }

} // namespace VCX::Labs::GettingStarted
