#include "Labs/1-RigidBody/CaseComplex.h"
#include "Labs/1-RigidBody/Box.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"
#include "Collision.h"
#include <iostream>
#include <random>

static glm::vec3 eigen2glm(const Eigen::Vector3f& eigenVec) {
    return glm::vec3(eigenVec.x(), eigenVec.y(), eigenVec.z());
}

static Eigen::Vector3f glm2eigen(const glm::vec3& glmVec) {
    return Eigen::Vector3f(glmVec.x, glmVec.y, glmVec.z);
}

static Eigen::Quaternionf randomQuaternion() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-1.0, 1.0);

    // Generate four random numbers
    float w = dist(gen);
    float x = dist(gen);
    float y = dist(gen);
    float z = dist(gen);

    // Create the quaternion
    Eigen::Quaternionf q(w, x, y, z);

    // Normalize to ensure it's a valid rotation quaternion
    q.normalize();

    return q;
}

static Eigen::Vector3f randomVector(Eigen::Vector3f lowerBound, Eigen::Vector3f upperBound) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distx(lowerBound.x(), upperBound.x());
    std::uniform_real_distribution<float> disty(lowerBound.y(), upperBound.y());
    std::uniform_real_distribution<float> distz(lowerBound.z(), upperBound.z());

    // Generate three random numbers
    float x = distx(gen);
    float y = disty(gen);
    float z = distz(gen);

    // Create the vector
    Eigen::Vector3f v(x, y, z);

    return v;
}

namespace VCX::Labs::RigidBody {

    CaseComplex::CaseComplex():
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

        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7 };
        
        std::vector<std::uint32_t> line_indices = {};
        for(int i=0; i<_boxNum; i++){
            for(int j=0; j<line_index.size(); j++){
                line_indices.push_back(line_index[j] + 8*i);
            }
        }
        _lineItem.UpdateElementBuffer(line_indices);

        std::vector<std::uint32_t> tri_indices = {};
        for(int i=0; i<_boxNum; i++){
            for(int j=0; j<tri_index.size(); j++){
                tri_indices.push_back(tri_index[j] + 8*i);
            }
        }
        
        _boxItem.UpdateElementBuffer(tri_indices);
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        GenerateBox();

    }

    void CaseComplex::GenerateBox() {
        float groundSize = 8;
        float groundHeight = 0.5;
    
        // fixed ground
        _box[0] = Box({groundSize,groundHeight,groundSize},{0,-5,0},{1,0,0,0},10000);  

        // fixed wall
        _box[1] = Box({groundHeight,groundSize,groundSize},{-5,0,0},{0,1,0,0},10000);
        _box[2] = Box({groundHeight,groundSize,groundSize},{5,0,0},{0,1,0,0},10000);
        _box[3] = Box({groundSize,groundSize,groundHeight},{0,0,5},{0,0,0,1},10000);


        for(int i=_fixedboxNum; i < _boxNum; i++)
        {
            float randomness = 0.2;
            // add some randomness to the box to make the demo more interesting
            _box[i] = Box({1,1,1},randomVector({-randomness,-5 + 2*i - randomness,-randomness},{randomness,-5 + 2*i + randomness,randomness}) ,randomQuaternion(),1);
            _box[i].velocity = randomVector({-randomness,-1-randomness,-randomness},{randomness,-1+randomness,randomness});
        }
    }

    void CaseComplex::OnProcessMouseControl(glm::vec3 mouseDelta) {
        if (ImGui::IsKeyPressed(ImGuiKey_F)) {
            GenerateBox();
            return;
        }

        float movingScale = 0.1f;
        _box[0].center += glm2eigen(mouseDelta) * movingScale;
    }


    void CaseComplex::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_box[0].boxColor));
        }
        if (ImGui::CollapsingHeader("Physics Parameter", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("Restitution", &_restitution, 0, 1);
            ImGui::SliderFloat("Gravity", &_gravity, 0, 10);
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseComplex::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
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
        VertsPosition.resize(8 * _boxNum);

        for(int i=0; i<_boxNum; i++) {
            Eigen::Matrix3f orientationMatrix = _box[i].orientation.toRotationMatrix();
            Eigen::Vector3f new_x = (_box[i].dim[0] / 2) * orientationMatrix * Eigen::Vector3f(1.f, 0.f, 0.f);
            Eigen::Vector3f new_y = (_box[i].dim[1] / 2) * orientationMatrix * Eigen::Vector3f(0.f, 1.f, 0.f);
            Eigen::Vector3f new_z = (_box[i].dim[2] / 2) * orientationMatrix * Eigen::Vector3f(0.f, 0.f, 1.f);

            VertsPosition[8*i+0] = eigen2glm(_box[i].center - new_x + new_y + new_z);
            VertsPosition[8*i+1] = eigen2glm(_box[i].center + new_x + new_y + new_z);
            VertsPosition[8*i+2] = eigen2glm(_box[i].center + new_x + new_y - new_z);
            VertsPosition[8*i+3] = eigen2glm(_box[i].center - new_x + new_y - new_z);
            VertsPosition[8*i+4] = eigen2glm(_box[i].center - new_x - new_y + new_z);
            VertsPosition[8*i+5] = eigen2glm(_box[i].center + new_x - new_y + new_z);
            VertsPosition[8*i+6] = eigen2glm(_box[i].center + new_x - new_y - new_z);
            VertsPosition[8*i+7] = eigen2glm(_box[i].center - new_x - new_y - new_z);
        }

        auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);

        _program.GetUniforms().SetByName("u_Color", _box[0].boxColor);
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

    void CaseComplex::Advance(float timeDelta) {
        for(int i=0; i<_boxNum; i++){
            if(i>=_fixedboxNum)
                _box[i].velocity.y() -= _gravity * timeDelta; // gravity
            _box[i].center += timeDelta * _box[i].velocity;   // update position

            Eigen::Quaternionf _angularVelocityQuaternion(0, _box[i].angularVelocity.x() * timeDelta * 0.5f, _box[i].angularVelocity.y() * timeDelta * 0.5f, _box[i].angularVelocity.z() * timeDelta * 0.5f);

            _box[i].orientation.coeffs() += (_angularVelocityQuaternion*_box[i].orientation).coeffs();
            _box[i].orientation.normalize();
        }
        DetectCollision();
        SolveCollision();
    }

    void CaseComplex::DetectCollision() {
        for(int i=0;i<_boxNum;i++)
        for(int j=i+1;j<_boxNum;j++) {
            VCX::Labs::RigidBody::DetectCollision(_box[i], _box[j], _collisionResult[i][j]);   
        }
    }

    void CaseComplex::SolveCollision() {
        for(int i=0;i<_boxNum;i++)
        for(int j=i+1;j<_boxNum;j++) {
            VCX::Labs::RigidBody::SolveCollision(_box[i], _box[j], _collisionResult[i][j],_restitution);   
        }
    }

    void CaseComplex::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

} // namespace VCX::Labs::GettingStarted
