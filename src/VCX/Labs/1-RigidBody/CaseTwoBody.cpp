#include "Labs/1-RigidBody/CaseTwoBody.h"
#include "Labs/1-RigidBody/Box.h"
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
    static constexpr auto c_Cases = std::array<char const *, 3> {
        "Face-Vertex Collision",
        "Line-Line Collision",
        "Face-Face Collision"
    };

    static const std::array<std::pair<Eigen::Vector3f, Eigen::Vector3f>, 3> c_BoxDims = {
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{1, 1, 1}, Eigen::Vector3f{1, 1, 1}},
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{0.5, 0.5, 1.5}, Eigen::Vector3f{0.5, 1.5, 0.5}},
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{1, 1, 1}, Eigen::Vector3f{1, 1, 1}}
    };

    static const std::array<std::pair<Eigen::Vector3f, Eigen::Vector3f>, 3> c_BoxPositions = {
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{0, 0, 0}, Eigen::Vector3f{3, 0, 0}},
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{0, 0, 0}, Eigen::Vector3f{3, -0.5, 0}},
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{0, 0, 0}, Eigen::Vector3f{3, 0, 0}}
    };

    static const std::array<std::pair<Eigen::Quaternionf, Eigen::Quaternionf>, 3> c_BoxOrientations = {
        std::pair<Eigen::Quaternionf, Eigen::Quaternionf>{Eigen::Quaternionf{1, 0, 0, 0}, Eigen::Quaternionf{0.3, 0.9, 0.3, 0.1}},
        std::pair<Eigen::Quaternionf, Eigen::Quaternionf>{Eigen::Quaternionf{1, 0, 0, 0}, Eigen::Quaternionf{0.3, 0.3, 0.9, 0.1}},
        std::pair<Eigen::Quaternionf, Eigen::Quaternionf>{Eigen::Quaternionf{1, 0, 0, 0}, Eigen::Quaternionf{1, 0, 0, 0}}
    };

    CaseTwoBody::CaseTwoBody():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _boxNum(2),
        _box{Box({1,1,1},{0,0,0}), Box({1,1,1},{3.,0,0},{0.3,0.9,0.3,0.1})} {
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

        _box[0].velocity = Eigen::Vector3f(0.5, 0, 0);
        _box[1].velocity = Eigen::Vector3f(-0.5, 0, 0);

        _initialBox[0] = _box[0];
        _initialBox[1] = _box[1];
    }

    void CaseTwoBody::Reset() {
        _box[0] = _initialBox[0];
        _box[1] = _initialBox[1];
    }

    void CaseTwoBody::OnProcessMouseControl() {
        if (ImGui::IsKeyPressed(ImGuiKey_F)) {
            Reset();
            return;
        }

        if(_recompute) {
            _recompute = false;
            _initialBox[0].center = c_BoxPositions[_caseId].first;
            _initialBox[1].center = c_BoxPositions[_caseId].second;

            _initialBox[0].orientation = c_BoxOrientations[_caseId].first;
            _initialBox[1].orientation = c_BoxOrientations[_caseId].second;

            _initialBox[0].dim = c_BoxDims[_caseId].first;
            _initialBox[1].dim = c_BoxDims[_caseId].second;
            _box[0] = _initialBox[0];
            _box[1] = _initialBox[1];
        }
    }


    void CaseTwoBody::OnSetupPropsUI() {
        _recompute |= ImGui::Combo("Case", &_caseId, c_Cases.data(), c_Cases.size());
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_box[0].boxColor));
            ImGui::SliderFloat("Box 0 dim_x", &_initialBox[0].dim[0], 0.5, 4);
            ImGui::SliderFloat("Box 0 dim_y", &_initialBox[0].dim[1], 0.5, 4);
            ImGui::SliderFloat("Box 0 dim_z", &_initialBox[0].dim[2], 0.5, 4);

            ImGui::SliderFloat("Box 1 dim_x", &_initialBox[1].dim[0], 0.5, 4);
            ImGui::SliderFloat("Box 1 dim_y", &_initialBox[1].dim[1], 0.5, 4);
            ImGui::SliderFloat("Box 1 dim_z", &_initialBox[1].dim[2], 0.5, 4);

            ImGui::InputFloat("Box 0 pos_x", &_initialBox[0].center[0]);
            ImGui::InputFloat("Box 0 pos_y", &_initialBox[0].center[1]);
            ImGui::InputFloat("Box 0 pos_z", &_initialBox[0].center[2]);

            ImGui::InputFloat("Box 1 pos_x", &_initialBox[1].center[0]);
            ImGui::InputFloat("Box 1 pos_y", &_initialBox[1].center[1]);
            ImGui::InputFloat("Box 1 pos_z", &_initialBox[1].center[2]);

        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseTwoBody::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        OnProcessMouseControl();
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

    void CaseTwoBody::Advance(float timeDelta) {
        for(int i=0; i<_boxNum; i++){
            _box[i].center += timeDelta * _box[i].velocity;   // update position

            Eigen::Quaternionf _angularVelocityQuaternion(0, _box[i].angularVelocity.x() * timeDelta * 0.5f, _box[i].angularVelocity.y() * timeDelta * 0.5f, _box[i].angularVelocity.z() * timeDelta * 0.5f);

            _box[i].orientation.coeffs() += (_angularVelocityQuaternion*_box[i].orientation).coeffs();
            _box[i].orientation.normalize();
        }
        DetectCollision();
        SolveCollision();
    }

    void CaseTwoBody::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }


    // void CaseTwoBody::DetectCollision() {
    //     Box & b0 = _box[0];
    //     Box & b1 = _box[1];
    //     // Eigen::Vector3f RigidBody::dim - size of a box
    //     using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
    //     CollisionGeometryPtr_t box_geometry_A(new fcl::Box<float>(b0.dim[0], b0.dim[1],
    //     b0.dim[2]));
    //     CollisionGeometryPtr_t box_geometry_B(new fcl::Box<float>(b1.dim[0], b1.dim[1],
    //     b1.dim[2]));
    //     // Eigen::Vector3f RigidBody::x - position of a box, Eigen::Quaternionf
    //     fcl::CollisionObject<float> box_A(box_geometry_A,
    //     fcl::Transform3f(Eigen::Translation3f(b0.center)*b0.orientation));

    //     fcl::CollisionObject<float> box_B(box_geometry_B,
    //     fcl::Transform3f(Eigen::Translation3f(b1.center)*b1.orientation));
    //     // Compute collision - at most 8 contacts and return contact information.
    //     fcl::CollisionRequest<float> collisionRequest(1, true);
    //     fcl::CollisionResult<float> collisionResult;
    //     fcl::collide(&box_A, &box_B, collisionRequest, collisionResult);
    //     _collisionResult = collisionResult;
    // }

    // void CaseTwoBody::SolveCollision() {
    //     if(!_collisionResult.isCollision()) {
    //         return;
    //     }
        
    //     Box & b0 = _box[0];
    //     Box & b1 = _box[1];

    //     std::vector<fcl::Contact<float>> contacts;
    //     _collisionResult.getContacts(contacts);
    //     Eigen::Vector3f contact_pos = contacts[0].pos;
    //     Eigen::Vector3f contact_normal = -contacts[0].normal;

    //     // Compute relative velocity
    //     Eigen::Vector3f v0 = b0.velocity + b0.angularVelocity.cross(contact_pos - b0.center);
    //     Eigen::Vector3f v1 = b1.velocity + b1.angularVelocity.cross(contact_pos - b1.center);
    //     float relative_velocity = (v0 - v1).dot(contact_normal);

    //     // return if objects are separating
    //     if(relative_velocity > 0) {
    //         return;
    //     }

    //     // Compute impulse
    //     float e = 0.9f; // coefficient of restitution
    //     float numerator = -(1 + e) * relative_velocity;
    //     float denominator = 1 / b0.mass + 1 / b1.mass + contact_normal.dot((b0.GetInertiaMatrix().inverse() * (contact_pos - b0.center).cross(contact_normal)).cross(contact_pos - b0.center)) + contact_normal.dot((b1.GetInertiaMatrix().inverse() * (contact_pos - b1.center).cross(contact_normal)).cross(contact_pos - b1.center));
    //     float impulse = numerator / denominator;

    //     // Apply impulse
    //     b0.velocity += impulse / b0.mass * contact_normal;
    //     b1.velocity -= impulse / b1.mass * contact_normal;
    //     b0.angularVelocity += impulse * b0.GetInertiaMatrix().inverse() * (contact_pos - b0.center).cross(contact_normal);
    //     b1.angularVelocity -= impulse * b1.GetInertiaMatrix().inverse() * (contact_pos - b1.center).cross(contact_normal);
    // }

} // namespace VCX::Labs::GettingStarted
