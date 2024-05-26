#pragma once

#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <glm/glm.hpp>
#include <iostream>

namespace VCX::Labs::PD {
    struct MassSpringSystem {
        struct Spring {
            std::pair<std::size_t, std::size_t> AdjIdx;
            float                               RestLength;
        };

        std::vector<glm::vec3> Positions;
        std::vector<glm::vec3> Velocities;
        std::vector<int>       Fixed;
        float                  Mass { 1 };

        std::vector<Spring> Springs;
        float               Stiffness { 100 };
        float               Damping { .2f };
        float               Gravity { .3f };

        void prefactorize_lhs(float const dt, Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>& solver) {
            const int                          nDoFs = int(Positions.size()) * 3;
            Eigen::SparseMatrix<float>         matLinearized(nDoFs, nDoFs);
            std::vector<Eigen::Triplet<float>> coefficients;
            
            const auto AddBlock = [&](int const p0, int const p1, glm::mat3 const & block) {
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        coefficients.emplace_back(p0 * 3 + i, p1 * 3 + j, block[j][i]);
            };

            for (int i = 0; i < nDoFs; i++) coefficients.emplace_back(i, i, Mass / dt/ dt); // Mass term

            // Hessians
            for (auto const & spring : Springs) {
                auto const p0 = spring.AdjIdx.first;
                auto const p1 = spring.AdjIdx.second;
                AddBlock(p0, p0, glm::mat3(1.0));
                AddBlock(p0, p1, -glm::mat3(1.0));
                AddBlock(p1, p0, -glm::mat3(1.0));
                AddBlock(p1, p1, glm::mat3(1.0));
            }

            matLinearized.setFromTriplets(coefficients.begin(), coefficients.end());
            solver.compute(matLinearized);

        }

        void AddParticle(glm::vec3 const & position, glm::vec3 const & velocity = glm::vec3(0)) {
            Positions.push_back(position);
            Velocities.push_back(velocity);
            Fixed.push_back(false);
        }

        void AddSpring(std::size_t const adjIdx0, std::size_t const adjIdx1, float const restLength = -1) {
            Springs.push_back({
                .AdjIdx { adjIdx0, adjIdx1 },
                .RestLength { restLength < 0 ? glm::length(Positions[adjIdx0] - Positions[adjIdx1]) : restLength }
            });
        }

        void AdvanceMassSpringSystem(float const dt) {
            int total_steps = 3;

            // save original positions
            std::vector<glm::vec3> original_positions(Positions);

            std::vector<glm::vec3> f_ext(Positions.size(), glm::vec3(0, -Gravity, 0) * Mass);

            // add damping force
            for (std::size_t i = 0; i < Positions.size(); i++) {
                f_ext[i] += -Damping * Velocities[i];
            }

            for (std::size_t i = 0; i < Positions.size(); i++) {
                if (Fixed[i]) f_ext[i] = glm::vec3(0);
            }

            std::vector<glm::vec3> target_positions(Positions.size(),glm::vec3(0, 0, 0)); // y-x
            
            for (std::size_t i = 0; i < Positions.size(); i++) {
                target_positions[i] = original_positions[i] + (Velocities[i] + dt * f_ext[i] / Mass) * dt;
            }

            while(total_steps--) {

                std::vector<glm::vec3> target_diffs(Positions.size(),glm::vec3(0, 0, 0)); // y-x
                
                for (std::size_t i = 0; i < Positions.size(); i++) {
                    target_diffs[i] = target_positions[i] - Positions[i];
                }
            
                const int                          nDoFs = int(Positions.size()) * 3;

                std::vector<glm::vec3> f_ints(Positions.size(), glm::vec3(0, 0, 0));
                for (auto const & spring : Springs) {
                    auto const p0 = spring.AdjIdx.first;
                    auto const p1 = spring.AdjIdx.second;
                    // if (Fixed[p0] && Fixed[p1]) continue;

                    glm::vec3 const x0   = Positions[p0];
                    glm::vec3 const x1   = Positions[p1];
                    glm::vec3 const x01   = x1 - x0;
                    glm::vec3 const e01   = glm::normalize(x01);
                    float const    l      = glm::length(x01);

                    glm::vec3 xe0 = x0 + ((l - spring.RestLength) / 2.0f) * e01;
                    glm::vec3 xe1 = x1 - ((l - spring.RestLength) / 2.0f) * e01;

                    if (Fixed[p0] && !Fixed[p1]) {
                        xe0 = x0;
                        xe1 = x1 - e01 * (l - spring.RestLength);
                    }
                    else if(!Fixed[p0] && Fixed[p1]) {
                        xe0 = x0 + e01 * (l - spring.RestLength);
                        xe1 = x1;
                    }

                    if (!Fixed[p0])
                        f_ints[p0] -= Stiffness * (x0 - x1 - (xe0 - xe1));
                    if (!Fixed[p1])
                        f_ints[p1] -= Stiffness * (x1 - x0 - (xe1 - xe0));

                }

                auto vec_target_diffs = Eigen::Map<Eigen::VectorXf, Eigen::Aligned>(reinterpret_cast<float *>(target_diffs.data()), nDoFs);
                auto vec_f_ints     = Eigen::Map<Eigen::VectorXf, Eigen::Aligned>(reinterpret_cast<float *>(f_ints.data()), nDoFs);

                Eigen::VectorXf rhsLinearized = Mass*vec_target_diffs/dt/dt + vec_f_ints;

                Eigen::SimplicialLLT<Eigen::SparseMatrix<float>> solver;
                prefactorize_lhs(dt,solver);

                // prefactorize_lhs(dt);
                vec_target_diffs = solver.solve(rhsLinearized);

                // print matLinearized and rhsLinearized for debugging
                // std::cout << "matLinearized:" << matLinearized << std::endl;
                // std::cout << "rhsLinearized:" << rhsLinearized << std::endl;


                for (std::size_t i = 0; i < Positions.size(); i++) {
                    for (int j = 0; j < 3; j++)
                        Positions[i][j] += vec_target_diffs[i*3+j];
                }
            }

            for (std::size_t i = 0; i < Positions.size(); i++) {
                Velocities[i] = (Positions[i]-original_positions[i]) / dt;
            }

        }
        
    };
} // namespace VCX::Labs::GettingStarted
