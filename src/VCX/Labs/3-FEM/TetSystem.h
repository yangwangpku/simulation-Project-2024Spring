#pragma once

#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <glm/glm.hpp>
#include <iostream>
#include <utility>
#include <vector>
#include <cmath>


namespace VCX::Labs::FEM {
    struct Simulator {
        std::vector<glm::vec3> particlePos; // Particle Position
        std::vector<glm::vec3> particleVel; // Particle Velocity
        std::vector<glm::ivec4> tet;    // Tetrahedron
        std::vector<glm::vec3> particlePosRest; // Particle Position
        std::vector<glm::vec3> particleVelRest; // Particle Velocity
        std::vector<glm::vec3> particleForce; // Particle Force
        
        int wx; // number of particles in x direction
        int wy;
        int wz;
        float delta; // distance between particles
        float particle_weight;  // weight of each particle

        float poison = 0.3f;
        float young = 10000.0f;
        float density = 100.0f;
        float friction = 10.0f;

        float g = 0.1f;

        inline float trace(glm::mat3 const & m) {
            return m[0][0] + m[1][1] + m[2][2];
        }

        std::vector<glm::vec3> computeForceTet(const int tetId) {
            glm::vec3 x0 = particlePos[tet[tetId][0]];
            glm::vec3 x1 = particlePos[tet[tetId][1]];
            glm::vec3 x2 = particlePos[tet[tetId][2]];
            glm::vec3 x3 = particlePos[tet[tetId][3]];

            glm::mat3 Ds = glm::mat3(x1 - x0, x2 - x0, x3 - x0);

            glm::mat3 Ds_rest = glm::mat3(particlePosRest[tet[tetId][1]] - particlePosRest[tet[tetId][0]],
                                          particlePosRest[tet[tetId][2]] - particlePosRest[tet[tetId][0]],
                                          particlePosRest[tet[tetId][3]] - particlePosRest[tet[tetId][0]]);
            glm::mat3 F = Ds * glm::inverse(Ds_rest);
            float lambda = young * poison / ((1 + poison) * (1 - 2 * poison));
            float mu = young / (2 * (1 + poison));
            glm::mat3 G = 0.5f*(glm::transpose(F) * F - glm::mat3(1.0f));   // Green-Lagrange Strain
            glm::mat3 S = 2 * mu * G + lambda * trace(G) * glm::mat3(1.0f); // Cauchy Stress

            float V_rest = abs(glm::determinant(Ds_rest)) / 6.0f;

            glm::mat3 force = -V_rest * F * S * glm::transpose(glm::inverse(Ds_rest)) ;
            glm::vec3 f1 = force[0];
            glm::vec3 f2 = force[1];
            glm::vec3 f3 = force[2];
            glm::vec3 f0 = -f1 - f2 - f3;

            return {f0, f1, f2, f3};
        }

        void SimulateSubstep(float const dt) {
            glm::vec3 gravity { 0, -g, 0 };

            for(int i=0; i<particlePos.size(); i++)
            {
                particleForce[i] += gravity * particle_weight;
                // friction
                particleForce[i] -= friction * particleVel[i];
            }

            for(int i=0; i<tet.size(); i++)
            {
                std::vector<glm::vec3> force = computeForceTet(i);
                for(int j=0; j<4; j++)
                {
                    particleForce[tet[i][j]] += force[j];
                }
            }

            // update velocity
            for(int i=0; i<particleVel.size(); i++)
            {
                if(!is_fixed(i))
                    particleVel[i] += particleForce[i] / particle_weight * dt;
            }

            for(int i=0; i<particlePos.size(); i++)
            {
                particlePos[i] += particleVel[i] * dt;
            }

            // reset particle force
            for(int i=0; i<particleForce.size(); i++)
            {
                particleForce[i] = {0, 0, 0};
            }
        }


        void SimulateTimestep(float const dt) {
            int subSteps = 20;
            for(int i=0; i<subSteps; i++)
            {
                SimulateSubstep(dt/subSteps);
            }
        }

        inline int GetID(std::size_t const i, std::size_t const j, std::size_t const k)
        {
            return i * (wy + 1) * (wz + 1) + j * (wz + 1) + k;
        }

        inline glm::ivec3 GetCoord(int const id) {
            int x = id / ((wy + 1) * (wz + 1));
            int y = (id % ((wy + 1) * (wz + 1))) / (wz + 1);
            int z = id % (wz + 1);
            return {x, y, z};
        }

        inline bool is_fixed(const int id) {
            return id < (wy + 1) * (wz + 1);
        }

        void AddParticle(glm::vec3 const & pos) {
            particlePos.push_back(pos);
            particleVel.push_back({0, 0, 0});
        }

        void AddTet(int const a, int const b, int const c, int const d) {
            tet.push_back({a, b, c, d});
        }


        void setupSceneSimple() {
            particlePos = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
            particleVel = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
            wx = wy = wz = 0;
            particle_weight = 100;
            AddTet(0,1,2,3);
            // copy Particle Position and Velocity
            particlePosRest = particlePos;
            particleVelRest = particleVel;
        }

        void setupScene(int _wx, int _wy, int _wz, float _delta) {
            wx = _wx;
            wy = _wy;
            wz = _wz;
            delta = _delta;
            particle_weight = delta * delta * delta * density;
            for (std::size_t i = 0; i <= wx; i++) {
                for (std::size_t j = 0; j <= wy; j++) {
                    for (std::size_t k = 0; k <= wz; k++) {
                        AddParticle({ i * delta, j * delta, k * delta});
                    }
                }
            }

            for (std::size_t i = 0; i < wx; i++) {
                for (std::size_t j = 0; j < wy; j++) {
                    for (std::size_t k = 0; k < wz; k++) {
                        AddTet(GetID(i, j, k), GetID(i, j, k + 1), GetID(i, j + 1, k + 1), GetID(i + 1, j + 1, k + 1));
                        AddTet(GetID(i, j, k), GetID(i, j + 1, k), GetID(i, j + 1, k + 1), GetID(i + 1, j + 1, k + 1));
                        AddTet(GetID(i, j, k), GetID(i, j, k + 1), GetID(i + 1, j, k + 1), GetID(i + 1, j + 1, k + 1));
                        AddTet(GetID(i, j, k), GetID(i + 1, j, k), GetID(i + 1, j, k + 1), GetID(i + 1, j + 1, k + 1));
                        AddTet(GetID(i, j, k), GetID(i, j + 1, k), GetID(i + 1, j + 1, k), GetID(i + 1, j + 1, k + 1));
                        AddTet(GetID(i, j, k), GetID(i + 1, j, k), GetID(i + 1, j + 1, k), GetID(i + 1, j + 1, k + 1));
                    }
                }
            }
            // copy Particle Position and Velocity
            particlePosRest = particlePos;
            particleVelRest = particleVel;
            particleForce.resize(particlePos.size(), {0, 0, 0});
        }
    };
} // namespace VCX::Labs::Fluid