#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <glm/glm.hpp>
#include <iostream>
#include <utility>
#include <vector>
#include "Labs/2-FluidSimulation/FluidSimulator.h"
#include "spdlog/spdlog.h"

namespace VCX::Labs::Fluid {
    void Simulator::integrateParticles(float timeStep) {
        // Integrate particle positions
        for (int i = 0; i < m_particlePos.size(); i++) {
            m_particleVel[i] += gravity * timeStep;
            m_particlePos[i] += m_particleVel[i] * timeStep;
        }
    }

    void Simulator::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRadius, glm::vec3 obstacleVel) {
        for (int i = 0; i < m_particlePos.size(); i++) {
            if(m_particlePos[i].x < m_h + m_particleRadius - 0.5f) {
                m_particlePos[i].x = m_h + m_particleRadius - 0.5f;
                m_particleVel[i].x = 0;
            }

            if(m_particlePos[i].x > (m_fInvSpacing - 1)*m_h - m_particleRadius - 0.5f) {
                m_particlePos[i].x = (m_fInvSpacing - 1)*m_h - m_particleRadius - 0.5f;
                m_particleVel[i].x = 0;
            }

            if(m_particlePos[i].y < m_h + m_particleRadius - 0.5f) {
                m_particlePos[i].y = m_h + m_particleRadius - 0.5f;
                m_particleVel[i].y = 0;
            }

            if(m_particlePos[i].y > (m_fInvSpacing - 1)*m_h - m_particleRadius - 0.5f) {
                m_particlePos[i].y = (m_fInvSpacing - 1)*m_h - m_particleRadius - 0.5f;
                m_particleVel[i].y = 0;
            }

            if(m_particlePos[i].z < m_h + m_particleRadius - 0.5f) {
                m_particlePos[i].z = m_h + m_particleRadius - 0.5f;
                m_particleVel[i].z = 0;
            }

            if(m_particlePos[i].z > (m_fInvSpacing - 1)*m_h - m_particleRadius - 0.5f) {
                m_particlePos[i].z = (m_fInvSpacing - 1)*m_h - m_particleRadius - 0.5f;
                m_particleVel[i].z = 0;
            }
        }
    }

    inline int Simulator::index2GridOffset(glm::ivec3 index) {
        return index.x + index.y * m_iCellX + index.z * m_iCellX * m_iCellY;
    }

    void Simulator::transferVelocities(bool toGrid, float flipRatio) {
        if(toGrid) {
            // Clear grid velocities
            for (int i = 0; i < m_iNumCells; i++) {
                m_vel[i] = glm::vec3(0.0f);
                m_near_num[0][i] = 0.0f;
                m_near_num[1][i] = 0.0f;
                m_near_num[2][i] = 0.0f;
            }

            // Init m_type
            for (int i = 0; i < m_iNumCells; i++) {
                if(m_s[i] > 0.0f) {
                    m_type[i] = EMPTY_CELL;
                }
                else {
                    m_type[i] = SOLID_CELL;
                }
            }

            // copy m_vel to m_pre_vel
            for (int i = 0; i < m_iNumCells; i++) {
                m_pre_vel[i] = m_vel[i];
            }
        }


        for (int i = 0; i < m_particlePos.size(); i++) {
            glm::vec3 pos = m_particlePos[i];

            // update m_type
            glm::vec3 gridOffset = glm::vec3(-0.5f);
            
            glm::vec3 posRelGrid = pos - gridOffset;
            glm::ivec3 cellIndex = glm::ivec3(posRelGrid / m_h);

            m_type[index2GridOffset(cellIndex)] = FLUID_CELL;

            for(int j = 0; j < 3; j++) {
                glm::vec3 gridOffset = glm::vec3(-0.5f) + m_h * glm::vec3(0.5f);
                gridOffset[j] -= m_h * 0.5f;
                
                glm::vec3 posRelGrid = pos - gridOffset;
                glm::ivec3 cellIndex = glm::ivec3(posRelGrid / m_h);

                glm::vec3 delta = posRelGrid - glm::vec3(cellIndex) * m_h;
                glm::vec3 deltaComplement = glm::vec3(1.0f) - delta;

                // print delta for debugging
                // spdlog::info("delta: ({}, {}, {})", delta.x, delta.y, delta.z);

                if(toGrid) {
                    // Transfer particle velocities to grid
                    glm::vec3 vel = m_particleVel[i];
                    m_near_num[j][index2GridOffset(cellIndex + glm::ivec3(0, 0, 0))] += deltaComplement.x * deltaComplement.y * deltaComplement.z;
                    m_near_num[j][index2GridOffset(cellIndex + glm::ivec3(1, 0, 0))] += delta.x * deltaComplement.y * deltaComplement.z;
                    m_near_num[j][index2GridOffset(cellIndex + glm::ivec3(0, 1, 0))] += deltaComplement.x * delta.y * deltaComplement.z;
                    m_near_num[j][index2GridOffset(cellIndex + glm::ivec3(1, 1, 0))] += delta.x * delta.y * deltaComplement.z;
                    m_near_num[j][index2GridOffset(cellIndex + glm::ivec3(0, 0, 1))] += deltaComplement.x * deltaComplement.y * delta.z;
                    m_near_num[j][index2GridOffset(cellIndex + glm::ivec3(1, 0, 1))] += delta.x * deltaComplement.y * delta.z;
                    m_near_num[j][index2GridOffset(cellIndex + glm::ivec3(0, 1, 1))] += deltaComplement.x * delta.y * delta.z;
                    m_near_num[j][index2GridOffset(cellIndex + glm::ivec3(1, 1, 1))] += delta.x * delta.y * delta.z;
                
                    m_vel[index2GridOffset(cellIndex + glm::ivec3(0, 0, 0))][j] += vel[j] * deltaComplement.x * deltaComplement.y * deltaComplement.z;
                    m_vel[index2GridOffset(cellIndex + glm::ivec3(1, 0, 0))][j] += vel[j] * delta.x * deltaComplement.y * deltaComplement.z;
                    m_vel[index2GridOffset(cellIndex + glm::ivec3(0, 1, 0))][j] += vel[j] * deltaComplement.x * delta.y * deltaComplement.z;
                    m_vel[index2GridOffset(cellIndex + glm::ivec3(1, 1, 0))][j] += vel[j] * delta.x * delta.y * deltaComplement.z;
                    m_vel[index2GridOffset(cellIndex + glm::ivec3(0, 0, 1))][j] += vel[j] * deltaComplement.x * deltaComplement.y * delta.z;
                    m_vel[index2GridOffset(cellIndex + glm::ivec3(1, 0, 1))][j] += vel[j] * delta.x * deltaComplement.y * delta.z;
                    m_vel[index2GridOffset(cellIndex + glm::ivec3(0, 1, 1))][j] += vel[j] * deltaComplement.x * delta.y * delta.z;
                    m_vel[index2GridOffset(cellIndex + glm::ivec3(1, 1, 1))][j] += vel[j] * delta.x * delta.y * delta.z;
                }
                else {
                    // Transfer grid velocities to particles
                    float vel = 0;
                    vel += m_vel[index2GridOffset(cellIndex + glm::ivec3(0, 0, 0))][j] * deltaComplement.x * deltaComplement.y * deltaComplement.z;
                    vel += m_vel[index2GridOffset(cellIndex + glm::ivec3(1, 0, 0))][j] * delta.x * deltaComplement.y * deltaComplement.z;
                    vel += m_vel[index2GridOffset(cellIndex + glm::ivec3(0, 1, 0))][j] * deltaComplement.x * delta.y * deltaComplement.z;
                    vel += m_vel[index2GridOffset(cellIndex + glm::ivec3(1, 1, 0))][j] * delta.x * delta.y * deltaComplement.z;
                    vel += m_vel[index2GridOffset(cellIndex + glm::ivec3(0, 0, 1))][j] * deltaComplement.x * deltaComplement.y * delta.z;
                    vel += m_vel[index2GridOffset(cellIndex + glm::ivec3(1, 0, 1))][j] * delta.x * deltaComplement.y * delta.z;
                    vel += m_vel[index2GridOffset(cellIndex + glm::ivec3(0, 1, 1))][j] * deltaComplement.x * delta.y * delta.z;
                    vel += m_vel[index2GridOffset(cellIndex + glm::ivec3(1, 1, 1))][j] * delta.x * delta.y * delta.z;
                    // m_particleVel[i][j] = vel;
                    // print flipRatio for debugging
                    
                    m_particleVel[i][j] = (vel - m_pre_vel[i][j] + m_particleVel[i][j]) * flipRatio + (1-flipRatio) * vel;
                }

            }
        }

        if (toGrid) {
            // Normalize grid velocities
            for (int i = 0; i < m_iNumCells; i++) {
                if (m_s[i] > 0) {
                    for (int j = 0; j < 3; j++) {
                        if (m_near_num[j][i] > 0.0f) {
                            m_vel[i][j] /= m_near_num[j][i];
                        }
                        else {
                            m_vel[i][j] = 0.0f;
                        }
                    }
                }
            }
        }
    }

    void Simulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
        while(numIters--) {
            for(int i = 0; i < m_iCellX; i++) {
                for(int j = 0; j < m_iCellY; j++) {
                    for(int k = 0; k < m_iCellZ; k++) {
                        if (m_type[index2GridOffset(glm::ivec3(i, j, k))] == FLUID_CELL) {
                            float d = overRelaxation * (-m_vel[index2GridOffset(glm::ivec3(i, j, k))].x
                                -m_vel[index2GridOffset(glm::ivec3(i, j, k))].y
                                -m_vel[index2GridOffset(glm::ivec3(i, j, k))].z
                                +m_vel[index2GridOffset(glm::ivec3(i + 1, j, k))].x
                                +m_vel[index2GridOffset(glm::ivec3(i, j + 1, k))].y
                                +m_vel[index2GridOffset(glm::ivec3(i, j, k + 1))].z);

                            if (compensateDrift)    
                                d -= m_particleDensity[index2GridOffset(glm::ivec3(i, j, k))] - m_particleRestDensity;
                            float s = m_s[index2GridOffset(glm::ivec3(i + 1, j, k))]
                                +m_s[index2GridOffset(glm::ivec3(i, j + 1, k))]
                                +m_s[index2GridOffset(glm::ivec3(i, j, k + 1))]
                                +m_s[index2GridOffset(glm::ivec3(i - 1, j, k))]
                                +m_s[index2GridOffset(glm::ivec3(i, j - 1, k))]
                                +m_s[index2GridOffset(glm::ivec3(i, j, k - 1))];

                            m_vel[index2GridOffset(glm::ivec3(i, j, k))].x += (d * m_s[index2GridOffset(glm::ivec3(i - 1, j, k))]) / s;
                            m_vel[index2GridOffset(glm::ivec3(i, j, k))].y += (d * m_s[index2GridOffset(glm::ivec3(i, j - 1, k))]) / s;
                            m_vel[index2GridOffset(glm::ivec3(i, j, k))].z += (d * m_s[index2GridOffset(glm::ivec3(i, j, k - 1))]) / s;
                            m_vel[index2GridOffset(glm::ivec3(i+1, j, k))].x -= (d * m_s[index2GridOffset(glm::ivec3(i + 1, j, k))]) / s;
                            m_vel[index2GridOffset(glm::ivec3(i, j+1, k))].y -= (d * m_s[index2GridOffset(glm::ivec3(i, j + 1, k))]) / s;
                            m_vel[index2GridOffset(glm::ivec3(i, j, k+1))].z -= (d * m_s[index2GridOffset(glm::ivec3(i, j, k + 1))]) / s;
                        }
                    }
                }
            }
        }
    }

    void Simulator::updateParticleDensity() {
        for(int i=0; i < m_iNumCells;i++) {
            m_particleDensity[i] = 0.0f;

            if(m_s[i] == 0.0f) {
                m_type[i] = SOLID_CELL;
            }
            else {
                m_type[i] = EMPTY_CELL;
            }
        }
        
        for (int i = 0; i < m_particlePos.size(); i++) {
            glm::vec3 pos = m_particlePos[i];
            glm::vec3 gridOffset = glm::vec3(-0.5f);
            
            glm::vec3 posRelGrid = pos - gridOffset;
            glm::ivec3 cellIndex = glm::ivec3(posRelGrid / m_h);

            m_particleDensity[index2GridOffset(cellIndex)] += 1;

            m_type[index2GridOffset(cellIndex)] = FLUID_CELL;
        }
    }

    void Simulator::pushParticlesApart(int numIters) {
        while(numIters--) {
            for(int i=0; i < m_particlePos.size(); i++) {
                for(int j=i+1; j < m_particlePos.size(); j++) {
                    glm::vec3 diff = m_particlePos[i] - m_particlePos[j];
                    float dist = glm::length(diff) + 0.0001f;
                    if(dist < 2.0f * m_particleRadius) {
                        glm::vec3 s = 0.5f * (2.0f * m_particleRadius - dist) * (diff) / dist;
                        m_particlePos[i] += s;
                        m_particlePos[j] -= s;
                    }
                }
            }
        }
    }
}