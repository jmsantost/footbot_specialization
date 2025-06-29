#include "specialization_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/general.h>
#include <argos3/core/utility/logging/argos_log.h>
#include "footbot_specialization.h"
#include <limits>

CSpecializationLoopFunctions::CSpecializationLoopFunctions() :
    m_pcRNG(nullptr),
    m_fTaskRatio(0.5) {}

void CSpecializationLoopFunctions::Init(TConfigurationNode& t_node) {
    m_pcRNG = CRandom::CreateRNG("argos");
    TConfigurationNode tParams;
    try {
        tParams = GetNode(t_node, "params");
        GetNodeAttributeOrDefault(tParams, "task_ratio", m_fTaskRatio, m_fTaskRatio);
    } catch(CARGoSException& ex) {}

    CSpace::TMapPerType& mapLights = GetSpace().GetEntitiesByType("light");
    for (auto const& [key, val] : mapLights) {
        if(key.find("fb") == std::string::npos) {
            m_vecTasks.push_back(any_cast<CLightEntity*>(val));
        }
    }
    Reset();
}

void CSpecializationLoopFunctions::Reset() {
    m_mapAssignedTasks.clear();
    for (CLightEntity* pc_task : m_vecTasks) {
        if (m_pcRNG->Uniform(CRange<Real>(0.0, 1.0)) < m_fTaskRatio) {
            pc_task->SetColor(CColor::BLUE);
        } else {
            pc_task->SetColor(CColor::GREEN);
        }
    }
}

bool CSpecializationLoopFunctions::TryToWorkOnTask(CLightEntity* pc_task, CFootBotSpecialization* pc_robot) {
    if (m_mapAssignedTasks.count(pc_task)) {
        LOG << "[LoopFunc] Task " << pc_task->GetId() << " assignment failed (already taken)." << std::endl;
        return false;
    }
    m_mapAssignedTasks[pc_task] = {
        pc_robot,
        GetSpace().GetSimulationClock(),
        pc_robot->GetWorkTimeInTicks(),
        pc_task->GetColor()
    };
    pc_task->SetColor(CColor::RED);
    LOG << "[LoopFunc] Task " << pc_task->GetId() << " assigned to robot " << pc_robot->GetId() << std::endl;
    return true;
}

void CSpecializationLoopFunctions::PostStep() {
    for (auto it = m_mapAssignedTasks.begin(); it != m_mapAssignedTasks.end(); ) {
        SAssignedTask& s_assignment = it->second;
        UInt32 unElapsedTime = GetSpace().GetSimulationClock() - s_assignment.TimeStarted;
        if (unElapsedTime >= s_assignment.WorkingTimeTicks) {
            LOG << "[LoopFunc] Task " << it->first->GetId() << " time expired. Notifying robot." << std::endl;

            s_assignment.Robot->TaskCompleted(s_assignment.OriginalColor);
            
            if (m_pcRNG->Uniform(CRange<Real>(0.0, 1.0)) < m_fTaskRatio) {
                it->first->SetColor(CColor::BLUE);
            } else {
                it->first->SetColor(CColor::GREEN);
            }
            it = m_mapAssignedTasks.erase(it);
        } else {
            ++it;
        }
    }
}

CLightEntity* CSpecializationLoopFunctions::GetClosestTaskToPosition(const CVector2& c_pos) {
    if (m_vecTasks.empty()) return nullptr;
    CLightEntity* pcClosestTask = nullptr;
    Real fMinDistSq = std::numeric_limits<Real>::max();
    for (CLightEntity* pc_task : m_vecTasks) {
        // Filtra tareas ya asignadas para que los robots no compitan por ellas
        if (m_mapAssignedTasks.count(pc_task) == 0) {
            CVector2 cTaskPos(pc_task->GetPosition().GetX(),
                              pc_task->GetPosition().GetY());
            Real fDistSq = SquareDistance(cTaskPos, c_pos);
            if (fDistSq < fMinDistSq) {
                fMinDistSq = fDistSq;
                pcClosestTask = pc_task;
            }
        }
    }
    return pcClosestTask;
}

REGISTER_LOOP_FUNCTIONS(CSpecializationLoopFunctions, "specialization_loop_functions");
