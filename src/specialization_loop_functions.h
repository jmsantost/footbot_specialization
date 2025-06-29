#ifndef SPECIALIZATION_LOOP_FUNCTIONS_H
#define SPECIALIZATION_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/core/utility/math/rng.h>
#include <map>
#include <string>

using namespace argos;

class CFootBotSpecialization;

class CSpecializationLoopFunctions : public CLoopFunctions {

public:
    CSpecializationLoopFunctions();
    virtual ~CSpecializationLoopFunctions() {}

    virtual void Init(TConfigurationNode& t_node);
    virtual void PostStep();
    virtual void Reset();

    bool TryToWorkOnTask(CLightEntity* pc_task, CFootBotSpecialization* pc_robot);
    CLightEntity* GetClosestTaskToPosition(const CVector2& c_pos);

private:
    struct SAssignedTask {
        CFootBotSpecialization* Robot;
        UInt32 TimeStarted;
        UInt32 WorkingTimeTicks;
        CColor OriginalColor;
    };

    std::map<CLightEntity*, SAssignedTask> m_mapAssignedTasks;
    std::vector<CLightEntity*> m_vecTasks;
    CRandom::CRNG* m_pcRNG;
    Real m_fTaskRatio;
};

#endif
