#ifndef FOOTBOT_SPECIALIZATION_H
#define FOOTBOT_SPECIALIZATION_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CSpecializationLoopFunctions;

class CFootBotSpecialization : public CCI_Controller {

public:
    enum EState {
        STATE_SEARCHING,
        STATE_GO_TO_TASK,
        STATE_WORKING
    };
    
public:
    CFootBotSpecialization();
    virtual ~CFootBotSpecialization() {}

    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();
    virtual void Reset();
    virtual void Destroy() {}

    void TaskCompleted(const CColor& c_task_color);
    UInt32 GetWorkTimeInTicks();

private:
    void Search();
    void GoToTask();
    void Work();
    bool ObstacleAvoidance();
    
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* GetClosestAvailableTask();

private:
    /* Punteros a sensores y actuadores */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    CCI_LEDsActuator* m_pcLEDs;
    CCI_FootBotProximitySensor* m_pcProximity;
    CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
    CCI_PositioningSensor* m_pcPositioning;

    CSpecializationLoopFunctions* m_pcLoopFunctions;

    EState m_eState;
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* m_psTargetTaskBlob;
    
    Real m_fMaxSpeed;
    Real m_fArrivalDistance;
    
    static const UInt32 TICKS_PER_SECOND = 10;
};

#endif
