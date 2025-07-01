#include "footbot_specialization.h"
#include "specialization_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector2.h>
#include <limits>

CFootBotSpecialization::CFootBotSpecialization() :
   m_pcWheels(nullptr), m_pcLEDs(nullptr), m_pcProximity(nullptr), m_pcCamera(nullptr),
   m_pcPositioning(nullptr), m_pcLoopFunctions(nullptr), m_eState(STATE_SEARCHING),
   m_psTargetTaskBlob(nullptr), m_fMaxSpeed(8.0f), m_fArrivalDistance(0.1f) {}

void CFootBotSpecialization::Init(TConfigurationNode& t_node) {
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
   m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
   m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
   m_pcCamera->Enable();
   GetNodeAttributeOrDefault(t_node, "max_speed", m_fMaxSpeed, m_fMaxSpeed);
   m_pcLoopFunctions = &dynamic_cast<CSpecializationLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions());
   Reset();
}

void CFootBotSpecialization::Reset() {
   m_eState = STATE_SEARCHING;
   m_psTargetTaskBlob = nullptr;
   m_pcLEDs->SetAllColors(CColor::BLACK);
   RLOG << GetId() << ": [Reset] New state: SEARCHING" << std::endl;
}

void CFootBotSpecialization::ControlStep() {
    if (ObstacleAvoidance()) return;
    switch(m_eState) {
        case STATE_SEARCHING:    Search();   break;
        case STATE_GO_TO_TASK:   GoToTask(); break;
        case STATE_WORKING:      Work();     break;
    }
}

bool CFootBotSpecialization::ObstacleAvoidance() {
    const auto& tProxReads = m_pcProximity->GetReadings();
    CVector2 cAccumulator;
    for (const auto& tRead : tProxReads) { cAccumulator += CVector2(tRead.Value, tRead.Angle); }
    cAccumulator /= tProxReads.size();
    if (cAccumulator.Length() > 0.05f) {
        CRadians cAngle = cAccumulator.Angle();
        if (cAngle.GetValue() > 0) m_pcWheels->SetLinearVelocity(m_fMaxSpeed, 0.0f);
        else m_pcWheels->SetLinearVelocity(0.0f, m_fMaxSpeed);
        return true;
    }
    return false;
}

void CFootBotSpecialization::Search() {
    m_pcLEDs->SetAllColors(CColor::YELLOW);
    m_pcWheels->SetLinearVelocity(m_fMaxSpeed, m_fMaxSpeed);
    const auto& tBlobs = m_pcCamera->GetReadings().BlobList;
    RLOG << GetId() << ": Searching... " << tBlobs.size() << " blobs detected." << std::endl;

    m_psTargetTaskBlob = GetClosestAvailableTask();
    if (m_psTargetTaskBlob != nullptr) {
        m_eState = STATE_GO_TO_TASK;
        RLOG << GetId() << ": Task found! New state: GO_TO_TASK" << std::endl;
    }
}

void CFootBotSpecialization::GoToTask() {
    m_pcLEDs->SetAllColors(CColor::ORANGE);
    m_psTargetTaskBlob = GetClosestAvailableTask();
    if (m_psTargetTaskBlob == nullptr) {
        m_eState = STATE_SEARCHING;
        RLOG << GetId() << ": Target lost. New state: SEARCHING" << std::endl;
        return;
    }
    if (m_psTargetTaskBlob->Distance < m_fArrivalDistance) {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        const CVector3& cRobotPos = m_pcPositioning->GetReading().Position;
        const CQuaternion& cRobotOrient = m_pcPositioning->GetReading().Orientation;
        CRadians cZ, cY, cX;
        cRobotOrient.ToEulerAngles(cZ, cY, cX);
        CVector2 cBlobRelPos(m_psTargetTaskBlob->Distance, m_psTargetTaskBlob->Angle);
        CVector2 cBlobAbsPos = cBlobRelPos.Rotate(cZ) + CVector2(cRobotPos.GetX(), cRobotPos.GetY());
        CLightEntity* pcLight = m_pcLoopFunctions->GetClosestTaskToPosition(cBlobAbsPos);
        if (pcLight && m_pcLoopFunctions->TryToWorkOnTask(pcLight, this)) {
            m_eState = STATE_WORKING;
            RLOG << GetId() << ": Task assigned. New state: WORKING" << std::endl;
        } else {
            m_eState = STATE_SEARCHING;
            m_psTargetTaskBlob = nullptr;
            RLOG << GetId() << ": Failed to get task. New state: SEARCHING" << std::endl;
        }
    } else {
        CRadians angle = m_psTargetTaskBlob->Angle;
        angle.SignedNormalize();
        if (angle.GetValue() > 0.1) m_pcWheels->SetLinearVelocity(m_fMaxSpeed * 0.5f, m_fMaxSpeed);
        else if (angle.GetValue() < -0.1) m_pcWheels->SetLinearVelocity(m_fMaxSpeed, m_fMaxSpeed * 0.5f);
        else m_pcWheels->SetLinearVelocity(m_fMaxSpeed, m_fMaxSpeed);
    }
}

void CFootBotSpecialization::Work() {
    m_pcLEDs->SetAllColors(CColor::RED);
    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* CFootBotSpecialization::GetClosestAvailableTask() {
    const auto& tBlobs = m_pcCamera->GetReadings().BlobList;
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* sClosestBlob = nullptr;
    Real fMinDistance = std::numeric_limits<Real>::max();
    for (const auto& pBlob : tBlobs) {
        if ((pBlob->Color == CColor::BLUE || pBlob->Color == CColor::GREEN) && pBlob->Distance < fMinDistance) {
            fMinDistance = pBlob->Distance;
            sClosestBlob = pBlob;
        }
    }
    return sClosestBlob;
}

UInt32 CFootBotSpecialization::GetWorkTimeInTicks() { return 5 * TICKS_PER_SECOND; }

void CFootBotSpecialization::TaskCompleted(const CColor& c_task_color) {
    m_eState = STATE_SEARCHING;
    RLOG << GetId() << ": Task completed. New state: SEARCHING" << std::endl;
    m_psTargetTaskBlob = nullptr;
}

REGISTER_CONTROLLER(CFootBotSpecialization, "footbot_specialization_controller");
