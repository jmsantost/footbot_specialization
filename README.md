Borra la carpeta Build


Crea una nueva carpeta build
para construir el paquete

    cmake ../src
    make

En Experiments está el .argos que se lanza así:

    argos3 -c footbot_specialization.argos

En src tienes que cambiar las direcciones para tu compu
También ahí está el .cpp del controlador del robot y de los loops function


# 🧠 Proyecto ARGoS: Especialización de FootBots

Este README explica cada línea relevante del código, con descripciones específicas sobre su propósito.


## 📂 Archivo: `specialization_loop_functions.h`

`#ifndef SPECIALIZATION_LOOP_FUNCTIONS_H`
→ Lógica específica o parte de un método.

`#define SPECIALIZATION_LOOP_FUNCTIONS_H`
→ Lógica específica o parte de un método.

`#include <argos3/core/simulator/loop_functions.h>`
→ Se importan las funciones base del ciclo de simulación de ARGoS.

`#include <argos3/plugins/simulator/entities/light_entity.h>`
→ Se importan las entidades de luz, que representan tareas en el entorno.

`#include <argos3/core/utility/math/rng.h>`
→ Importa un módulo de ARGoS (simulador, sensores, actuadores, matemáticas, logging, etc.).

`#include <map>`
→ Se importa la estructura de mapa (`std::map`) para relacionar tareas con robots.

`#include <string>`
→ Se importa la clase `std::string` para trabajar con cadenas de texto.

`using namespace argos;`
→ Permite utilizar directamente las clases de ARGoS sin el prefijo `argos::`.

`class CFootBotSpecialization;`
→ Declaración anticipada de una clase.

`class CSpecializationLoopFunctions : public CLoopFunctions {`
→ Definición de una clase y su herencia.

`public:`
→ Sección pública de la clase: interfaz accesible externamente.

`CSpecializationLoopFunctions();`
→ Declaración de un método vacío o inline.

`virtual ~CSpecializationLoopFunctions() {}`
→ Cierre de un bloque de código.

`virtual void Init(TConfigurationNode& t_node);`
→ Método para inicializar el objeto o leer configuración.

`virtual void PostStep();`
→ Método que se ejecuta después de cada paso del simulador.

`virtual void Reset();`
→ Método para reiniciar el estado interno del objeto.

`bool TryToWorkOnTask(CLightEntity* pc_task, CFootBotSpecialization* pc_robot);`
→ Lógica específica o parte de un método.

`CLightEntity* GetClosestTaskToPosition(const CVector2& c_pos);`
→ Lógica específica o parte de un método.

`private:`
→ Sección privada de la clase: solo accesible desde métodos internos.

`struct SAssignedTask {`
→ Lógica específica o parte de un método.

`CFootBotSpecialization* Robot;`
→ Lógica específica o parte de un método.

`UInt32 TimeStarted;`
→ Lógica específica o parte de un método.

`UInt32 WorkingTimeTicks;`
→ Lógica específica o parte de un método.

`CColor OriginalColor;`
→ Lógica específica o parte de un método.

`};`
→ Cierre de un bloque de código.

`std::map<CLightEntity*, SAssignedTask> m_mapAssignedTasks;`
→ Inicio de la implementación de un método.

`std::vector<CLightEntity*> m_vecTasks;`
→ Inicio de la implementación de un método.

`CRandom::CRNG* m_pcRNG;`
→ Inicio de la implementación de un método.

`Real m_fTaskRatio;`
→ Lógica específica o parte de un método.

`};`
→ Cierre de un bloque de código.

`#endif`
→ Lógica específica o parte de un método.


## 📂 Archivo: `specialization_loop_functions.cpp`

`#include "specialization_loop_functions.h"`
→ Se importa el encabezado de la clase que implementa las funciones del ciclo del simulador.

`#include <argos3/core/simulator/simulator.h>`
→ Se importa el núcleo del simulador de ARGoS.

`#include <argos3/core/utility/math/general.h>`
→ Se importan funciones matemáticas generales de ARGoS.

`#include <argos3/core/utility/logging/argos_log.h>`
→ Se importa el sistema de registro para imprimir mensajes en consola.

`#include "footbot_specialization.h"`
→ Se importa el controlador del robot FootBot que contiene su lógica de comportamiento.

`#include <limits>`
→ Se importa `<limits>` para acceder a valores máximos y mínimos numéricos.

`CSpecializationLoopFunctions::CSpecializationLoopFunctions() :`
→ Inicio de la implementación de un método.

`m_pcRNG(nullptr),`
→ Lógica específica o parte de un método.

`m_fTaskRatio(0.5) {}`
→ Cierre de un bloque de código.

`void CSpecializationLoopFunctions::Init(TConfigurationNode& t_node) {`
→ Método para inicializar el objeto o leer configuración.

`m_pcRNG = CRandom::CreateRNG("argos");`
→ Inicio de la implementación de un método.

`TConfigurationNode tParams;`
→ Lógica específica o parte de un método.

`try {`
→ Lógica específica o parte de un método.

`tParams = GetNode(t_node, "params");`
→ Inicialización de un atributo o variable.

`GetNodeAttributeOrDefault(tParams, "task_ratio", m_fTaskRatio, m_fTaskRatio);`
→ Lógica específica o parte de un método.

`} catch(CARGoSException& ex) {}`
→ Cierre de un bloque de código.

`CSpace::TMapPerType& mapLights = GetSpace().GetEntitiesByType("light");`
→ Inicio de la implementación de un método.

`for (auto const& [key, val] : mapLights) {`
→ Lógica específica o parte de un método.

`if(key.find("fb") == std::string::npos) {`
→ Lógica específica o parte de un método.

`m_vecTasks.push_back(any_cast<CLightEntity*>(val));`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`Reset();`
→ Método para reiniciar el estado interno del objeto.

`}`
→ Cierre de un bloque de código.

`void CSpecializationLoopFunctions::Reset() {`
→ Método para reiniciar el estado interno del objeto.

`m_mapAssignedTasks.clear();`
→ Declaración de un método vacío o inline.

`for (CLightEntity* pc_task : m_vecTasks) {`
→ Lógica específica o parte de un método.

`if (m_pcRNG->Uniform(CRange<Real>(0.0, 1.0)) < m_fTaskRatio) {`
→ Lógica específica o parte de un método.

`pc_task->SetColor(CColor::BLUE);`
→ Inicio de la implementación de un método.

`} else {`
→ Lógica específica o parte de un método.

`pc_task->SetColor(CColor::GREEN);`
→ Inicio de la implementación de un método.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`bool CSpecializationLoopFunctions::TryToWorkOnTask(CLightEntity* pc_task, CFootBotSpecialization* pc_robot) {`
→ Lógica específica o parte de un método.

`if (m_mapAssignedTasks.count(pc_task)) {`
→ Lógica específica o parte de un método.

`LOG << "[LoopFunc] Task " << pc_task->GetId() << " assignment failed (already taken)." << std::endl;`
→ Inicio de la implementación de un método.

`return false;`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`m_mapAssignedTasks[pc_task] = {`
→ Lógica específica o parte de un método.

`pc_robot,`
→ Lógica específica o parte de un método.

`GetSpace().GetSimulationClock(),`
→ Lógica específica o parte de un método.

`pc_robot->GetWorkTimeInTicks(),`
→ Lógica específica o parte de un método.

`pc_task->GetColor()`
→ Lógica específica o parte de un método.

`};`
→ Cierre de un bloque de código.

`pc_task->SetColor(CColor::RED);`
→ Inicio de la implementación de un método.

`LOG << "[LoopFunc] Task " << pc_task->GetId() << " assigned to robot " << pc_robot->GetId() << std::endl;`
→ Inicio de la implementación de un método.

`return true;`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`void CSpecializationLoopFunctions::PostStep() {`
→ Método que se ejecuta después de cada paso del simulador.

`for (auto it = m_mapAssignedTasks.begin(); it != m_mapAssignedTasks.end(); ) {`
→ Inicialización de un atributo o variable.

`SAssignedTask& s_assignment = it->second;`
→ Inicialización de un atributo o variable.

`UInt32 unElapsedTime = GetSpace().GetSimulationClock() - s_assignment.TimeStarted;`
→ Inicialización de un atributo o variable.

`if (unElapsedTime >= s_assignment.WorkingTimeTicks) {`
→ Lógica específica o parte de un método.

`LOG << "[LoopFunc] Task " << it->first->GetId() << " time expired. Notifying robot." << std::endl;`
→ Inicio de la implementación de un método.

`s_assignment.Robot->TaskCompleted(s_assignment.OriginalColor);`
→ Lógica específica o parte de un método.

`if (m_pcRNG->Uniform(CRange<Real>(0.0, 1.0)) < m_fTaskRatio) {`
→ Lógica específica o parte de un método.

`it->first->SetColor(CColor::BLUE);`
→ Inicio de la implementación de un método.

`} else {`
→ Lógica específica o parte de un método.

`it->first->SetColor(CColor::GREEN);`
→ Inicio de la implementación de un método.

`}`
→ Cierre de un bloque de código.

`it = m_mapAssignedTasks.erase(it);`
→ Inicialización de un atributo o variable.

`} else {`
→ Lógica específica o parte de un método.

`++it;`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`CLightEntity* CSpecializationLoopFunctions::GetClosestTaskToPosition(const CVector2& c_pos) {`
→ Lógica específica o parte de un método.

`if (m_vecTasks.empty()) return nullptr;`
→ Lógica específica o parte de un método.

`CLightEntity* pcClosestTask = nullptr;`
→ Inicialización de un atributo o variable.

`Real fMinDistSq = std::numeric_limits<Real>::max();`
→ Inicio de la implementación de un método.

`for (CLightEntity* pc_task : m_vecTasks) {`
→ Lógica específica o parte de un método.

`// Filtra tareas ya asignadas para que los robots no compitan por ellas`
→ Lógica específica o parte de un método.

`if (m_mapAssignedTasks.count(pc_task) == 0) {`
→ Lógica específica o parte de un método.

`CVector2 cTaskPos(pc_task->GetPosition().GetX(),`
→ Lógica específica o parte de un método.

`pc_task->GetPosition().GetY());`
→ Lógica específica o parte de un método.

`Real fDistSq = SquareDistance(cTaskPos, c_pos);`
→ Inicialización de un atributo o variable.

`if (fDistSq < fMinDistSq) {`
→ Lógica específica o parte de un método.

`fMinDistSq = fDistSq;`
→ Inicialización de un atributo o variable.

`pcClosestTask = pc_task;`
→ Inicialización de un atributo o variable.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`return pcClosestTask;`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`REGISTER_LOOP_FUNCTIONS(CSpecializationLoopFunctions, "specialization_loop_functions");`
→ Lógica específica o parte de un método.


## 📂 Archivo: `footbot_specialization.h`

`#ifndef FOOTBOT_SPECIALIZATION_H`
→ Lógica específica o parte de un método.

`#define FOOTBOT_SPECIALIZATION_H`
→ Lógica específica o parte de un método.

`#include <argos3/core/control_interface/ci_controller.h>`
→ Importa un módulo de ARGoS (simulador, sensores, actuadores, matemáticas, logging, etc.).

`#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>`
→ Se importa el actuador de ruedas diferenciales del robot.

`#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>`
→ Se importan los actuadores de LEDs del robot.

`#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>`
→ Se importa el sensor de proximidad del robot FootBot.

`#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>`
→ Se importa el sensor de posicionamiento (GPS) del robot.

`#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>`
→ Se importa la cámara omnidireccional para detectar blobs de colores (tareas).

`#include <argos3/core/utility/math/rng.h>`
→ Importa un módulo de ARGoS (simulador, sensores, actuadores, matemáticas, logging, etc.).

`using namespace argos;`
→ Permite utilizar directamente las clases de ARGoS sin el prefijo `argos::`.

`class CSpecializationLoopFunctions;`
→ Declaración anticipada de una clase.

`class CFootBotSpecialization : public CCI_Controller {`
→ Definición de una clase y su herencia.

`public:`
→ Sección pública de la clase: interfaz accesible externamente.

`enum EState {`
→ Lógica específica o parte de un método.

`STATE_SEARCHING,`
→ Lógica específica o parte de un método.

`STATE_GO_TO_TASK,`
→ Lógica específica o parte de un método.

`STATE_WORKING`
→ Lógica específica o parte de un método.

`};`
→ Cierre de un bloque de código.

`public:`
→ Sección pública de la clase: interfaz accesible externamente.

`CFootBotSpecialization();`
→ Declaración de un método vacío o inline.

`virtual ~CFootBotSpecialization() {}`
→ Cierre de un bloque de código.

`virtual void Init(TConfigurationNode& t_node);`
→ Método para inicializar el objeto o leer configuración.

`virtual void ControlStep();`
→ Declaración de un método vacío o inline.

`virtual void Reset();`
→ Método para reiniciar el estado interno del objeto.

`virtual void Destroy() {}`
→ Cierre de un bloque de código.

`void TaskCompleted(const CColor& c_task_color);`
→ Lógica específica o parte de un método.

`UInt32 GetWorkTimeInTicks();`
→ Declaración de un método vacío o inline.

`private:`
→ Sección privada de la clase: solo accesible desde métodos internos.

`void Search();`
→ Declaración de un método vacío o inline.

`void GoToTask();`
→ Declaración de un método vacío o inline.

`void Work();`
→ Declaración de un método vacío o inline.

`// ========================================================`
→ Lógica específica o parte de un método.

`// ==         AQUÍ ESTÁ LA LÍNEA QUE FALTABA             ==`
→ Lógica específica o parte de un método.

`// ========================================================`
→ Lógica específica o parte de un método.

`bool ObstacleAvoidance();`
→ Declaración de un método vacío o inline.

`const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* GetClosestAvailableTask();`
→ Inicio de la implementación de un método.

`private:`
→ Sección privada de la clase: solo accesible desde métodos internos.

`/* Punteros a sensores y actuadores */`
→ Lógica específica o parte de un método.

`CCI_DifferentialSteeringActuator* m_pcWheels;`
→ Lógica específica o parte de un método.

`CCI_LEDsActuator* m_pcLEDs;`
→ Lógica específica o parte de un método.

`CCI_FootBotProximitySensor* m_pcProximity;`
→ Lógica específica o parte de un método.

`CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;`
→ Lógica específica o parte de un método.

`CCI_PositioningSensor* m_pcPositioning;`
→ Lógica específica o parte de un método.

`CSpecializationLoopFunctions* m_pcLoopFunctions;`
→ Lógica específica o parte de un método.

`EState m_eState;`
→ Lógica específica o parte de un método.

`const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* m_psTargetTaskBlob;`
→ Inicio de la implementación de un método.

`Real m_fMaxSpeed;`
→ Lógica específica o parte de un método.

`Real m_fArrivalDistance;`
→ Lógica específica o parte de un método.

`static const UInt32 TICKS_PER_SECOND = 10;`
→ Inicialización de un atributo o variable.

`};`
→ Cierre de un bloque de código.

`#endif`
→ Lógica específica o parte de un método.


## 📂 Archivo: `footbot_specialization.cpp`

`#include "footbot_specialization.h"`
→ Se importa el controlador del robot FootBot que contiene su lógica de comportamiento.

`#include "specialization_loop_functions.h"`
→ Se importa el encabezado de la clase que implementa las funciones del ciclo del simulador.

`#include <argos3/core/simulator/simulator.h>`
→ Se importa el núcleo del simulador de ARGoS.

`#include <argos3/core/utility/logging/argos_log.h>`
→ Se importa el sistema de registro para imprimir mensajes en consola.

`#include <argos3/core/utility/math/vector2.h>`
→ Importa un módulo de ARGoS (simulador, sensores, actuadores, matemáticas, logging, etc.).

`#include <limits>`
→ Se importa `<limits>` para acceder a valores máximos y mínimos numéricos.

`CFootBotSpecialization::CFootBotSpecialization() :`
→ Inicio de la implementación de un método.

`m_pcWheels(nullptr), m_pcLEDs(nullptr), m_pcProximity(nullptr), m_pcCamera(nullptr),`
→ Lógica específica o parte de un método.

`m_pcPositioning(nullptr), m_pcLoopFunctions(nullptr), m_eState(STATE_SEARCHING),`
→ Lógica específica o parte de un método.

`m_psTargetTaskBlob(nullptr), m_fMaxSpeed(8.0f), m_fArrivalDistance(0.1f) {}`
→ Cierre de un bloque de código.

`void CFootBotSpecialization::Init(TConfigurationNode& t_node) {`
→ Método para inicializar el objeto o leer configuración.

`m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");`
→ Inicialización de un atributo o variable.

`m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");`
→ Inicialización de un atributo o variable.

`m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");`
→ Inicialización de un atributo o variable.

`m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");`
→ Inicialización de un atributo o variable.

`m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");`
→ Inicialización de un atributo o variable.

`m_pcCamera->Enable();`
→ Declaración de un método vacío o inline.

`GetNodeAttributeOrDefault(t_node, "max_speed", m_fMaxSpeed, m_fMaxSpeed);`
→ Lógica específica o parte de un método.

`m_pcLoopFunctions = &dynamic_cast<CSpecializationLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions());`
→ Inicio de la implementación de un método.

`Reset();`
→ Método para reiniciar el estado interno del objeto.

`}`
→ Cierre de un bloque de código.

`void CFootBotSpecialization::Reset() {`
→ Método para reiniciar el estado interno del objeto.

`m_eState = STATE_SEARCHING;`
→ Inicialización de un atributo o variable.

`m_psTargetTaskBlob = nullptr;`
→ Inicialización de un atributo o variable.

`m_pcLEDs->SetAllColors(CColor::BLACK);`
→ Inicio de la implementación de un método.

`RLOG << GetId() << ": [Reset] New state: SEARCHING" << std::endl;`
→ Inicio de la implementación de un método.

`}`
→ Cierre de un bloque de código.

`void CFootBotSpecialization::ControlStep() {`
→ Lógica específica o parte de un método.

`if (ObstacleAvoidance()) return;`
→ Lógica específica o parte de un método.

`switch(m_eState) {`
→ Lógica específica o parte de un método.

`case STATE_SEARCHING:    Search();   break;`
→ Lógica específica o parte de un método.

`case STATE_GO_TO_TASK:   GoToTask(); break;`
→ Lógica específica o parte de un método.

`case STATE_WORKING:      Work();     break;`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`bool CFootBotSpecialization::ObstacleAvoidance() {`
→ Lógica específica o parte de un método.

`const auto& tProxReads = m_pcProximity->GetReadings();`
→ Declaración de un método vacío o inline.

`CVector2 cAccumulator;`
→ Lógica específica o parte de un método.

`for (const auto& tRead : tProxReads) { cAccumulator += CVector2(tRead.Value, tRead.Angle); }`
→ Inicialización de un atributo o variable.

`cAccumulator /= tProxReads.size();`
→ Declaración de un método vacío o inline.

`if (cAccumulator.Length() > 0.05f) {`
→ Lógica específica o parte de un método.

`CRadians cAngle = cAccumulator.Angle();`
→ Declaración de un método vacío o inline.

`if (cAngle.GetValue() > 0) m_pcWheels->SetLinearVelocity(m_fMaxSpeed, 0.0f);`
→ Lógica específica o parte de un método.

`else m_pcWheels->SetLinearVelocity(0.0f, m_fMaxSpeed);`
→ Lógica específica o parte de un método.

`return true;`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`return false;`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`void CFootBotSpecialization::Search() {`
→ Lógica específica o parte de un método.

`m_pcLEDs->SetAllColors(CColor::YELLOW);`
→ Inicio de la implementación de un método.

`m_pcWheels->SetLinearVelocity(m_fMaxSpeed, m_fMaxSpeed);`
→ Lógica específica o parte de un método.

`// ========================================================`
→ Lógica específica o parte de un método.

`// ==         NUEVA LÍNEA DE DEBUG AQUÍ                  ==`
→ Lógica específica o parte de un método.

`// ========================================================`
→ Lógica específica o parte de un método.

`const auto& tBlobs = m_pcCamera->GetReadings().BlobList;`
→ Inicialización de un atributo o variable.

`RLOG << GetId() << ": Searching... " << tBlobs.size() << " blobs detected." << std::endl;`
→ Inicio de la implementación de un método.

`m_psTargetTaskBlob = GetClosestAvailableTask();`
→ Declaración de un método vacío o inline.

`if (m_psTargetTaskBlob != nullptr) {`
→ Lógica específica o parte de un método.

`m_eState = STATE_GO_TO_TASK;`
→ Inicialización de un atributo o variable.

`RLOG << GetId() << ": Task found! New state: GO_TO_TASK" << std::endl;`
→ Inicio de la implementación de un método.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`void CFootBotSpecialization::GoToTask() {`
→ Lógica específica o parte de un método.

`m_pcLEDs->SetAllColors(CColor::ORANGE);`
→ Inicio de la implementación de un método.

`m_psTargetTaskBlob = GetClosestAvailableTask();`
→ Declaración de un método vacío o inline.

`if (m_psTargetTaskBlob == nullptr) {`
→ Lógica específica o parte de un método.

`m_eState = STATE_SEARCHING;`
→ Inicialización de un atributo o variable.

`RLOG << GetId() << ": Target lost. New state: SEARCHING" << std::endl;`
→ Inicio de la implementación de un método.

`return;`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`if (m_psTargetTaskBlob->Distance < m_fArrivalDistance) {`
→ Lógica específica o parte de un método.

`m_pcWheels->SetLinearVelocity(0.0f, 0.0f);`
→ Lógica específica o parte de un método.

`const CVector3& cRobotPos = m_pcPositioning->GetReading().Position;`
→ Inicialización de un atributo o variable.

`const CQuaternion& cRobotOrient = m_pcPositioning->GetReading().Orientation;`
→ Inicialización de un atributo o variable.

`CRadians cZ, cY, cX;`
→ Lógica específica o parte de un método.

`cRobotOrient.ToEulerAngles(cZ, cY, cX);`
→ Lógica específica o parte de un método.

`CVector2 cBlobRelPos(m_psTargetTaskBlob->Distance, m_psTargetTaskBlob->Angle);`
→ Lógica específica o parte de un método.

`CVector2 cBlobAbsPos = cBlobRelPos.Rotate(cZ) + CVector2(cRobotPos.GetX(), cRobotPos.GetY());`
→ Inicialización de un atributo o variable.

`CLightEntity* pcLight = m_pcLoopFunctions->GetClosestTaskToPosition(cBlobAbsPos);`
→ Inicialización de un atributo o variable.

`if (pcLight && m_pcLoopFunctions->TryToWorkOnTask(pcLight, this)) {`
→ Lógica específica o parte de un método.

`m_eState = STATE_WORKING;`
→ Inicialización de un atributo o variable.

`RLOG << GetId() << ": Task assigned. New state: WORKING" << std::endl;`
→ Inicio de la implementación de un método.

`} else {`
→ Lógica específica o parte de un método.

`m_eState = STATE_SEARCHING;`
→ Inicialización de un atributo o variable.

`m_psTargetTaskBlob = nullptr;`
→ Inicialización de un atributo o variable.

`RLOG << GetId() << ": Failed to get task. New state: SEARCHING" << std::endl;`
→ Inicio de la implementación de un método.

`}`
→ Cierre de un bloque de código.

`} else {`
→ Lógica específica o parte de un método.

`CRadians angle = m_psTargetTaskBlob->Angle;`
→ Inicialización de un atributo o variable.

`angle.SignedNormalize();`
→ Declaración de un método vacío o inline.

`if (angle.GetValue() > 0.1) m_pcWheels->SetLinearVelocity(m_fMaxSpeed * 0.5f, m_fMaxSpeed);`
→ Lógica específica o parte de un método.

`else if (angle.GetValue() < -0.1) m_pcWheels->SetLinearVelocity(m_fMaxSpeed, m_fMaxSpeed * 0.5f);`
→ Lógica específica o parte de un método.

`else m_pcWheels->SetLinearVelocity(m_fMaxSpeed, m_fMaxSpeed);`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`void CFootBotSpecialization::Work() {`
→ Lógica específica o parte de un método.

`m_pcLEDs->SetAllColors(CColor::RED);`
→ Inicio de la implementación de un método.

`m_pcWheels->SetLinearVelocity(0.0f, 0.0f);`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* CFootBotSpecialization::GetClosestAvailableTask() {`
→ Lógica específica o parte de un método.

`const auto& tBlobs = m_pcCamera->GetReadings().BlobList;`
→ Inicialización de un atributo o variable.

`const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* sClosestBlob = nullptr;`
→ Inicio de la implementación de un método.

`Real fMinDistance = std::numeric_limits<Real>::max();`
→ Inicio de la implementación de un método.

`for (const auto& pBlob : tBlobs) {`
→ Lógica específica o parte de un método.

`if ((pBlob->Color == CColor::BLUE || pBlob->Color == CColor::GREEN) && pBlob->Distance < fMinDistance) {`
→ Lógica específica o parte de un método.

`fMinDistance = pBlob->Distance;`
→ Inicialización de un atributo o variable.

`sClosestBlob = pBlob;`
→ Inicialización de un atributo o variable.

`}`
→ Cierre de un bloque de código.

`}`
→ Cierre de un bloque de código.

`return sClosestBlob;`
→ Lógica específica o parte de un método.

`}`
→ Cierre de un bloque de código.

`UInt32 CFootBotSpecialization::GetWorkTimeInTicks() { return 5 * TICKS_PER_SECOND; }`
→ Cierre de un bloque de código.

`void CFootBotSpecialization::TaskCompleted(const CColor& c_task_color) {`
→ Lógica específica o parte de un método.

`m_eState = STATE_SEARCHING;`
→ Inicialización de un atributo o variable.

`RLOG << GetId() << ": Task completed. New state: SEARCHING" << std::endl;`
→ Inicio de la implementación de un método.

`m_psTargetTaskBlob = nullptr;`
→ Inicialización de un atributo o variable.

`}`
→ Cierre de un bloque de código.

`REGISTER_CONTROLLER(CFootBotSpecialization, "footbot_specialization_controller");`
→ Lógica específica o parte de un método.
