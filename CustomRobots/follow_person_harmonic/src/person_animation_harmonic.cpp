#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/common/Console.hh>
#include <sdf/Element.hh>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

namespace person_plugin
{
class PersonAnim :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
private:
    gz::sim::Entity actorEntity{gz::sim::kNullEntity};
    gz::transport::Node node;

    double animationFactor{4.0};   // segundos de animacion por metro recorrido

    double linearSpeed{0.0};  // (m/s)
    double angularSpeed{0.0};   // (rad/s)
    std::mutex mtx;

public:
    // ============================================
    // CONFIGURE
    // ============================================
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &) override
    {
        this->actorEntity = _entity;

        auto actorComp = _ecm.Component<gz::sim::components::Actor>(_entity);
        if (!actorComp) {
            gzerr << "[PersonAnim] Entity [" << _entity
                  << "] is not an actor." << std::endl;
            return;
        }

        // Parametros opcionales del SDF
        if (_sdf->HasElement("animation_factor"))
            this->animationFactor = _sdf->Get<double>("animation_factor");

        // Nombre de la animacion
        std::string animationName = "walk";
        if (_sdf->HasElement("animation"))
            animationName = _sdf->Get<std::string>("animation");

        auto animNameComp =
            _ecm.Component<gz::sim::components::AnimationName>(_entity);
        if (!animNameComp) {
            _ecm.CreateComponent(_entity,
                gz::sim::components::AnimationName(animationName));
        }
        else {
            *animNameComp = gz::sim::components::AnimationName(animationName);
        }
        _ecm.SetChanged(_entity,
            gz::sim::components::AnimationName::typeId,
            gz::sim::ComponentState::OneTimeChange);

        // Tiempo de animacion controlado por este plugin
        if (!_ecm.Component<gz::sim::components::AnimationTime>(_entity)) {
            _ecm.CreateComponent(_entity, gz::sim::components::AnimationTime());
        }

        // Pose inicial: el actor se controla en X/Y con TrajectoryPose
        gz::math::Pose3d initialPose;
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(_entity);
        if (!poseComp) {
            _ecm.CreateComponent(_entity, gz::sim::components::Pose(gz::math::Pose3d::Zero));
        }
        else {
            initialPose = poseComp->Data();

            auto newPose = initialPose;
            newPose.Pos().X(0);
            newPose.Pos().Y(0);
            *poseComp = gz::sim::components::Pose(newPose);
        }

        // Tener TrajectoryPose evita que el actor siga el script del SDF
        if (!_ecm.Component<gz::sim::components::TrajectoryPose>(_entity)) {
            initialPose.Pos().Z(0);
            _ecm.CreateComponent(_entity,
                gz::sim::components::TrajectoryPose(initialPose));
        }

        this->node.Subscribe("/person/cmd_vel", &PersonAnim::OnCmdVel, this);
    }

    // ============================================
    // CALLBACK CMD_VEL
    // ============================================
    void OnCmdVel(const gz::msgs::Twist &_msg)
    {
        std::lock_guard<std::mutex> lock(this->mtx);
        this->linearSpeed  = _msg.linear().x();
        this->angularSpeed = _msg.angular().z();
    }

    // ============================================
    // UPDATE
    // ============================================
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override
    {
        if (_info.paused || this->actorEntity == gz::sim::kNullEntity)
            return;

        auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity);
        auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(this->actorEntity);
        if (!trajPoseComp || !animTimeComp)
            return;

        const double dt = std::chrono::duration<double>(_info.dt).count();

        // Bloque de velocidades
        double lin, ang;
        {
            std::lock_guard<std::mutex> lock(this->mtx);
            lin = this->linearSpeed;
            ang = this->angularSpeed;
        }

        gz::math::Pose3d pose = trajPoseComp->Data();
        double yaw = pose.Rot().Euler().Z();
        double distanceTraveled = 0.0;

        if (std::abs(lin) > 0.001 || std::abs(ang) > 0.001)
        {
            // Rotacion incremental (rad/s * s)
            yaw += ang * dt;

            // Direccion de avance
            const double dx = lin * std::cos(yaw) * dt;
            const double dy = lin * std::sin(yaw) * dt;

            pose.Pos().X(pose.Pos().X() + dx);
            pose.Pos().Y(pose.Pos().Y() + dy);
            pose.Rot() = gz::math::Quaterniond(0, 0, yaw);

            distanceTraveled = std::sqrt(dx * dx + dy * dy);
        }

        *trajPoseComp = gz::sim::components::TrajectoryPose(pose);
        _ecm.SetChanged(this->actorEntity,
            gz::sim::components::TrajectoryPose::typeId,
            gz::sim::ComponentState::OneTimeChange);

        // La animacion avanza segun la distancia recorrida: si esta
        // quieto no camina, y si va mas rapido, camina mas rapido
        if (distanceTraveled > 0.0001)
        {
            auto animTime = animTimeComp->Data() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(
                        distanceTraveled * this->animationFactor));

            *animTimeComp = gz::sim::components::AnimationTime(animTime);
            _ecm.SetChanged(this->actorEntity,
                gz::sim::components::AnimationTime::typeId,
                gz::sim::ComponentState::OneTimeChange);
        }
    }
};
}

// ============================================
// REGISTRO DEL PLUGIN
// ============================================
GZ_ADD_PLUGIN(
    person_plugin::PersonAnim,
    gz::sim::System,
    person_plugin::PersonAnim::ISystemConfigure,
    person_plugin::PersonAnim::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(person_plugin::PersonAnim, "person_plugin::PersonAnim")
