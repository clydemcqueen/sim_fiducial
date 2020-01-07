#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{

  class MotionPlugin : public ModelPlugin
  {
    physics::LinkPtr link_;
    double duration_{0};
    double period_{5};
    ignition::math::Vector3d linear_vel_{0.1, 0, 0};
    ignition::math::Vector3d angular_vel_{0.5, 0, 0};

    ignition::math::Vector3d rev_linear_vel_;
    ignition::math::Vector3d rev_angular_vel_;
    ignition::math::Vector3d zero_vel_;

    event::ConnectionPtr update_connection_;

    bool running_{false};
    gazebo::common::Time start_time_;

  public:

    // Called once when the plugin is loaded.
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
      std::string link_name{"base_link"};

      std::cout << std::endl;
      std::cout << "MOTION PLUGIN PARAMETERS" << std::endl;
      std::cout << "-----------------------------------------" << std::endl;
      std::cout << "Default link name: " << link_name << std::endl;
      std::cout << "Default duration (0=no end): " << duration_ << std::endl;
      std::cout << "Default period: " << period_ << std::endl;
      std::cout << "Default linear_vel: " << linear_vel_ << std::endl;
      std::cout << "Default angular_vel: " << angular_vel_ << std::endl;

      GZ_ASSERT(model != nullptr, "Model is null");
      GZ_ASSERT(sdf != nullptr, "SDF is null");

      if (sdf->HasElement("link")) {
        sdf::ElementPtr linkElem = sdf->GetElement("link"); // Only one link is supported

        if (linkElem->HasAttribute("name")) {
          linkElem->GetAttribute("name")->Get(link_name);
          std::cout << "Link name: " << link_name << std::endl;
        }

        if (linkElem->HasElement("duration")) {
          duration_ = linkElem->GetElement("duration")->Get<double>();
          std::cout << "duration: " << duration_ << std::endl;
        }

        if (linkElem->HasElement("period")) {
          period_ = linkElem->GetElement("period")->Get<double>();
          std::cout << "period: " << period_ << std::endl;
        }

        if (linkElem->HasElement("linear_vel")) {
          linear_vel_ = linkElem->GetElement("linear_vel")->Get<ignition::math::Vector3d>();
          std::cout << "linear_vel: " << linear_vel_ << std::endl;
        }

        if (linkElem->HasElement("angular_vel")) {
          angular_vel_ = linkElem->GetElement("angular_vel")->Get<ignition::math::Vector3d>();
          std::cout << "angular_vel: " << angular_vel_ << std::endl;
        }
      }

      link_ = model->GetLink(link_name);
      GZ_ASSERT(link_ != nullptr, "Missing link");

      // Listen for the update event. This event is broadcast every simulation iteration.
      update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotionPlugin::OnUpdate, this, _1));

      std::cout << "-----------------------------------------" << std::endl;
      std::cout << std::endl;

      rev_linear_vel_ = ignition::math::Vector3d(-linear_vel_.X(), -linear_vel_.Y(), -linear_vel_.Z());
      rev_angular_vel_ = ignition::math::Vector3d(-angular_vel_.X(), -angular_vel_.Y(), -angular_vel_.Z());
      zero_vel_ = ignition::math::Vector3d{};
    }

    // Called by the world update start event, up to 1000 times per second.
    void OnUpdate(const common::UpdateInfo &info)
    {
      if (!running_) {
        running_ = true;
        start_time_ = info.simTime;
      }

      auto elapsed = (info.simTime - start_time_).Double();
      if (duration_ != 0.0 && elapsed > duration_) {
        // stop
        link_->SetLinearVel(zero_vel_);
        link_->SetAngularVel(zero_vel_);
      } else if (fmod(elapsed, period_) > period_ / 2) {
        // Forward
        link_->SetLinearVel(linear_vel_);
        link_->SetAngularVel(angular_vel_);
      } else {
        // Reverse
        link_->SetLinearVel(rev_linear_vel_);
        link_->SetAngularVel(rev_angular_vel_);
      }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(MotionPlugin)

}
