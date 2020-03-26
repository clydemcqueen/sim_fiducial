

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include <gazebo_ros/node.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace sim_fiducial_gazebo
{

  class ElementalCameraPlugin : public gazebo::ModelPlugin
  {
    // XBox One constants
    constexpr static int JOY_AXIS_LEFT_LR = 0;       // Left stick left/right; 1.0 is left and -1.0 is right
    constexpr static int JOY_AXIS_LEFT_FB = 1;       // Left stick forward/back; 1.0 is forward and -1.0 is back
    constexpr static int JOY_AXIS_LEFT_TRIGGER = 2;      // 0 until it is pressed. 1.0 is out, -1.0 is fully depressed
    constexpr static int JOY_AXIS_RIGHT_LR = 3;      // Right stick left/right; 1.0 is left and -1.0 is right
    constexpr static int JOY_AXIS_RIGHT_FB = 4;      // Right stick forward/back; 1.0 is forward and -1.0 is back
    constexpr static int JOY_AXIS_RIGHT_TRIGGER = 5;      // 0 until it is pressed. 1.0 is out, -1.0 is fully depressed
    constexpr static int JOY_AXIS_CROSS_LEFT_RIGHT = 6;      // 1.0 is left, -1.0 ir right, 0 (-0) is no press
    constexpr static int JOY_AXIS_CROSS_UP_DOWN = 7;      // 1.0 is up, -1.0 is down, 0 is no press
    constexpr static int JOY_BUTTON_A = 0;
    constexpr static int JOY_BUTTON_B = 1;
    constexpr static int JOY_BUTTON_X = 2;
    constexpr static int JOY_BUTTON_Y = 3;
    constexpr static int JOY_BUTTON_LEFT_FRONT = 4;
    constexpr static int JOY_BUTTON_RIGHT_FRONT = 5;
    constexpr static int JOY_BUTTON_VIEW = 6;
    constexpr static int JOY_BUTTON_MENU = 7;
    constexpr static int JOY_BUTTON_POWER = 8;
    constexpr static int JOY_BUTTON_LEFT_STICK_PRESS = 9;
    constexpr static int JOY_BUTTON_RIGHT_STICK_PRESS = 10;

    // XBox One assignments
    const int joy_axis_throttle_ = JOY_AXIS_RIGHT_FB;
    const int joy_axis_strafe_ = JOY_AXIS_RIGHT_LR;
    const int joy_axis_vertical_ = JOY_AXIS_LEFT_FB;
    const int joy_axis_roll_ = JOY_AXIS_CROSS_LEFT_RIGHT;
    const int joy_axis_pitch_ = JOY_AXIS_CROSS_UP_DOWN;
    const int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;


    gazebo::physics::LinkPtr link_;
    double joy_scale_xy_{0.5};
    double joy_scale_z_{0.5};
    double joy_scale_roll_pitch_{0.5};
    double joy_scale_yaw_{0.5};

    // Connection to Gazebo message bus
    gazebo::event::ConnectionPtr update_connection_;

    // GazeboROS node
    gazebo_ros::Node::SharedPtr node_;

    // For joystick messages.
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Pass commands from the joystick callback to the OnUpdate callback.
    geometry_msgs::msg::Twist cmd_vel_{};

  public:

    // Called once when the plugin is loaded.
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
      GZ_ASSERT(model != nullptr, "Model is null");
      GZ_ASSERT(sdf != nullptr, "SDF is null");

      std::cout << std::endl;
      std::cout << "ElementalCamera PLUGIN" << std::endl;

      link_ = model->GetLink("base_link");
      GZ_ASSERT(link_ != nullptr, "Missing link");

      // Listen for the update event. This event is broadcast every simulation iteration.
      update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ElementalCameraPlugin::OnUpdate, this, _1));

      // ROS node
      node_ = gazebo_ros::Node::Get(sdf);

      // Subscribe to the joystick messages.
      joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1,
        [this](sensor_msgs::msg::Joy::UniquePtr joy_msg) -> void
        {
          joy_callback(*joy_msg);
        });

      std::cout << "-----------------------------------------" << std::endl;
      std::cout << std::endl;
    }

    // Called by the world update start event, up to 1000 times per second.
    void OnUpdate(const gazebo::common::UpdateInfo &info)
    {
      // The camera moves like it was suspended from a drone. The sticks move the drone like
      // a mode 2 controller. The cross buttons roll and pitch the camera.
      auto camera_view_vector = link_->WorldPose().Rot().RotateVector(ignition::math::Vector3d::UnitX);
      camera_view_vector.Z() = 0.;
      // If the camera is pointing up or down, then the X vector is very small and it is hard to
      // know which direction is forward. In this situation, use the z axis to determine forward and
      // backward. Leave this threshold large for testing. It should be a small number ultimately.
      if (camera_view_vector.Length() < 0.1) {
        camera_view_vector = link_->WorldPose().Rot().RotateVector(ignition::math::Vector3d::UnitZ);
        camera_view_vector.Z() = 0.;
      };
      auto camera_yaw_angle = std::atan2(camera_view_vector.Y(), camera_view_vector.X());

      auto linear_vel_f_world =
        ignition::math::Quaternion<double>{0., 0., camera_yaw_angle}.RotateVector(
          ignition::math::Vector3d{cmd_vel_.linear.x * joy_scale_xy_,
                                   cmd_vel_.linear.y * joy_scale_xy_,
                                   cmd_vel_.linear.z * joy_scale_z_});

      auto angular_vel_f_world = link_->WorldPose().Rot().RotateVector(
        ignition::math::Vector3d{
          cmd_vel_.angular.x * joy_scale_roll_pitch_,
          cmd_vel_.angular.y * joy_scale_roll_pitch_,
          0.});

      // yaw is around the world z axis. Add this in now. This will give some strange results
      // when combining multiple angular velocities, but for individual rotations it should work as expected.
      angular_vel_f_world.Z() += cmd_vel_.angular.z * joy_scale_yaw_;

      link_->SetLinearVel(linear_vel_f_world);
      link_->SetAngularVel(angular_vel_f_world);
    }

    double dead_zone(double v)
    {
      if (v > 1.0) {
        return 1.0;
      }
      if (v > 0.1) {
        return v * 0.9 + 0.1;
      }
      if (v > -0.1) {
        return 0.0;
      }
      if (v > -1.0) {
        return v * 0.9 - 0.1;
      }
      return -1.0;
    }

    void joy_callback(const sensor_msgs::msg::Joy &joy_msg)
    {
      cmd_vel_.linear.x = dead_zone(joy_msg.axes[joy_axis_throttle_]);
      cmd_vel_.linear.y = dead_zone(joy_msg.axes[joy_axis_strafe_]);
      cmd_vel_.linear.z = dead_zone(joy_msg.axes[joy_axis_vertical_]);
      cmd_vel_.angular.x = dead_zone(-joy_msg.axes[joy_axis_roll_]);
      cmd_vel_.angular.y = dead_zone(joy_msg.axes[joy_axis_pitch_]);
      cmd_vel_.angular.z = dead_zone(joy_msg.axes[joy_axis_yaw_]);
    }

  };

  GZ_REGISTER_MODEL_PLUGIN(ElementalCameraPlugin)
}

