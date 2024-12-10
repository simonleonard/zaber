#ifndef ZABER_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
#define ZABER_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <zaber/motion/ascii.h>

namespace zaber_driver{

  class Axis {
    
  public:
    Axis(const std::string& name, double home, double upper_limit, double lower_limit, const zaber::motion::ascii::Axis& axis);
    Axis(const Axis&) = delete;
    Axis& operator=(const Axis&) = delete;
    
    const std::string& name() const { return name_; }

    zaber::motion::ascii::Warnings getWarnings();

    double getPosition(); 
   
    void moveAbs(double position, double velocity, double accel);
    void moveRel(double position, double velocity, double accel);

    void sendVel(double vel);
    
    void home(bool wait_until_idle = false);
    
    void stop();
    
    std::pair<double, double> getRange() const {
      return std::make_pair(lower_limit_ - home_, upper_limit_ - home_);
    }
    
    double position_;

    constexpr static double kDefaultVel = 0.0025;   /* mm / s */
    constexpr static double kDefaultAccel = 0.005;  /* mm / s^2 */
    
  private:
    bool withinRange(double position) const;
    bool busy();
    std::string name_;
    double home_;
    double lower_limit_;
    double upper_limit_;
    zaber::motion::ascii::Axis axis_;

    constexpr static zaber::motion::Units kLenUnitM = zaber::motion::Units::LENGTH_METRES;
    constexpr static zaber::motion::Units kVelUnitMPS = zaber::motion::Units::VELOCITY_METRES_PER_SECOND;
    constexpr static zaber::motion::Units kAccelUnitMPS2 = zaber::motion::Units::ACCELERATION_METRES_PER_SECOND_SQUARED;

    constexpr static zaber::motion::Units kLenUnitMM = zaber::motion::Units::LENGTH_MILLIMETRES;
    constexpr static zaber::motion::Units kVelUnitMMPS = zaber::motion::Units::VELOCITY_MILLIMETRES_PER_SECOND;
    constexpr static zaber::motion::Units kAccelUnitMMPS2 = zaber::motion::Units::ACCELERATION_MILLIMETRES_PER_SECOND_SQUARED;
    
  };

  
  class ZaberSystemHardwareInterface : public hardware_interface::SystemInterface {
  public:

    RCLCPP_SHARED_PTR_DEFINITIONS(ZaberSystemHardwareInterface)
    virtual ~ZaberSystemHardwareInterface();
    
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
								const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
								const std::vector<std::string>& stop_interfaces) override;

    void home(bool wait_until_idle);
    
  private:
    hardware_interface::HardwareInfo info_;
    std::vector<double> hw_states_position_;
    std::vector<double> hw_commands_position_;
    std::vector<double> hw_commands_velocity_;
    int cmd_mode_;

    zaber::motion::ascii::Connection connection_;
    std::vector<zaber::motion::ascii::Device> devices_;
    std::unordered_map<std::string, Axis> axes_;

    constexpr static double kTxHome = 0.0;//125;
    constexpr static double kTxLowerLimit = 0.00;
    constexpr static double kTxUpperLimit = 0.025;

    constexpr static double kLsHome = 0.0;
    constexpr static double kLsLowerLimit = 0.0;
    constexpr static double kLsUpperLimit = 0.1;
    
    constexpr static double kTzHome = 0.0;
    constexpr static double kTzLowerLimit = 0.00;
    constexpr static double kTzUpperLimit = 0.01;

  };
}

#endif
