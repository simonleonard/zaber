
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "zaber_robot_driver/hardware_interface.hpp"

namespace zaber_driver {

  Axis::Axis(const std::string& name, double home, double lower_limit, double upper_limit, const zaber::motion::ascii::Axis& axis)
    : name_(name),
      home_(home),
      lower_limit_(lower_limit),
      upper_limit_(upper_limit),
      axis_(axis){
    RCLCPP_INFO(rclcpp::get_logger("Axis"), " %s: [ '%f', '%f' ]", name.c_str(), lower_limit_ - home_, upper_limit_ - home_ );
  }
  
  double Axis::getPosition(){
    zaber::motion::ascii::Warnings warning = axis_.getWarnings();
    //for(auto w : warning.getFlags())
    //std::cout << w << std::endl;
    return axis_.getPosition(kLenUnitMM) - home_;
  }
  
  void Axis::moveAbs(double position, double velocity, double accel) {
    if (!busy() && withinRange(position))                                                                     
      axis_.moveAbsolute((position + home_), kLenUnitMM, false, velocity, kVelUnitMMPS, accel, kAccelUnitMMPS2);
  }
  void Axis::moveRel(double distance, double velocity, double accel) {
    if (!busy() && withinRange(distance + getPosition()))
      axis_.moveRelative(distance, kLenUnitMM, false, velocity, kVelUnitMMPS);
  }

  void Axis::sendVel(double vel){
    axis_.moveVelocity(vel, kVelUnitMMPS, 10, kAccelUnitMMPS2);
  }

  void Axis::home(bool wait_until_idle) {
    //axis_.moveAbsolute(home_, kLenUnitMM, wait_until_idle, kDefaultVel, kVelUnitMMPS, kDefaultAccel, kAccelUnitMMPS2);
    axis_.home(wait_until_idle);
  }
  
  void Axis::stop() {
    axis_.stop(false);
  }

  bool Axis::withinRange(double position) const {
    if (lower_limit_ <= position + home_ && position + home_ <= upper_limit_)
      return true;
    RCLCPP_INFO(rclcpp::get_logger("Axis"), "position '%f' is out of range: ['%f', '%f']",
		position, lower_limit_ - home_, upper_limit_ - home_);
    return false;
  }
  
  bool Axis::busy() {
    if(axis_.isBusy())
      { return true; }
    return false;
  }
  
  ZaberSystemHardwareInterface::~ZaberSystemHardwareInterface()
  { on_deactivate(rclcpp_lifecycle::State()); }
  
  hardware_interface::CallbackReturn
  ZaberSystemHardwareInterface::on_init
  (const hardware_interface::HardwareInfo& info){
    
    if( hardware_interface::SystemInterface::on_init(info) !=
	hardware_interface::CallbackReturn::SUCCESS ){
      return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    
    try { connection_ = zaber::motion::ascii::Connection::openSerialPort("/dev/ttyUSB0"); }
    catch (const std::exception& exc) {
      RCLCPP_ERROR(rclcpp::get_logger("ZaberSystemHardwareInterface"), " zaber connection failed.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    devices_ = connection_.detectDevices();                                                                 
    RCLCPP_ERROR(rclcpp::get_logger("ZaberSystemHardwareInterface"), " %lu ", devices_.size(), " devices.");
    if (devices_.size() != 3) return hardware_interface::CallbackReturn::ERROR;
    
    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocity_.resize(info_.joints.size(), 0.0);

    for (const hardware_interface::ComponentInfo & joint : info_.joints){
      
      if( joint.name == "insertion_joint" )
	axes_.emplace(std::piecewise_construct,
		      std::forward_as_tuple("insertion_joint"),
		      std::forward_as_tuple("insertion_joint", kLsHome, kLsLowerLimit, kLsUpperLimit, devices_[0].getAxis(1)) );
      else if( joint.name == "horizontal_joint" )
	axes_.emplace(std::piecewise_construct,
		      std::forward_as_tuple("horizontal_joint"),
		      std::forward_as_tuple("horizontal_joint", kTxHome, kTxLowerLimit, kTxUpperLimit, devices_[1].getAxis(1)) );
      else if( joint.name == "vertical_joint" )
	axes_.emplace(std::piecewise_construct,
		      std::forward_as_tuple("vertical_joint"),
		      std::forward_as_tuple("vertical_joint", kTzHome, kTzLowerLimit, kTzUpperLimit, devices_[2].getAxis(1)) );
      else{
	RCLCPP_ERROR(rclcpp::get_logger("ZaberSystemHardwareInterface"), " unsupported joint name.");
	return hardware_interface::CallbackReturn::ERROR;
      }
      
      for ( std::size_t i=0; i<joint.command_interfaces.size(); i++ ){
	if( joint.command_interfaces[i].name == hardware_interface::HW_IF_POSITION )
	  RCLCPP_INFO(rclcpp::get_logger("ZaberSystemHardwareInterface"), " %s has position command interface.", joint.name.c_str());
	if( joint.command_interfaces[i].name == hardware_interface::HW_IF_VELOCITY )
	  RCLCPP_INFO(rclcpp::get_logger("ZaberSystemHardwareInterface"), " %s has velocity command interface.", joint.name.c_str());
      }
      
      for ( std::size_t i=0; i<joint.state_interfaces.size(); i++ ){
	if( joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION )
	  RCLCPP_INFO(rclcpp::get_logger("ZaberSystemHardwareInterface"), " %s has position state interface.", joint.name.c_str());
      }
      
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
    
  }

  hardware_interface::CallbackReturn
  ZaberSystemHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
    RCLCPP_INFO(rclcpp::get_logger("ZaberSystemHardwareInterface"), "Configuring ...please wait while homing...");

    home(true);
    
    RCLCPP_INFO(rclcpp::get_logger("ZaberSystemHardwareInterface"), "Successfully configured.");
    
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> ZaberSystemHardwareInterface::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++){
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
								       hardware_interface::HW_IF_POSITION,
								       &hw_states_position_[i]) );
    }
    return state_interfaces;
  }
  
  std::vector<hardware_interface::CommandInterface> ZaberSystemHardwareInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
									   hardware_interface::HW_IF_POSITION,
									   &hw_commands_position_[i]) );
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
									   hardware_interface::HW_IF_VELOCITY,
									   &hw_commands_velocity_[i]) );
    }
    return command_interfaces;
  }
  
  hardware_interface::CallbackReturn
  ZaberSystemHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state){
    for (auto& pair : axes_)
      { pair.second.stop(); }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  hardware_interface::CallbackReturn
  ZaberSystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state){
    for (auto& pair : axes_)
      { pair.second.stop(); }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  ZaberSystemHardwareInterface::on_error(const rclcpp_lifecycle::State& previous_state){
    for (auto& pair : axes_)
      { pair.second.stop(); }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type
  ZaberSystemHardwareInterface::prepare_command_mode_switch
  (const std::vector<std::string>& start_interfaces,
   const std::vector<std::string>& stop_interfaces){
    
    std::cout << "ZaberSystemHardwareInterface::prepare_command_mode_switch" << std::endl;
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;
    for (const auto& key : start_interfaces){
      std::cout << "start: " << key << std::endl;
      for (auto i = 0u; i < info_.joints.size(); i++) {
	if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
	  hw_commands_velocity_[i] = 0.0;
	}
	if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
	  hw_commands_velocity_[i] = 0.0;
	}
      }
    }

    for (const auto& key : stop_interfaces){
      std::cout << "stop: " << key << std::endl;
      for (auto i = 0u; i < info_.joints.size(); i++) {
	if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
	  hw_commands_velocity_[i] = 0.0;
	}
	if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
	  hw_commands_velocity_[i] = 0.0;	  
	}
      }
    }
    return ret_val;
  }
  
  hardware_interface::return_type
  ZaberSystemHardwareInterface::perform_command_mode_switch
  (const std::vector<std::string>& start_interfaces,
   const std::vector<std::string>& stop_interfaces){

    std::cout << "ZaberSystemHardwareInterface::perform_command_mode_switch" << std::endl;
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;
    for (const auto& key : start_interfaces){
      std::cout << "start: " << key << std::endl;
      for (auto i = 0u; i < info_.joints.size(); i++) {
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){	  
	  for (auto& pair : axes_) pair.second.stop();
	}
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
	  hw_commands_velocity_[i] = 0.0;
	}
      }
    }
    
    for (const auto& key : stop_interfaces){
      std::cout << "stop: " << key << std::endl;
      for (auto i = 0u; i < info_.joints.size(); i++) {
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
	  for (auto& pair : axes_) pair.second.stop();
	}
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
	  hw_commands_velocity_[i] = 0.0;
	}
      }
    }
    
    return ret_val;
  }
  
  hardware_interface::return_type ZaberSystemHardwareInterface::read( const rclcpp::Time& /*time*/,
								      const rclcpp::Duration& /*period*/){
    for(std::size_t i=0; i<info_.joints.size(); i++ ){
      hw_states_position_[i] = axes_.at(info_.joints[i].name).getPosition();
    }
    return hardware_interface::return_type::OK;
  }
  
  hardware_interface::return_type ZaberSystemHardwareInterface::write(const rclcpp::Time& /*time*/,
								      const rclcpp::Duration& /*period*/){
    for( std::size_t i=0; i<info_.joints.size(); i++ ){
      axes_.at(info_.joints[i].name).sendVel(hw_commands_velocity_[i]);
    }
    return hardware_interface::return_type::OK;
  }

  void ZaberSystemHardwareInterface::home(bool wait_until_idle) {
    for (auto& pair : axes_){
      pair.second.home(wait_until_idle);
    }
  }
  
}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(zaber_driver::ZaberSystemHardwareInterface, hardware_interface::SystemInterface)
