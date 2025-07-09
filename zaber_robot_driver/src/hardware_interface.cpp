
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
  
  zaber::motion::ascii::Warnings Axis::getWarnings(){
    mutex.lock();
    zaber::motion::ascii::Warnings warnings = axis_.getWarnings();
    mutex.unlock();
    return warnings;
  }

  double Axis::getPosition(){
    mutex.lock();
    double p = axis_.getPosition(kLenUnitM) - home_;
    mutex.unlock();
    return p;
  }
  
  void Axis::moveAbs(double position, double velocity, double accel) {
    if(!busy() && withinRange(position)){
      mutex.lock();
      axis_.moveAbsolute((position + home_), kLenUnitM, false, velocity, kVelUnitMPS, accel, kAccelUnitMPS2);
      mutex.unlock();
    }
  }
  void Axis::moveRel(double distance, double velocity, double /*accel*/) {
    if(!busy() && withinRange(distance + getPosition())){
      mutex.lock();
      axis_.moveRelative(distance, kLenUnitM, false, velocity, kVelUnitMPS);
      mutex.unlock();
    }
  }

  void Axis::sendVel(double vel){
    mutex.lock();
    axis_.moveVelocity(vel, kVelUnitMMPS, 10, kAccelUnitMPS2);
    mutex.unlock();
  }

  void Axis::home(bool wait_until_idle) {
    mutex.lock();
    axis_.moveAbsolute(home_, kLenUnitM, wait_until_idle, kDefaultVel, kVelUnitMPS, kDefaultAccel, kAccelUnitMPS2);
    mutex.unlock();
    //axis_.home(wait_until_idle);
  }
  
  void Axis::stop(){
    mutex.lock();
    axis_.stop(false);
    mutex.unlock();
  }

  bool Axis::withinRange(double position) const {
    if (lower_limit_ <= position + home_ && position + home_ <= upper_limit_)
      return true;
    else{
      RCLCPP_INFO(rclcpp::get_logger("Axis"),
		  "%s position '%f' is out of range: ['%f', '%f']",
		  name(),
		  position,
		  lower_limit_ - home_,
		  upper_limit_ - home_);
      return false;
    }
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
    
    cmd_mode_ = 0;
    
    if( hardware_interface::SystemInterface::on_init(info) !=
	hardware_interface::CallbackReturn::SUCCESS ){
      return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    std::string com_port = info_.hardware_parameters["com_port"];

    
    try { connection_ = zaber::motion::ascii::Connection::openSerialPort(com_port); }
    catch (const std::exception& exc) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("ZaberSystemHardwareInterface"), " Zaber connection failed on port " << com_port );
      return hardware_interface::CallbackReturn::ERROR;
    }

    devices_ = connection_.detectDevices();
    RCLCPP_INFO_STREAM( rclcpp::get_logger("ZaberSystemHardwareInterface"), " " << devices_.size() << " devices." );
    if(devices_.size() != 3){
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("ZaberSystemHardwareInterface"), "Expected 3 devices, got " << devices_.size() );
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_position_.resize(info_.joints.size(), 0.0);
    hw_commands_velocity_.resize(info_.joints.size(), 0.0);

    for( const hardware_interface::ComponentInfo& joint : info_.joints ){

      double minpos=0.0, maxpos=0.0;

      for( const hardware_interface::InterfaceInfo& cmd_interface : joint.command_interfaces ){
	if( cmd_interface.name == "position" ){
	  minpos = std::stod(cmd_interface.min);
	  maxpos = std::stod(cmd_interface.max);
	}
      }
	
      if( joint.name == "insertion_joint" ){
	auto ret = axes_.emplace(std::piecewise_construct,
				 std::forward_as_tuple("insertion_joint"),
				 std::forward_as_tuple("insertion_joint", kLsHome, minpos, maxpos, devices_[0].getAxis(1)) );
	if( ret.second ){
	  RCLCPP_INFO_STREAM(rclcpp::get_logger("ZaberSystemHardwareInterface"), "Configured insertion axis");
	}
      }
      else if( joint.name == "horizontal_joint" ){
	auto ret = axes_.emplace(std::piecewise_construct,
				 std::forward_as_tuple("horizontal_joint"),
				 std::forward_as_tuple("horizontal_joint", kTxHome, minpos, maxpos, devices_[1].getAxis(1)) );
	if( ret.second ){
	  RCLCPP_INFO_STREAM(rclcpp::get_logger("ZaberSystemHardwareInterface"), "Configured horizontal axis");
	}
      }
      else if( joint.name == "vertical_joint" ){
	auto ret = axes_.emplace(std::piecewise_construct,
				 std::forward_as_tuple("vertical_joint"),
				 std::forward_as_tuple("vertical_joint", kTzHome, minpos, maxpos, devices_[2].getAxis(1)) );
	if( ret.second ){
	  RCLCPP_INFO_STREAM(rclcpp::get_logger("ZaberSystemHardwareInterface"), "Configured vertical axis");
	}
      }
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
  ZaberSystemHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){
    for (auto& pair : axes_)
      { pair.second.stop(); }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  hardware_interface::CallbackReturn
  ZaberSystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
    for (auto& pair : axes_)
      { pair.second.stop(); }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  ZaberSystemHardwareInterface::on_error(const rclcpp_lifecycle::State& /*previous_state*/){
    for (auto& pair : axes_)
      { pair.second.stop(); }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type
  ZaberSystemHardwareInterface::prepare_command_mode_switch
  (const std::vector<std::string>& start_interfaces,
   const std::vector<std::string>& stop_interfaces){
    
    RCLCPP_INFO_STREAM( rclcpp::get_logger("ZaberSystemHardwareInterface"), "Preparing to switch." );
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;
    for (const auto& key : start_interfaces){
      RCLCPP_INFO_STREAM( rclcpp::get_logger("ZaberSystemHardwareInterface"), "Starting " << key );
      for (auto i = 0u; i < info_.joints.size(); i++) {
	if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
	  cmd_mode_ = 1;
	  hw_commands_velocity_[i] = 0.0;
	}
	if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
	  cmd_mode_ = 2;
	  hw_commands_velocity_[i] = 0.0;
	}
      }
    }

    for (const auto& key : stop_interfaces){
      RCLCPP_INFO_STREAM( rclcpp::get_logger("ZaberSystemHardwareInterface"), "Stopping " << key );
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
    
    RCLCPP_INFO_STREAM( rclcpp::get_logger("ZaberSystemHardwareInterface"), "Switching mode." );
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;
    for (const auto& key : start_interfaces){
      RCLCPP_INFO_STREAM( rclcpp::get_logger("ZaberSystemHardwareInterface"), "Starting " << key );
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
      RCLCPP_INFO_STREAM( rclcpp::get_logger("ZaberSystemHardwareInterface"), "Stopping " << key );
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
      try{
	// All axis are flipped except Y (insertion)
	hw_states_position_[i] = axes_.at(info_.joints[i].name).getPosition();
	/*
	zaber::motion::ascii::Warnings warnings = axes_.at(info_.joints[i].name).getWarnings();
	for( auto w=warnings.getFlags().begin(); w!=warnings.getFlags().end(); w++ )
	  std::cout << *w << std::endl;
	*/
      }catch(const std::out_of_range& exception){
	RCLCPP_ERROR_STREAM( rclcpp::get_logger("ZaberSystemHardwareInterface"), "Failed to access axis " << info_.joints[i].name );
      }
      
    }

    return hardware_interface::return_type::OK;
  }
  
  hardware_interface::return_type ZaberSystemHardwareInterface::write(const rclcpp::Time& /*time*/,
								      const rclcpp::Duration& /*period*/){
    for( std::size_t i=0; i<info_.joints.size(); i++ ){
      try{
	if( cmd_mode_ == 1 ){
	  axes_.at(info_.joints[i].name).moveAbs(hw_commands_position_[i], Axis::kDefaultVel, Axis::kDefaultAccel );
	}
	if( cmd_mode_ == 2 ){
	  axes_.at(info_.joints[i].name).sendVel(hw_commands_velocity_[i]);
	}
      }
      catch(const std::out_of_range& exception){
	RCLCPP_ERROR_STREAM( rclcpp::get_logger("ZaberSystemHardwareInterface"), "Failed to access axis " << info_.joints[i].name );
      }
    }
    //std::cout << std::endl;
    
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
