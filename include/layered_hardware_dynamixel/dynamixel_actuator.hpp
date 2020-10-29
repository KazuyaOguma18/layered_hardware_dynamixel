#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_HPP

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface_extensions/integer_interface.hpp>
#include <layered_hardware_dynamixel/clear_multi_turn_mode.hpp>
#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/current_based_position_mode.hpp>
#include <layered_hardware_dynamixel/current_mode.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <layered_hardware_dynamixel/extended_position_mode.hpp>
#include <layered_hardware_dynamixel/operating_mode_base.hpp>
#include <layered_hardware_dynamixel/reboot_mode.hpp>
#include <layered_hardware_dynamixel/torque_disable_mode.hpp>
#include <layered_hardware_dynamixel/velocity_mode.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace layered_hardware_dynamixel {

class DynamixelActuator {
public:
  DynamixelActuator() {}

  virtual ~DynamixelActuator() {
    // finalize the present mode
    if (present_mode_) {
      ROS_INFO_STREAM("DynamixelActuator::~DynamixelActuator(): Stopping operating mode '"
                      << present_mode_->getName() << "' for actuator '" << data_->name
                      << "' (id: " << static_cast< int >(data_->id) << ")");
      present_mode_->stopping();
      present_mode_ = OperatingModePtr();
    }
  }

  bool init(const std::string &name, DynamixelWorkbench *const dxl_wb, hi::RobotHW *const hw,
            const ros::NodeHandle &param_nh) {
    // dynamixel id from param
    int id;
    if (!param_nh.getParam("id", id)) {
      ROS_ERROR_STREAM("DynamixelActuator::init(): Failed to get param '"
                       << param_nh.resolveName("id") << "'");
      return false;
    }

    // find dynamixel actuator by id
    std::uint16_t model_number;
    if (!dxl_wb->ping(id, &model_number)) {
      ROS_ERROR_STREAM("DynamixelActuator::init(): Failed to ping the actuator '"
                       << name << "' (id: " << static_cast< int >(id) << ")");
      return false;
    }

    // torque constant from param
    double torque_constant;
    if (!param_nh.getParam("torque_constant", torque_constant)) {
      ROS_ERROR_STREAM("DynamixelActuator::init(): Failed to get param '"
                       << param_nh.resolveName("torque_constant") << "'");
      return false;
    }

    // names of additinal states & commands from param (optional)
    const std::vector< std::string > additional_state_names(
        param_nh.param("additional_states", std::vector< std::string >()));
    const std::vector< std::string > additional_cmd_names(
        param_nh.param("additional_commands", std::vector< std::string >()));

    // allocate data structure
    data_.reset(new DynamixelActuatorData(name, dxl_wb, id, torque_constant, additional_state_names,
                                          additional_cmd_names));

    // register actuator states & commands to corresponding hardware interfaces
    const hi::ActuatorStateHandle state_handle(data_->name, &data_->pos, &data_->vel, &data_->eff);
    if (!registerActuatorTo< hi::ActuatorStateInterface >(hw, state_handle) ||
        !registerActuatorTo< hi::PositionActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->pos_cmd)) ||
        !registerActuatorTo< hi::VelocityActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->vel_cmd)) ||
        !registerActuatorTo< hi::EffortActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->eff_cmd))) {
      return false;
    }

    // register additional states & commands to corresponding hardware interfaces
    for (std::map< std::string, std::int32_t >::value_type &state : data_->additional_states) {
      if (!registerActuatorTo< hie::Int32StateInterface >(
              hw, hie::Int32StateHandle(data_->name + "/" + state.first, &state.second))) {
        return false;
      }
    }
    for (std::map< std::string, std::int32_t >::value_type &cmd : data_->additional_cmds) {
      if (!registerActuatorTo< hie::Int32Interface >(
              hw, hie::Int32Handle(data_->name + "/" + cmd.first, &cmd.second, &cmd.second))) {
        return false;
      }
    }

    // make operating mode map from ros-controller name to dynamixel's operating mode
    std::map< std::string, std::string > mode_name_map;
    if (!param_nh.getParam("operating_mode_map", mode_name_map)) {
      ROS_ERROR_STREAM("DynamixelActuator::init(): Failed to get param '"
                       << param_nh.resolveName("operating_mode_map") << "'");
      return false;
    }
    for (const std::map< std::string, std::string >::value_type &mode_name : mode_name_map) {
      std::map< std::string, int > item_map;
      param_nh.getParam(ros::names::append("item_map", mode_name.second), item_map);
      const OperatingModePtr mode(makeOperatingMode(mode_name.second, item_map));
      if (!mode) {
        ROS_ERROR_STREAM("DynamixelActuator::init(): Failed to make operating mode '"
                         << mode_name.second << "' for the actuator '" << data_->name
                         << "' (id: " << static_cast< int >(data_->id) << ")");
        return false;
      }
      mode_map_[mode_name.first] = mode;
    }

    return true;
  }

  bool prepareSwitch(const std::list< hi::ControllerInfo > &starting_controller_list,
                     const std::list< hi::ControllerInfo > &stopping_controller_list) {
    // check if switching is possible by counting number of operating modes after switching

    // number of modes before switching
    std::size_t n_modes(present_mode_ ? 1 : 0);

    // number of modes after stopping controllers
    if (n_modes != 0) {
      for (const hi::ControllerInfo &stopping_controller : stopping_controller_list) {
        const std::map< std::string, OperatingModePtr >::const_iterator mode_to_stop(
            mode_map_.find(stopping_controller.name));
        if (mode_to_stop != mode_map_.end() && mode_to_stop->second == present_mode_) {
          n_modes = 0;
          break;
        }
      }
    }

    // number of modes after starting controllers
    for (const hi::ControllerInfo &starting_controller : starting_controller_list) {
      const std::map< std::string, OperatingModePtr >::const_iterator mode_to_start(
          mode_map_.find(starting_controller.name));
      if (mode_to_start != mode_map_.end() && mode_to_start->second) {
        ++n_modes;
      }
    }

    // assert 0 or 1 operating modes. multiple modes are impossible.
    if (n_modes != 0 && n_modes != 1) {
      ROS_ERROR_STREAM("DynamixelActuator::prepareSwitch(): Rejected unfeasible controller "
                       "switching for the actuator '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id) << ")");
      return false;
    }

    return true;
  }

  void doSwitch(const std::list< hi::ControllerInfo > &starting_controller_list,
                const std::list< hi::ControllerInfo > &stopping_controller_list) {
    // stop actuator's operating mode according to stopping controller list
    if (present_mode_) {
      for (const hi::ControllerInfo &stopping_controller : stopping_controller_list) {
        const std::map< std::string, OperatingModePtr >::const_iterator mode_to_stop(
            mode_map_.find(stopping_controller.name));
        if (mode_to_stop != mode_map_.end() && mode_to_stop->second == present_mode_) {
          ROS_INFO_STREAM("DynamixelActuator::doSwitch(): Stopping operating mode '"
                          << present_mode_->getName() << "' for the actuator '" << data_->name
                          << "' (id: " << static_cast< int >(data_->id) << ")");
          present_mode_->stopping();
          present_mode_ = OperatingModePtr();
          break;
        }
      }
    }

    // start actuator's operating modes according to starting controllers
    if (!present_mode_) {
      for (const hi::ControllerInfo &starting_controller : starting_controller_list) {
        const std::map< std::string, OperatingModePtr >::const_iterator mode_to_start(
            mode_map_.find(starting_controller.name));
        if (mode_to_start != mode_map_.end() && mode_to_start->second) {
          ROS_INFO_STREAM("DynamixelActuator::doSwitch(): Starting operating mode '"
                          << mode_to_start->second->getName() << "' for the actuator '"
                          << data_->name << "' (id: " << static_cast< int >(data_->id) << ")");
          present_mode_ = mode_to_start->second;
          present_mode_->starting();
          break;
        }
      }
    }
  }

  void read(const ros::Time &time, const ros::Duration &period) {
    if (present_mode_) {
      present_mode_->read(time, period);
    }
  }

  void write(const ros::Time &time, const ros::Duration &period) {
    if (present_mode_) {
      present_mode_->write(time, period);
    }
  }

private:
  template < typename Interface, typename Handle >
  static bool registerActuatorTo(hi::RobotHW *const hw, const Handle &handle) {
    Interface *const iface(hw->get< Interface >());
    if (!iface) {
      ROS_ERROR("DynamixelActuator::registerActuatorTo(): Failed to get a hardware interface");
      return false;
    }
    iface->registerHandle(handle);
    return true;
  }

  OperatingModePtr makeOperatingMode(const std::string &mode_str,
                                     const std::map< std::string, int > &item_map) {
    if (mode_str == "clear_multi_turn") {
      return std::make_shared< ClearMultiTurnMode >(data_);
    } else if (mode_str == "current") {
      return std::make_shared< CurrentMode >(data_, item_map);
    } else if (mode_str == "current_based_position") {
      return std::make_shared< CurrentBasedPositionMode >(data_, item_map);
    } else if (mode_str == "extended_position") {
      return std::make_shared< ExtendedPositionMode >(data_, item_map);
    } else if (mode_str == "reboot") {
      return std::make_shared< RebootMode >(data_);
    } else if (mode_str == "torque_disable") {
      return std::make_shared< TorqueDisableMode >(data_);
    } else if (mode_str == "velocity") {
      return std::make_shared< VelocityMode >(data_, item_map);
    }
    ROS_ERROR_STREAM("DynamixelActuator::makeOperatingMode(): Unknown operating mode name '"
                     << mode_str << " for the actuator '" << data_->name
                     << "' (id: " << static_cast< int >(data_->id) << ")");
    return OperatingModePtr();
  }

private:
  DynamixelActuatorDataPtr data_;

  std::map< std::string, OperatingModePtr > mode_map_;
  OperatingModePtr present_mode_;
};

typedef std::shared_ptr< DynamixelActuator > DynamixelActuatorPtr;
typedef std::shared_ptr< const DynamixelActuator > DynamixelActuatorConstPtr;
} // namespace layered_hardware_dynamixel

#endif