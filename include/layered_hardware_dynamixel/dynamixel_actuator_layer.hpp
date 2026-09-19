#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_LAYER_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_LAYER_HPP

#include <memory>
#include <string>
#include <utility> // for std::move()
#include <vector>

#include <controller_interface/controller_interface_base.hpp> // for ci::InterfaceConfiguration
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <hardware_interface/handle.hpp> // for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp> // for hi::return_type
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/merge_utils.hpp>
#include <layered_hardware/string_registry.hpp>
#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_driver.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/logging_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware_dynamixel {

class DynamixelActuatorLayer : public lh::LayerInterface {
public:
  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo &hardware_info) override {
    // initialize the base class first
    const CallbackReturn is_base_initialized =
        lh::LayerInterface::on_init(layer_name, hardware_info);
    if (is_base_initialized != CallbackReturn::SUCCESS) {
      return is_base_initialized;
    }

    // find parameter group for this layer
    const auto params_it = hardware_info.hardware_parameters.find(layer_name);
    if (params_it == hardware_info.hardware_parameters.end()) {
      lhd_error("DynamixelActuatorLayer::on_init(): \"%s\" parameter is missing", layer_name);
      return CallbackReturn::ERROR;
    }

    // parse parameters for this layer as yaml
    std::string serial_iface;
    std::uint32_t baudrate;
    std::vector<std::string> ator_names;
    std::vector<YAML::Node> ator_params;
    try {
      const YAML::Node params = YAML::Load(params_it->second);
      serial_iface = params["serial_interface"].as<std::string>("/dev/ttyUSB0");
      baudrate = params["baudrate"].as<int>(115200);
      for (const auto &name_param_pair : params["actuators"]) {
        ator_names.emplace_back(name_param_pair.first.as<std::string>());
        ator_params.emplace_back(name_param_pair.second);
      }
    } catch (const YAML::Exception &error) {
      lhd_error("DynamixelActuatorLayer::on_init(): %s (on parsing \"%s\" parameter)", //
                error, layer_name);
      return CallbackReturn::ERROR;
    }

    // open USB serial device
    const auto dxl_wb = std::make_shared<DynamixelWorkbench>();
    if (!dxl_wb->init(serial_iface.c_str(), baudrate)) {
      lhd_error("DynamixelActuatorLayer::on_init(): Failed to open DynamielWorkbench (%s, %d)",
                serial_iface, baudrate);
      return CallbackReturn::ERROR;
    }

    // init actuators with param "actuators/<actuator_name>"
    // (on_init() may run again on a re-initialized hardware component, and appending to the
    // previous contents would give the sync read duplicate ids)
    drivers_.clear();
    contexts_.clear();
    for (std::size_t i = 0; i < ator_names.size(); ++i) {
      try {
        drivers_.emplace_back(new DynamixelActuatorDriver(ator_names[i], ator_params[i], dxl_wb));
      } catch (const std::runtime_error &error) {
        lhd_error("DynamixelActuatorLayer::on_init(): Failed to create driver for \"%s\" actuator",
                  ator_names[i]);
        return CallbackReturn::ERROR;
      }
      contexts_.push_back(drivers_.back()->context());
      lhd_info("DynamixelActuatorLayer::on_init(): Initialized the actuator \"%s\"", ator_names[i]);
    }

    // prepare to read the states of all the actuators on this bus in one round trip.
    // an invalid layout or a negative index just leaves read() on the per-actuator path.
    sync_layout_ = make_sync_state_layout(contexts_);
    sync_handler_index_ = contexts_.empty() ? -1 : add_sync_state_handler(contexts_.front(), sync_layout_);
    sync_read_failures_ = 0;
    sync_buffers_.resize(contexts_.size());
    sync_write_handlers_.ids.reserve(contexts_.size());
    sync_write_handlers_.values.reserve(contexts_.size());
    if (sync_handler_index_ >= 0) {
      lhd_info("DynamixelActuatorLayer::on_init(): Will read the states of %d actuators with a "
               "single sync read of %d bytes from address %d",
               static_cast<int>(contexts_.size()), sync_layout_.block_length,
               sync_layout_.block_address);
    }

    return CallbackReturn::SUCCESS;
  }

  virtual std::vector<hi::StateInterface> export_state_interfaces() override {
    // export reference to actuator states owned by this layer
    std::vector<hi::StateInterface> ifaces;
    for (const auto &driver : drivers_) {
      ifaces = lh::merge(std::move(ifaces), driver->export_state_interfaces());
    }
    return ifaces;
  }

  virtual std::vector<hi::CommandInterface> export_command_interfaces() override {
    // export reference to actuator commands owned by this layer
    std::vector<hi::CommandInterface> ifaces;
    for (const auto &driver : drivers_) {
      ifaces = lh::merge(std::move(ifaces), driver->export_command_interfaces());
    }
    return ifaces;
  }

  virtual ci::InterfaceConfiguration state_interface_configuration() const override {
    // any state interfaces required from other layers because this layer is "source"
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual ci::InterfaceConfiguration command_interface_configuration() const override {
    // any command interfaces required from other layers because this layer is "source"
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual void
  assign_interfaces(std::vector<hi::LoanedStateInterface> && /*state_interfaces*/,
                    std::vector<hi::LoanedCommandInterface> && /*command_interfaces*/) override {
    // any interfaces has to be imported from other layers because this layer is "source"
  }

  virtual hi::return_type
  prepare_command_mode_switch(const lh::StringRegistry &active_interfaces) override {
    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->prepare_command_mode_switch(active_interfaces));
    }
    return result;
  }

  virtual hi::return_type
  perform_command_mode_switch(const lh::StringRegistry &active_interfaces) override {
    // notify controller switching to all actuators
    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->perform_command_mode_switch(active_interfaces));
    }
    return result;
  }

  virtual hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // read from all actuators
    // fill every actuator's states with one sync read if possible. on success the drivers
    // below just convert what is already in their contexts; on failure they fall back to
    // reading the items one by one, which also surfaces which actuator is unresponsive.
    if (sync_handler_index_ >= 0) {
      if (sync_read_states(contexts_, sync_layout_, sync_handler_index_, &sync_buffers_)) {
        sync_read_failures_ = 0;
      } else if (++sync_read_failures_ >= max_consecutive_sync_read_failures) {
        // a sync read that keeps failing costs its timeout on top of the per-item fallback,
        // which is slower than never having tried. re-initializing the hardware component
        // (the documented recovery path) sets this up again.
        lhd_warn("DynamixelActuatorLayer::read(): Sync read failed %d times in a row. "
                 "Reading states actuator by actuator from now on.",
                 sync_read_failures_);
        sync_handler_index_ = -1;
      }
    }

    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->read(time, period));
    }

    // the sync read result is only valid for this cycle
    for (const auto &context : contexts_) {
      context->states_fresh = false;
    }
    return result;
  }

  virtual hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // let the operating modes queue their control table writes rather than perform them, so
    // that the whole bus is written with one sync write per item instead of one blocking
    // round trip per actuator (11.0 ms each, of which 10 ms is a sleep inside
    // DynamixelWorkbench::writeRegister())
    for (const auto &context : contexts_) {
      context->defer_writes = true;
    }

    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->write(time, period));
    }

    if (!sync_write_pending(contexts_, &sync_write_handlers_)) {
      result = lh::merge(result, hi::return_type::ERROR);
    }
    for (const auto &context : contexts_) {
      context->defer_writes = false;
    }
    return result;
  }

private:
  std::vector<std::unique_ptr<DynamixelActuatorDriver>> drivers_;

  // the contexts of drivers_, in the same order, so that the whole bus can be read at once
  std::vector<std::shared_ptr<DynamixelActuatorContext>> contexts_;
  SyncStateLayout sync_layout_;
  int sync_handler_index_ = -1;
  SyncStateBuffers sync_buffers_;
  SyncWriteHandlers sync_write_handlers_;
  int sync_read_failures_ = 0;
  static constexpr int max_consecutive_sync_read_failures = 10;
};
} // namespace layered_hardware_dynamixel

#endif