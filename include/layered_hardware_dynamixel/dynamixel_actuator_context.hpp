#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_CONTEXT_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_CONTEXT_HPP

#include <cstdint>
#include <limits>
#include <sstream>
#include <string>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

namespace layered_hardware_dynamixel {

struct DynamixelActuatorContext {
  // handles
  const std::string name;
  const std::shared_ptr<DynamixelWorkbench> dxl_wb;
  const std::uint8_t id;

  // params
  const double torque_constant;

  // states
  double pos = std::numeric_limits<double>::quiet_NaN(),
         vel = std::numeric_limits<double>::quiet_NaN(),
         eff = std::numeric_limits<double>::quiet_NaN();

  // set by DynamixelActuatorLayer when it has already filled pos/vel/eff for this
  // cycle with a single sync read covering the whole bus. read_all_states() then
  // skips its per-item round trips. cleared at the end of the layer's read(), so
  // a sync read failure simply falls back to the per-item path.
  bool states_fresh = false;

  // commands
  double pos_cmd = std::numeric_limits<double>::quiet_NaN(),
         vel_cmd = std::numeric_limits<double>::quiet_NaN(),
         eff_cmd = std::numeric_limits<double>::quiet_NaN();
};

// utility functions

static inline std::string get_display_name(const DynamixelActuatorContext &context) {
  std::ostringstream disp_name;
  disp_name << "\"" << context.name << "\" actuator (id: " << static_cast<int>(context.id) << ")";
  return disp_name.str();
}

} // namespace layered_hardware_dynamixel

#endif