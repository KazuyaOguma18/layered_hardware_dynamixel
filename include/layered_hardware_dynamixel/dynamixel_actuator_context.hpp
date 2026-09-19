#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_CONTEXT_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_CONTEXT_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

namespace layered_hardware_dynamixel {

// a control table write queued by an operating mode, to be sent by DynamixelActuatorLayer
// together with those of the other actuators on the bus. DynamixelWorkbench::writeRegister()
// sleeps 10 ms after every register write (measured: a write costs 11.0 ms against 1.0 ms for
// the identical round trip of a read), which a sync write skips entirely.
struct PendingWrite {
  const char *item = nullptr; // a string literal, kept for the blocking fallback path
  std::uint16_t address = 0, length = 0;
  std::int32_t value = 0;
};

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
  // the most writes an operating mode queues in one cycle (current-based position mode
  // writes profile velocity, effort limit and goal position)
  static constexpr std::size_t max_pending_writes = 3;
  std::array<PendingWrite, max_pending_writes> pending_writes{};
  std::size_t num_pending_writes = 0;
  // set by DynamixelActuatorLayer only while it is able to flush the queue. writes made
  // outside its write(), such as enabling torque on a mode switch, go out immediately.
  bool defer_writes = false;

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