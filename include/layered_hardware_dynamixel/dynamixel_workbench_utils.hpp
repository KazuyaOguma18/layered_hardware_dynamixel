#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_WORKBENCH_UTILS_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_WORKBENCH_UTILS_HPP

#include <algorithm> // for std::min(), std::max()
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/logging_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

// adapter functions to wrap DynamixelWorkbench and provide a more user-friendly interface
//   - display error messages
//   - argument type conversion
//   - workarounds for bugs

// instruction functions

static inline bool ping(const std::shared_ptr<DynamixelActuatorContext> &context) {
  const char *log = nullptr;
  if (!context->dxl_wb->ping(context->id, &log)) {
    lhd_error("ping(): Failed to ping to %s: %s", //
              get_display_name(*context), (log ? log : "No log from DynamixelWorkbench::ping()"));
    return false;
  }
  return true;
}

static inline bool ping_for(const std::shared_ptr<DynamixelActuatorContext> &context,
                            const rclcpp::Duration &timeout) {
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  const rclcpp::Time timeout_abs = clock.now() + timeout;
  while (true) {
    if (clock.now() > timeout_abs) {
      lhd_error("ping_for(): No ping response from %s for %f s", //
                get_display_name(*context), timeout.seconds());
      return false;
    }
    if (ping(context)) {
      return true;
    }
  }
  // never reach here
}

static inline bool reboot(const std::shared_ptr<DynamixelActuatorContext> &context) {
  const char *log = nullptr;
  if (!context->dxl_wb->reboot(context->id, &log)) {
    lhd_error("reboot(): Failed to reboot %s: %s", //
              get_display_name(*context), (log ? log : "No log from DynamixelWorkbench::reboot()"));
    return false;
  }
  return true;
}

// read functions

static inline bool has_item(const std::shared_ptr<DynamixelActuatorContext> &context,
                            const std::string &item) {
  const char *log = nullptr;
  return context->dxl_wb->getItemInfo(context->id, item.c_str(), &log) != NULL;
}

static inline bool read_item(const std::shared_ptr<DynamixelActuatorContext> &context,
                             const std::string &item, std::int32_t *value) {
  const char *log = nullptr;
  if (!context->dxl_wb->itemRead(context->id, item.c_str(), value, &log)) {
    lhd_error("read_item(): Failed to read control table item \"%s\" of %s: %s", //
              item, get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::itemRead()"));
    return false;
  }
  return true;
}

static inline bool read_position(const std::shared_ptr<DynamixelActuatorContext> &context) {
  float rad;
  const char *log = nullptr;
  if (!context->dxl_wb->getRadian(context->id, &rad, &log)) {
    lhd_error("read_position(): Failed to read position from %s: %s", //
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::getRadian()"));
    return false;
  }
  context->pos = rad;
  return true;
}

static inline bool read_velocity(const std::shared_ptr<DynamixelActuatorContext> &context) {
  std::int32_t value;
  // As of dynamixel_workbench_toolbox v2.0.0,
  // DynamixelWorkbench::getVelocity() reads a wrong item ...
  if (!read_item(context, "Present_Velocity", &value)) {
    return false;
  }
  context->vel = context->dxl_wb->convertValue2Velocity(context->id, value);
  return true;
}

static inline bool has_effort(const std::shared_ptr<DynamixelActuatorContext> &context) {
  return has_item(context, "Present_Current");
}

static inline bool read_effort(const std::shared_ptr<DynamixelActuatorContext> &context) {
  std::int32_t value;
  if (!read_item(context, "Present_Current", &value)) {
    return false;
  }
  // mA -> N*m
  context->eff =
      context->dxl_wb->convertValue2Current(context->id, value) * context->torque_constant / 1000.0;
  return true;
}

static inline bool read_all_states(const std::shared_ptr<DynamixelActuatorContext> &context) {
  // the owning layer may have already filled the states with a bus-wide sync read,
  // which costs one round trip for the whole bus instead of three per actuator
  if (context->states_fresh) {
    return true;
  }
  // if one fails, "return read_position() && read_velocity() && ..." does not call others.
  // on the other hand, lines below call all anyway to read info as much as possible.
  const bool pos_result = read_position(context);
  const bool vel_result = read_velocity(context);
  const bool eff_result = has_effort(context) ? read_effort(context) : true;
  return pos_result && vel_result && eff_result;
}

// sync read functions
//
// reading the present states of n actuators one control table item at a time costs 3n round
// trips on the serial bus, which dominates the cycle time of a multi-actuator bus. the helpers
// below instead issue a single sync read of the contiguous block that spans Present_Current,
// Present_Velocity and Present_Position (adjacent on every X series model), so the whole bus
// is read in one round trip. all addresses come from the control table at runtime, so models
// that lay the items out differently are detected and fall back to the per-item path.

struct SyncStateLayout {
  bool valid = false;
  bool has_effort = false;
  // the contiguous block covering all of the items below
  std::uint16_t block_address = 0, block_length = 0;
  std::uint16_t pos_address = 0, pos_length = 0;
  std::uint16_t vel_address = 0, vel_length = 0;
  std::uint16_t eff_address = 0, eff_length = 0;
  std::vector<std::uint8_t> ids;
};

// upper bound on the bytes fetched from each actuator by one sync read. a block wider than this
// means the items are not really adjacent, and shipping the gap would cost more than it saves.
static constexpr std::uint16_t max_sync_state_block_length = 32;

// describes the single sync read that would fill the states of all the given actuators,
// or returns an invalid layout if the actuators cannot be read that way
static inline SyncStateLayout
make_sync_state_layout(const std::vector<std::shared_ptr<DynamixelActuatorContext>> &contexts) {
  if (contexts.empty()) {
    return SyncStateLayout();
  }

  // sync read is an instruction of protocol 2.0
  const float protocol_version = contexts.front()->dxl_wb->getProtocolVersion();
  if (protocol_version < 2.0f) {
    lhd_info("make_sync_state_layout(): Sync read needs protocol 2.0 but the bus speaks %.1f. "
             "Falling back to reading states actuator by actuator.",
             protocol_version);
    return SyncStateLayout();
  }

  // every actuator has to agree, because a sync read fetches the same block from all of them.
  // deciding from the first alone would leave the others' effort unread but marked fresh.
  std::size_t num_with_effort = 0;
  for (const auto &context : contexts) {
    if (has_effort(context)) {
      ++num_with_effort;
    }
  }
  if (num_with_effort != 0 && num_with_effort != contexts.size()) {
    lhd_info("make_sync_state_layout(): Only %d of %d actuators on this bus expose "
             "Present_Current. Falling back to reading states actuator by actuator.",
             static_cast<int>(num_with_effort), static_cast<int>(contexts.size()));
    return SyncStateLayout();
  }

  SyncStateLayout layout;
  layout.has_effort = num_with_effort == contexts.size();
  for (std::size_t i = 0; i < contexts.size(); ++i) {
    const auto &context = contexts[i];
    const ControlItem *const pos = context->dxl_wb->getItemInfo(context->id, "Present_Position");
    const ControlItem *const vel = context->dxl_wb->getItemInfo(context->id, "Present_Velocity");
    const ControlItem *const eff =
        layout.has_effort ? context->dxl_wb->getItemInfo(context->id, "Present_Current") : nullptr;
    if (!pos || !vel || (layout.has_effort && !eff)) {
      lhd_info("make_sync_state_layout(): %s does not expose all the present state items. "
               "Falling back to reading states actuator by actuator.",
               get_display_name(*context));
      return SyncStateLayout();
    }
    if (i == 0) {
      layout.pos_address = pos->address;
      layout.pos_length = pos->data_length;
      layout.vel_address = vel->address;
      layout.vel_length = vel->data_length;
      if (layout.has_effort) {
        layout.eff_address = eff->address;
        layout.eff_length = eff->data_length;
      }
    } else if (pos->address != layout.pos_address || pos->data_length != layout.pos_length ||
               vel->address != layout.vel_address || vel->data_length != layout.vel_length ||
               (layout.has_effort && (eff->address != layout.eff_address ||
                                      eff->data_length != layout.eff_length))) {
      // a sync read fetches the same address range from every actuator
      lhd_info("make_sync_state_layout(): %s places the present state items differently from %s. "
               "Falling back to reading states actuator by actuator.",
               get_display_name(*context), get_display_name(*contexts.front()));
      return SyncStateLayout();
    }
    layout.ids.push_back(context->id);
  }

  // the block has to span every item so that one sync read fills them all
  std::uint16_t block_begin = layout.pos_address,
                block_end = layout.pos_address + layout.pos_length;
  const auto extend_block = [&block_begin, &block_end](const std::uint16_t address,
                                                       const std::uint16_t length) {
    block_begin = std::min<std::uint16_t>(block_begin, address);
    block_end = std::max<std::uint16_t>(block_end, address + length);
  };
  extend_block(layout.vel_address, layout.vel_length);
  if (layout.has_effort) {
    extend_block(layout.eff_address, layout.eff_length);
  }
  if (block_end - block_begin > max_sync_state_block_length) {
    lhd_info("make_sync_state_layout(): The present state items of %s span %d bytes, over the %d "
             "byte limit of a single block read. Falling back to reading states actuator by "
             "actuator.",
             get_display_name(*contexts.front()), block_end - block_begin,
             max_sync_state_block_length);
    return SyncStateLayout();
  }

  layout.block_address = block_begin;
  layout.block_length = block_end - block_begin;
  layout.valid = true;
  return layout;
}

// registers a sync read handler for the given layout on the given bus.
// returns the handler index, or a negative value on failure.
static inline int add_sync_state_handler(const std::shared_ptr<DynamixelActuatorContext> &context,
                                         const SyncStateLayout &layout) {
  if (!layout.valid) {
    return -1;
  }
  // DynamixelWorkbench numbers handlers in the order they are added
  const int index = context->dxl_wb->getTheNumberOfSyncReadHandler();
  const char *log = nullptr;
  if (!context->dxl_wb->addSyncReadHandler(layout.block_address, layout.block_length, &log)) {
    lhd_error("add_sync_state_handler(): Failed to add a sync read handler of %d bytes from "
              "address %d: %s",
              layout.block_length, layout.block_address,
              (log ? log : "No log from DynamixelWorkbench::addSyncReadHandler()"));
    return -1;
  }
  return index;
}

// scratch buffers owned by the layer, so that a control cycle allocates nothing
struct SyncStateBuffers {
  std::vector<std::uint8_t> ids;
  std::vector<std::int32_t> pos_values, vel_values, eff_values;

  void resize(const std::size_t size) {
    ids.resize(size);
    pos_values.resize(size);
    vel_values.resize(size);
    eff_values.resize(size);
  }
};

static inline bool get_sync_read_data(const std::shared_ptr<DynamixelWorkbench> &dxl_wb,
                                      const int handler_index, std::vector<std::uint8_t> *const ids,
                                      const std::uint16_t address, const std::uint16_t length,
                                      std::vector<std::int32_t> *const values) {
  const char *log = nullptr;
  if (!dxl_wb->getSyncReadData(static_cast<std::uint8_t>(handler_index), ids->data(),
                               static_cast<std::uint8_t>(ids->size()), address, length,
                               values->data(), &log)) {
    lhd_error("get_sync_read_data(): Failed to take %d bytes at address %d out of the sync read "
              "result: %s",
              length, address,
              (log ? log : "No log from DynamixelWorkbench::getSyncReadData()"));
    return false;
  }
  return true;
}

// fills the states of all the given actuators with a single sync read, and marks them fresh
// so that read_all_states() skips its per-item round trips for this cycle
static inline bool
sync_read_states(const std::vector<std::shared_ptr<DynamixelActuatorContext>> &contexts,
                 const SyncStateLayout &layout, const int handler_index,
                 SyncStateBuffers *const buffers) {
  if (!layout.valid || handler_index < 0 || contexts.size() != layout.ids.size()) {
    return false;
  }

  const std::shared_ptr<DynamixelWorkbench> &dxl_wb = contexts.front()->dxl_wb;
  buffers->resize(contexts.size());
  std::vector<std::uint8_t> &ids = buffers->ids;
  ids = layout.ids;
  const char *log = nullptr;
  if (!dxl_wb->syncRead(static_cast<std::uint8_t>(handler_index), ids.data(),
                        static_cast<std::uint8_t>(ids.size()), &log)) {
    lhd_error("sync_read_states(): Failed to sync read the states of %d actuators: %s",
              static_cast<int>(ids.size()),
              (log ? log : "No log from DynamixelWorkbench::syncRead()"));
    return false;
  }

  std::vector<std::int32_t> &pos_values = buffers->pos_values;
  std::vector<std::int32_t> &vel_values = buffers->vel_values;
  std::vector<std::int32_t> &eff_values = buffers->eff_values;
  if (!get_sync_read_data(dxl_wb, handler_index, &ids, layout.pos_address, layout.pos_length,
                          &pos_values) ||
      !get_sync_read_data(dxl_wb, handler_index, &ids, layout.vel_address, layout.vel_length,
                          &vel_values) ||
      (layout.has_effort && !get_sync_read_data(dxl_wb, handler_index, &ids, layout.eff_address,
                                                layout.eff_length, &eff_values))) {
    return false;
  }

  for (std::size_t i = 0; i < contexts.size(); ++i) {
    const auto &context = contexts[i];
    context->pos = context->dxl_wb->convertValue2Radian(context->id, pos_values[i]);
    context->vel = context->dxl_wb->convertValue2Velocity(context->id, vel_values[i]);
    if (layout.has_effort) {
      // mA -> N*m. narrowing to 16 bits restores the sign, as DynamixelWorkbench does
      // when reading Present_Current item by item
      context->eff = context->dxl_wb->convertValue2Current(
                         context->id, static_cast<std::int16_t>(eff_values[i])) *
                     context->torque_constant / 1000.0;
    }
    context->states_fresh = true;
  }
  return true;
}

// write functions

static inline bool
enable_operating_mode(const std::shared_ptr<DynamixelActuatorContext> &context,
                      bool (DynamixelWorkbench::*const set_func)(std::uint8_t, const char **)) {
  const char *log;
  // disable torque to make the actuator ready to change operating modes
  log = nullptr;
  if (!context->dxl_wb->torqueOff(context->id, &log)) {
    lhd_error("enable_operating_mode(): Failed to disable torque of %s: %s",
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::torqueOff()"));
    return false;
  }
  // change operating modes
  log = nullptr;
  if (!(context->dxl_wb.get()->*set_func)(context->id, &log)) {
    lhd_error("enable_operating_mode(): Failed to set operating mode of %s: %s",
              get_display_name(*context), (log ? log : "No log from DynamixelWorkbench"));
    return false;
  }
  // activate new operating mode by enabling torque
  log = nullptr;
  if (!context->dxl_wb->torqueOn(context->id, &log)) {
    lhd_error("enable_operating_mode(): Failed to enable torque of %s: %s",
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::torqueOn()"));
    return false;
  }
  return true;
}

static inline bool torque_off(const std::shared_ptr<DynamixelActuatorContext> &context) {
  const char *log = nullptr;
  if (!context->dxl_wb->torqueOff(context->id, &log)) {
    lhd_error("torque_off(): Failed to disable torque of %s: %s", //
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::torqueOff()"));
    return false;
  }
  return true;
}

static inline bool clear_multi_turn(const std::shared_ptr<DynamixelActuatorContext> &context) {
  const char *log = nullptr;
  if (!context->dxl_wb->clearMultiTurn(context->id, &log)) {
    lhd_error("clear_multi_turn(): Failed to clear multi turn count of %s: %s",
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::clearMultiTurn()"));
    return false;
  }
  return true;
}

static inline bool write_item(const std::shared_ptr<DynamixelActuatorContext> &context,
                              const std::string &item, const std::int32_t value) {
  const char *log = nullptr;
  if (!context->dxl_wb->itemWrite(context->id, item.c_str(), value, &log)) {
    lhd_error("write_item(): Failed to set control table item \"%s\" of %s: %s", //
              item, get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::itemWrite()"));
    return false;
  }
  return true;
}

// queues a control table write for the layer to send as part of a sync write, or performs it
// immediately when no one is going to flush the queue. writing item by item costs 11.0 ms each
// because DynamixelWorkbench::writeRegister() sleeps 10 ms after every write; a sync write of
// the whole bus costs well under a millisecond and does not sleep at all.
static inline bool queue_item(const std::shared_ptr<DynamixelActuatorContext> &context,
                              const char *const item, const std::int32_t value) {
  if (!context->defer_writes || context->num_pending_writes >= context->pending_writes.size()) {
    return write_item(context, item, value);
  }
  const char *log = nullptr;
  const ControlItem *const info = context->dxl_wb->getItemInfo(context->id, item, &log);
  if (!info) {
    lhd_error("queue_item(): %s does not have the control table item \"%s\": %s",
              get_display_name(*context), item,
              (log ? log : "No log from DynamixelWorkbench::getItemInfo()"));
    return false;
  }
  context->pending_writes[context->num_pending_writes] = {item, info->address, info->data_length,
                                                          value};
  ++context->num_pending_writes;
  return true;
}

static inline bool
write_position_command(const std::shared_ptr<DynamixelActuatorContext> &context) {
  return queue_item(
      context, "Goal_Position",
      context->dxl_wb->convertRadian2Value(context->id, static_cast<float>(context->pos_cmd)));
}

static inline bool
write_velocity_command(const std::shared_ptr<DynamixelActuatorContext> &context) {
  return queue_item(
      context, "Goal_Velocity",
      context->dxl_wb->convertVelocity2Value(context->id, static_cast<float>(context->vel_cmd)));
}

static inline bool
write_profile_velocity(const std::shared_ptr<DynamixelActuatorContext> &context) {
  return queue_item(context, "Profile_Velocity",
                    context->dxl_wb->convertVelocity2Value(
                        context->id, static_cast<float>(std::abs(context->vel_cmd))));
}

static inline bool write_effort_command(const std::shared_ptr<DynamixelActuatorContext> &context) {
  // N*m -> mA
  return queue_item(
      context, "Goal_Current",
      context->dxl_wb->convertCurrent2Value(
          context->id, static_cast<float>(context->eff_cmd / context->torque_constant * 1000.0)));
}

// caches one sync write handler per control table item written by the operating modes.
// DynamixelWorkbench allows MAX_HANDLER_NUM (5) handlers in total, one of which the sync read
// takes, so an exotic mix of modes can exhaust them and falls back to the blocking path.
struct SyncWriteHandlers {
  static constexpr std::size_t capacity = 4;
  std::array<std::uint32_t, capacity> keys{};
  std::array<int, capacity> indices{};
  std::size_t size = 0;
  // scratch buffers, so that a control cycle allocates nothing
  std::vector<std::uint8_t> ids;
  std::vector<std::int32_t> values;

  static std::uint32_t key_of(const std::uint16_t address, const std::uint16_t length) {
    return (static_cast<std::uint32_t>(address) << 16) | length;
  }

  // returns the handler index for the item, registering one on first use, or a negative
  // value if DynamixelWorkbench cannot take another handler
  int index_for(const std::shared_ptr<DynamixelWorkbench> &dxl_wb, const std::uint16_t address,
                const std::uint16_t length) {
    const std::uint32_t key = key_of(address, length);
    for (std::size_t i = 0; i < size; ++i) {
      if (keys[i] == key) {
        return indices[i];
      }
    }
    if (size >= capacity) {
      return -1;
    }
    const int index = dxl_wb->getTheNumberOfSyncWriteHandler();
    const char *log = nullptr;
    if (!dxl_wb->addSyncWriteHandler(address, length, &log)) {
      lhd_error("SyncWriteHandlers::index_for(): Failed to add a sync write handler of %d bytes "
                "to address %d: %s",
                length, address,
                (log ? log : "No log from DynamixelWorkbench::addSyncWriteHandler()"));
      return -1;
    }
    keys[size] = key;
    indices[size] = index;
    ++size;
    return index;
  }
};

// sends every queued write with one sync write per control table item, and clears the queues.
// an item the handlers cannot cover, or a sync write that fails to reach the bus, falls back to
// the blocking per-actuator path so that no command is silently dropped.
static inline bool
sync_write_pending(const std::vector<std::shared_ptr<DynamixelActuatorContext>> &contexts,
                   SyncWriteHandlers *const handlers) {
  if (contexts.empty()) {
    return true;
  }

  bool all_written = true;
  while (true) {
    // pick the item of the first write still queued anywhere on the bus
    const PendingWrite *item = nullptr;
    for (const auto &context : contexts) {
      if (context->num_pending_writes > 0) {
        item = &context->pending_writes[0];
        break;
      }
    }
    if (!item) {
      return all_written;
    }

    // collect that item's value from every actuator that has one queued
    const char *const item_name = item->item;
    const std::uint16_t address = item->address, length = item->length;
    handlers->ids.clear();
    handlers->values.clear();
    for (const auto &context : contexts) {
      for (std::size_t i = 0; i < context->num_pending_writes; ++i) {
        const PendingWrite &pending = context->pending_writes[i];
        if (pending.address != address || pending.length != length) {
          continue;
        }
        handlers->ids.push_back(context->id);
        handlers->values.push_back(pending.value);
        // remove it from the queue by shifting the rest down
        for (std::size_t k = i + 1; k < context->num_pending_writes; ++k) {
          context->pending_writes[k - 1] = context->pending_writes[k];
        }
        --context->num_pending_writes;
        break;
      }
    }

    const std::shared_ptr<DynamixelWorkbench> &dxl_wb = contexts.front()->dxl_wb;
    const int handler_index = handlers->index_for(dxl_wb, address, length);
    const char *log = nullptr;
    const bool sent =
        handler_index >= 0 &&
        dxl_wb->syncWrite(static_cast<std::uint8_t>(handler_index), handlers->ids.data(),
                          static_cast<std::uint8_t>(handlers->ids.size()), handlers->values.data(),
                          1, &log);
    if (sent) {
      continue;
    }

    lhd_error("sync_write_pending(): Failed to sync write \"%s\" to %d actuators (%s). "
              "Falling back to writing them one by one.",
              item_name, static_cast<int>(handlers->ids.size()),
              (log ? log : "no sync write handler available"));
    for (std::size_t i = 0; i < handlers->ids.size(); ++i) {
      for (const auto &context : contexts) {
        if (context->id == handlers->ids[i]) {
          all_written = write_item(context, item_name, handlers->values[i]) && all_written;
          break;
        }
      }
    }
  }
}

} // namespace layered_hardware_dynamixel

#endif