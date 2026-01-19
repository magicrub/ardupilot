--[[ 
  Freeflight AltaX8 CAN ESC Feedback Driver
--]]

local SCRIPT_NAME = "AltaX8 CAN"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local TelemetryType = {
      TEMPERATURE = 1 << 0,
      MOTOR_TEMPERATURE  = 1 << 1,
      VOLTAGE     = 1 << 2,
      CURRENT     = 1 << 3,
      CONSUMPTION = 1 << 4,
      USAGE       = 1 << 5,
      TEMPERATURE_EXTERNAL = 1 << 6,
      MOTOR_TEMPERATURE_EXTERNAL  = 1 << 7,
      EDT2_STATUS = 1 << 8,
      EDT2_STRESS = 1 << 9,
      INPUT_DUTY  = 1 << 10,
      OUTPUT_DUTY = 1 << 11,
      FLAGS       = 1 << 12,
      POWER_PERCENTAGE = 1 << 13
      }

-- timer constants
local UPDATE_INTERVAL_MS = 10 -- This controls the delay between fetching feedback msg and checking for it's response
local SEND_GET_FEEDBACK_MSG_INTERVAL_MS = uint32_t(100)
local SEND_INIT_MSG_INTERVAL_MS = uint32_t(5000)
local FEEDBACK_TIMEOUT_MS = uint32_t(1000)

-- timer variables
local feedback_msg_timestamps = {uint32_t(0), uint32_t(0), uint32_t(0), uint32_t(0)} -- index count acts as ESC count
local init_ms = uint32_t(0)
local send_get_feedback_msg_ms = uint32_t(0)
local now_ms = millis() -- might as well make it global file-wide so we only need to fetch it once per tick and all functions can see it


local ESC_COUNT = #feedback_msg_timestamps

local CAN_BUF_LEN = 25
local can_driver = CAN:get_device(CAN_BUF_LEN)
if not can_driver then
    gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("%s: Failed to load driver", SCRIPT_NAME))
    gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("%s: Check these params:", SCRIPT_NAME))
    gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("%s: CAN_P1_DRIVER=1 and CAN_D1_PROTOCOL=10", SCRIPT_NAME))
    return
end




-- Type conversion
function get_uint16(frame, indexMSB, indexLSB)
    return (frame:data(indexMSB) << 8) + frame:data(indexLSB)
end


function handle_frame(frame)
--  Example frames:
--  RX    22:32:15.144685    NFD         04D    02 15 B3 01 00 00 00 00
--  RX    22:32:15.144685    NFD         04E    02 00 00 00 00 00 00 00
--  RX    22:32:15.144685    NFD         04D    03 15 AF 01 00 00 00 00
--  RX    22:32:15.144685    NFD         04E    03 00 00 00 00 00 00 00
--  RX    22:32:15.145685    NFD         04D    04 16 B1 01 00 00 00 00
--  RX    22:32:15.145685    NFD         04E    04 00 00 00 00 00 00 00
--  RX    22:32:15.207698    NFD         04D    01 15 B3 01 AA 02 43 00
--  RX    22:32:15.207698    NFD         04E    51 07 B0 00 00 00 00 00
   if frame:isExtended() then
      -- This is not the packet you're looking for...
      return
   end
   local esc_index = (frame:data(0) & 0x0F);
   if esc_index == 0 or esc_index > ESC_COUNT then
      -- invalid esc index
      return
   end

   local telem_data = ESCTelemetryData()
   feedback_msg_timestamps[esc_index] = now_ms

   if frame:id() == 0x4D then
      -- Voltage and RPM
      -- note: 16bit data is LSB first
      telem_data:voltage(get_uint16(frame, 3, 2) * 0.1)
      esc_telem:update_telem_data(esc_index, telem_data, TelemetryType.VOLTAGE)
      esc_telem:update_rpm(esc_index, get_uint16(frame, 5, 4), 0)

   elseif frame:id() == 0x4E then
      -- Current
       -- note: 16bit data is MSB first
      telem_data:current(get_uint16(frame, 1, 2) * 0.0001)
      esc_telem:update_telem_data(esc_index, telem_data, TelemetryType.CURRENT)
   end
end



function send_init_msg()
   local init_msgs = {
      {0x4C, 0x00, 0x80, 0x00, 0x08},
      {0x55, 0x00, 0x80, 0x00, 0x08},
      {0x77, 0x00, 0x70, 0x00, 0x08}
   }

   for row=1, 3 do
      local msg = CANFrame()
      msg:id(0x010)
      for col=1, 5 do
         msg:data(col, init_msgs[row][col])
      end
      msg:dlc(5)
      can_driver:write_frame(msg, 10000)
   end
end


function update()
   now_ms = millis()

   for _ = 1, CAN_BUF_LEN do
      local frame = can_driver:read_frame()
      if not frame then
         -- buffer is empty
         break
      end
      handle_frame(frame)
   end

   -- check timeouts and re-init as needed
   if (now_ms - init_ms >= SEND_INIT_MSG_INTERVAL_MS) then
      -- don't send the init msg too often but if it's expired then lets check the timeouts at full-speed
      for esc_index=1, ESC_COUNT do
         -- Check for timeout on each ESC
         if (now_ms - feedback_msg_timestamps[esc_index] > FEEDBACK_TIMEOUT_MS) then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: re-init, ESC %d feedback timed out ", SCRIPT_NAME, esc_index))
            init_ms = now_ms
            send_init_msg()

            -- only send one init msg for when ANY ESC is timed out. All ESCs will get re-init msgs
            return update, UPDATE_INTERVAL_MS
         end
      end
   end

   -- Send request-feedback message at regular interval
   if (now_ms - send_get_feedback_msg_ms >= SEND_GET_FEEDBACK_MSG_INTERVAL_MS) then
      send_get_feedback_msg_ms = now_ms
      for esc_index=1, ESC_COUNT do
         -- Send request-feedback msg to each ESC
         local requestFeedbackMsg = CANFrame()
         requestFeedbackMsg:id(0x02A)
         requestFeedbackMsg:data(0, esc_index)
         requestFeedbackMsg:dlc(1)
         can_driver:write_frame(requestFeedbackMsg, 10000)
      end
   end

   return update, UPDATE_INTERVAL_MS
end


function init()
   gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Starting Driver", SCRIPT_NAME))
   init_ms = millis()
   send_init_msg()
   return update()
end

return init()
