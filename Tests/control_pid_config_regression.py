"""正式飞行控制PID配置与无台架调试代码的静态回归检查。"""

import re

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER = (ROOT / "Application/App_Control/App_Control.h").read_text(encoding="utf-8")
SOURCE = (ROOT / "Application/App_Control/App_Control.c").read_text(encoding="utf-8")
CMAKE = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
ATTITUDE_HEADER = (ROOT / "Application/App_attitude/App_attitude.h").read_text(encoding="utf-8")
ATTITUDE_CONFIG = (ROOT / "Application/App_attitude/App_attitude_config.h").read_text(encoding="utf-8")
ATTITUDE_SOURCE = (ROOT / "Application/App_attitude/App_attitude.c").read_text(encoding="utf-8")
COMMAND_HEADER = (ROOT / "Application/App_Uav_Cmd/App_Uav_Cmd.h").read_text(encoding="utf-8")
COMMAND_CONFIG = (ROOT / "Application/App_Uav_Cmd/App_Uav_Cmd_config.h").read_text(encoding="utf-8")
SD_HEADER = (ROOT / "Modules/modules_SD_Card/modules_SD_Card.h").read_text(encoding="utf-8")
SD_SOURCE = (ROOT / "Modules/modules_SD_Card/modules_SD_Card.c").read_text(encoding="utf-8")
SD_INTERNAL = (ROOT / "Modules/modules_SD_Card/modules_SD_Card_internal.h").read_text(encoding="utf-8")
SD_TRANSFER = (ROOT / "Modules/modules_SD_Card/modules_SD_Card_log_transfer.c").read_text(encoding="utf-8")
LOG_SOURCE = (ROOT / "Application/App_Data_Comm/log_service.c").read_text(encoding="utf-8")


def main() -> None:
    roll_config = re.search(
        r"static const PID_Init_Config_s roll_pitch_rate_pid_config = \{(?P<body>.*?)\n\};",
        SOURCE,
        re.DOTALL,
    )
    yaw_config = re.search(
        r"static const PID_Init_Config_s yaw_rate_pid_config = \{(?P<body>.*?)\n\};",
        SOURCE,
        re.DOTALL,
    )
    assert roll_config is not None
    assert yaw_config is not None

    roll_body = roll_config.group("body")
    yaw_body = yaw_config.group("body")
    assert ".Kp            = 0.08f," in roll_body
    assert ".Ki            = 0.015f," in roll_body
    assert ".Kd            = 0.0f," in roll_body
    assert ".IntegralLimit = 0.04f," in roll_body
    assert ".Improve       = PID_Integral_Limit," in roll_body
    assert ".Kp            = 0.14f," in yaw_body
    assert ".Ki            = 0.015f," in yaw_body
    assert ".Kd            = 0.0f," in yaw_body
    assert ".IntegralLimit = 0.04f," in yaw_body
    assert ".Improve       = PID_Integral_Limit," in yaw_body

    roll_init = SOURCE.index("PIDInit(&control_instance.roll_rate_pid, &roll_pitch_rate_pid_config);")
    pitch_init = SOURCE.index("PIDInit(&control_instance.pitch_rate_pid, &roll_pitch_rate_pid_config);")
    yaw_init = SOURCE.index("PIDInit(&control_instance.yaw_rate_pid, &yaw_rate_pid_config);")
    assert roll_init < pitch_init < yaw_init

    combined = HEADER + SOURCE + CMAKE
    control_module = HEADER + SOURCE
    for marker in ("YAW_BENCH", "RateBench", "rate_bench", "yaw_bench"):
        assert marker not in combined, f"debug bench marker remains: {marker}"
    assert "\n        DEBUG\n" not in CMAKE

    assert re.search(r"#define\s+CONTROL_CLOSED_LOOP_ENABLE_THROTTLE\s+0\.10f", control_module)
    assert re.search(r"#define\s+CONTROL_CLOSED_LOOP_DISABLE_THROTTLE\s+0\.07f", control_module)
    assert "CONTROL_ALT_HOLD_ARM_CENTER_TOLERANCE" not in control_module
    assert "control_instance.uav_cmd.throttle > CONTROL_ARM_THROTTLE_MAX" in SOURCE
    assert "Control_RunGroundIdle" in SOURCE
    assert "control_instance.yaw_target_rad   = feedback->euler_rad[2];" in SOURCE
    assert "{effective_throttle, effective_throttle, effective_throttle, effective_throttle}" in SOURCE

    assert "#define ATTITUDE_ENABLE_FLOW 1U" in ATTITUDE_CONFIG
    assert "#define ATTITUDE_ENABLE_RANGE 1U" in ATTITUDE_CONFIG
    assert "#define ATTITUDE_ALIGNMENT_SAMPLES 2000U" in ATTITUDE_CONFIG
    assert "control_q_nb" in ATTITUDE_HEADER
    assert '#include "VQF_C.h"' in ATTITUDE_HEADER
    assert "VqfC_Update(sample->gyro_rps" in ATTITUDE_SOURCE
    assert "VqfC_ZeroHeading" in ATTITUDE_SOURCE
    assert "AttitudeControl_Update" not in ATTITUDE_SOURCE
    assert not (ROOT / "Application/App_attitude/Control_Attitude.c").exists()
    assert "gyro_rps[axis] - output->state.gyro_bias_rps[axis]" in ATTITUDE_SOURCE

    command_source = (ROOT / "Application/App_Uav_Cmd/App_Uav_Cmd.c").read_text(encoding="utf-8")
    assert "uav_cmd->pitch_ref    = -pitch_stick * CONTROL_MAX_PITCH_ANGLE_RAD;" in command_source

    assert "#define RC_CHANNEL_ALT_HOLD 5U" in COMMAND_CONFIG
    assert "altitude_hold_request" in COMMAND_HEADER
    assert "#define CONTROL_ALT_HOLD_POSITION_KP" in control_module
    assert "#define CONTROL_ALT_HOLD_VELOCITY_KP" in control_module
    assert "Control_UpdateAltitudeThrottle" in SOURCE
    assert "Control_PrepareAltitudeGroundIdle" in SOURCE
    assert "control_instance.uav_cmd.arm_request == 0U" in SOURCE
    command_ready = SOURCE[SOURCE.index("static uint8_t Control_CommandIsReady") : SOURCE.index("static uint8_t Control_FeedbackIsReady")]
    assert "arm_request" not in command_ready
    assert "-feedback->pos_ned_m[2]" in SOURCE
    assert "-feedback->vel_ned_mps[2]" in SOURCE
    assert "CONTROL_ALT_HOLD_RANGE_TIMEOUT_US" in SOURCE
    assert "不能让它依赖解锁或定高开关" in SOURCE
    assert "(feedback_ready != 0U) ? Control_NavigationIsReady(&feedback) : 0U" in SOURCE
    assert "control_instance.navigation_valid = navigation_valid;" in SOURCE
    assert "CONTROL_ALT_HOLD_STICK_CENTER" in control_module
    assert re.search(r"#define\s+CONTROL_ALT_HOLD_TAKEOFF_STICK_THRESHOLD\s+0\.54f", control_module)
    assert "CONTROL_NAVIGATION_REJECT_RANGE_STALE" in control_module
    assert "navigation_reject_reason" in SOURCE
    assert "navigation_range_age_us" in SOURCE
    assert "Control_EnterAltitudeManualFallback" in SOURCE
    assert "Control_UpdateAltitudeManualFallback" in SOURCE
    assert "manual throttle retained" in SOURCE
    assert "case CONTROL_ALTITUDE_MODE_ABORTED:" in SOURCE
    assert "CONTROL_ALT_HOLD_LIFTOFF_MIN_THROTTLE" in control_module
    assert "CONTROL_ALTITUDE_MODE_GROUND_IDLE" in HEADER
    assert "CONTROL_ALTITUDE_MODE_TAKEOFF" in HEADER
    assert "CONTROL_ALTITUDE_MODE_HOLD" in HEADER
    assert "switch CH5 OFF" in SOURCE
    assert "RC_ALT_HOLD_SWITCH_ACTIVE_LOW" not in COMMAND_CONFIG
    assert "UAV_CMD_WAIT_TIMEOUT_MS  10U" in command_source
    assert "pdMS_TO_TICKS(UAV_CMD_WAIT_TIMEOUT_MS)" in command_source
    assert "throttle_raw" in COMMAND_HEADER
    assert "altitude_mode_raw" in COMMAND_HEADER
    assert "SDCard_FlightControlPayloadV2_t" in SD_HEADER
    assert "sizeof(SDCard_FlightControlPayloadV2_t) == 272U" in SD_HEADER
    assert "SDCard_FlightControlPayloadV3_t" in SD_HEADER
    assert "sizeof(SDCard_FlightControlPayloadV3_t) == 280U" in SD_HEADER
    assert "SDCard_FlightControlPayloadV4_t" in SD_HEADER
    assert "sizeof(SDCard_FlightControlPayloadV4_t) == 296U" in SD_HEADER
    assert "snapshot.command.throttle = effective_throttle" not in SOURCE
    assert re.search(r"base->throttle\s*= snapshot->effective_throttle", LOG_SOURCE)
    assert re.search(r"payload->pilot_throttle\s*= snapshot->pilot_throttle", LOG_SOURCE)
    assert re.search(r"payload->yaw_target_rad\s*= snapshot->yaw_target_rad", LOG_SOURCE)
    assert re.search(r"payload->motor_armed\s*= snapshot->motor_armed", LOG_SOURCE)
    assert re.search(r"base->control_active\s*= snapshot->closed_loop_active", LOG_SOURCE)
    assert "SD_CARD_MSG_FLIGHT_CONTROL, 4U" in LOG_SOURCE

    assert '#include "queue.h"' not in SD_HEADER
    assert "SDCard_LogRequest_t" not in SD_HEADER
    assert "SDCard_LogTransfer_t" not in SD_HEADER
    assert "SDCard_LogTransferContext_t" in SD_INTERNAL
    assert "SDCard_LogTransfer_ProcessRequests();" in SD_SOURCE
    assert "SDCard_LogTransfer_Process();" in SD_SOURCE
    assert "bool SDCard_RequestLogData" in SD_TRANSFER
    assert "bool SDCard_PopLogResponse" in SD_TRANSFER

    print("Control regression passed: attitude/rate loops, VQF attitude and guarded ESKF-Z altitude hold enabled.")


if __name__ == "__main__":
    main()
