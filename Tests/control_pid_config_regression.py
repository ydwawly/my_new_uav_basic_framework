"""控制链路配置、调度契约与日志 ABI 的静态回归检查。"""

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROL_HEADER = (ROOT / "Application/App_Control/App_Control.h").read_text(encoding="utf-8")
CONTROL_SOURCE = (ROOT / "Application/App_Control/App_Control.c").read_text(encoding="utf-8")
ATTITUDE_HEADER = (ROOT / "Application/App_attitude/App_attitude.h").read_text(encoding="utf-8")
ATTITUDE_INTERNAL = (ROOT / "Application/App_attitude/App_attitude_internal.h").read_text(encoding="utf-8")
ATTITUDE_CONFIG = (ROOT / "Application/App_attitude/App_attitude_config.h").read_text(encoding="utf-8")
ATTITUDE_SOURCE = (ROOT / "Application/App_attitude/App_attitude.c").read_text(encoding="utf-8")
COMMAND_HEADER = (ROOT / "Application/App_Uav_Cmd/App_Uav_Cmd.h").read_text(encoding="utf-8")
COMMAND_CONFIG = (ROOT / "Application/App_Uav_Cmd/App_Uav_Cmd_config.h").read_text(encoding="utf-8")
COMMAND_SOURCE = (ROOT / "Application/App_Uav_Cmd/App_Uav_Cmd.c").read_text(encoding="utf-8")
SD_HEADER = (ROOT / "Modules/modules_SD_Card/modules_SD_Card.h").read_text(encoding="utf-8")
SD_SOURCE = (ROOT / "Modules/modules_SD_Card/modules_SD_Card.c").read_text(encoding="utf-8")
SD_INTERNAL = (ROOT / "Modules/modules_SD_Card/modules_SD_Card_internal.h").read_text(encoding="utf-8")
SD_TRANSFER = (ROOT / "Modules/modules_SD_Card/modules_SD_Card_log_transfer.c").read_text(encoding="utf-8")
LOG_SOURCE = (ROOT / "Application/App_Data_Comm/log_service.c").read_text(encoding="utf-8")
CMAKE = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")


def assert_pid_configuration() -> None:
    assert "static const PID_Init_Config_s rate_rp_config" in CONTROL_SOURCE
    assert ".Kp            = CONTROL_RATE_RP_KP" in CONTROL_SOURCE
    assert ".Ki            = CONTROL_RATE_RP_KI" in CONTROL_SOURCE
    assert ".MaxOut        = CONTROL_RATE_RP_MAX_OUT" in CONTROL_SOURCE
    assert ".IntegralLimit = CONTROL_RATE_RP_INTEGRAL_LIMIT" in CONTROL_SOURCE

    assert "static const PID_Init_Config_s rate_yaw_config" in CONTROL_SOURCE
    assert ".Kp            = CONTROL_RATE_YAW_KP" in CONTROL_SOURCE
    assert ".Ki            = CONTROL_RATE_YAW_KI" in CONTROL_SOURCE
    assert ".MaxOut        = CONTROL_RATE_YAW_MAX_OUT" in CONTROL_SOURCE
    assert ".IntegralLimit = CONTROL_RATE_YAW_INTEGRAL_LIMIT" in CONTROL_SOURCE

    assert "static const PID_Init_Config_s altitude_config" in CONTROL_SOURCE
    assert ".Kp            = CONTROL_ALT_VELOCITY_KP" in CONTROL_SOURCE
    assert ".Ki            = CONTROL_ALT_VELOCITY_KI" in CONTROL_SOURCE
    assert "PID_Trapezoid_Integral" in CONTROL_SOURCE

    roll_init = CONTROL_SOURCE.index("PIDInit(&control.roll_rate_pid, &rate_rp_config);")
    pitch_init = CONTROL_SOURCE.index("PIDInit(&control.pitch_rate_pid, &rate_rp_config);")
    yaw_init = CONTROL_SOURCE.index("PIDInit(&control.yaw_rate_pid, &rate_yaw_config);")
    altitude_init = CONTROL_SOURCE.index("PIDInit(&control.altitude_velocity_pid, &altitude_config);")
    assert roll_init < pitch_init < yaw_init < altitude_init


def assert_notification_driven_control() -> None:
    assert "xTaskNotifyGive(task_handle);" in CONTROL_SOURCE
    assert "ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONTROL_TASK_MAX_WAIT_MS))" in CONTROL_SOURCE
    timeout_start = CONTROL_SOURCE.index("if (notification_count == 0U)")
    normal_start = CONTROL_SOURCE.index("Control_Feedback_t feedback", timeout_start)
    timeout_path = CONTROL_SOURCE[timeout_start:normal_start]
    assert "Control_Reset();" in timeout_path
    assert "continue;" in timeout_path
    assert "Control_Run(" not in timeout_path
    assert "Control_UpdateAltitude(" not in timeout_path

    assert "taskENTER_CRITICAL();" in CONTROL_SOURCE
    assert "*feedback = control.feedback;" in CONTROL_SOURCE
    assert "taskEXIT_CRITICAL();" in CONTROL_SOURCE
    assert "control.feedback                       = *feedback;" in CONTROL_SOURCE
    writer_start = CONTROL_SOURCE.index("void Control_SetFeedback")
    writer_end = CONTROL_SOURCE.index("static uint8_t Control_TimeFresh", writer_start)
    assert "taskENTER_CRITICAL" not in CONTROL_SOURCE[writer_start:writer_end]


def assert_safety_and_control_contracts() -> None:
    combined = CONTROL_HEADER + CONTROL_SOURCE + CMAKE
    for marker in ("YAW_BENCH", "RateBench", "rate_bench", "yaw_bench"):
        assert marker not in combined

    assert "control.uav_cmd.throttle > CONTROL_ARM_THROTTLE_MAX" in CONTROL_SOURCE
    assert "throttle < CONTROL_RATE_LOOP_MIN_THROTTLE" in CONTROL_SOURCE
    assert "Motor_EmergencyStop();" in CONTROL_SOURCE
    assert "Control_TimeFresh(now, control.uav_cmd.timestamp_us, CONTROL_CMD_TIMEOUT_US)" in CONTROL_SOURCE
    assert "Control_TimeFresh(now, feedback->timestamp_us, CONTROL_FEEDBACK_TIMEOUT_US)" in CONTROL_SOURCE
    assert "CONTROL_NAVIGATION_ESKF_VALID | CONTROL_NAVIGATION_RANGE_FUSED" in CONTROL_SOURCE
    assert "const float height = -feedback->pos_ned_m[2];" in CONTROL_SOURCE
    assert "const float vertical_speed = -feedback->vel_ned_mps[2];" in CONTROL_SOURCE
    assert "Control_Mixer(throttle, rate_output, motor);" in CONTROL_SOURCE
    assert "Motor_SetOutput(motor)" in CONTROL_SOURCE

    assert re.search(r"#define\s+ATTITUDE_ENABLE_FLOW\s+1U", ATTITUDE_CONFIG)
    assert re.search(r"#define\s+ATTITUDE_ENABLE_RANGE\s+1U", ATTITUDE_CONFIG)
    assert re.search(r"#define\s+ATTITUDE_ALIGNMENT_SAMPLES\s+2000U", ATTITUDE_CONFIG)
    assert "float q_nb[4];" in CONTROL_HEADER
    assert '#include "VQF_C.h"' in ATTITUDE_INTERNAL
    assert "VqfC_Update(sample->gyro_rps" in ATTITUDE_SOURCE
    assert "NAV_ESKF_Predict(&nav_eskf_instance" in ATTITUDE_SOURCE
    assert "VqfC_ZeroHeading" in ATTITUDE_SOURCE

    assert "uav_cmd->pitch_ref    = -pitch_stick * CONTROL_MAX_PITCH_ANGLE_RAD;" in COMMAND_SOURCE
    assert "#define RC_CHANNEL_ALT_HOLD 5U" in COMMAND_CONFIG
    assert "altitude_hold_request" in COMMAND_HEADER


def assert_log_and_transfer_abi() -> None:
    for version, size in ((2, 272), (3, 280), (4, 296)):
        assert f"SDCard_FlightControlPayloadV{version}_t" in SD_HEADER
        assert f"sizeof(SDCard_FlightControlPayloadV{version}_t) == {size}U" in SD_HEADER

    assert "SD_CARD_MSG_FLIGHT_CONTROL, 4U" in LOG_SOURCE
    assert "base->throttle       = snapshot->effective_throttle" in LOG_SOURCE
    assert "payload->pilot_throttle" in LOG_SOURCE
    assert "payload->yaw_target_rad" in LOG_SOURCE
    assert "payload->motor_armed" in LOG_SOURCE

    assert '#include "queue.h"' not in SD_HEADER
    assert "SDCard_LogTransferContext_t" in SD_INTERNAL
    assert "SDCard_LogTransfer_ProcessRequests();" in SD_SOURCE
    assert "SDCard_LogTransfer_Process();" in SD_SOURCE
    assert "bool SDCard_RequestLogData" in SD_TRANSFER
    assert "bool SDCard_PopLogResponse" in SD_TRANSFER


def main() -> None:
    assert_pid_configuration()
    assert_notification_driven_control()
    assert_safety_and_control_contracts()
    assert_log_and_transfer_abi()
    print("Control regression passed: configuration, scheduling, safety and log ABI contracts verified.")


if __name__ == "__main__":
    main()
