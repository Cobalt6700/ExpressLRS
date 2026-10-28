#if defined(PLATFORM_ESP32)
#include "mode_auto_level.h"
#include "devGyro.h"
#include "gyro.h"
#include "gyro_types.h"
#include "logging.h"

/**
 * Airplane Level/Stable Mode
 *
 * This mode tries to keep the plane flying level (pitch/roll) when there is no
 * stick input.
 */

/**
 * One button trick (Auto-Level mode only)
 *
 * Press the trick button (a channel set to the "Trick" function) to arm a trick.
 * Then, within the arm time, move a stick past the stick threshold:
 *   - Aileron left/right -> axial roll in that direction
 *   - Elevator up/down   -> inside/outside loop
 * The gyro flies the trick with a fixed control surface deflection, counting the
 * rotation from the gyro rate. Once the plane has rotated close to 360 degrees
 * (or the trick times out) control goes back to Auto-Level, which levels the
 * plane out. Pressing the button again cancels an armed or running trick.
 *
 * All the tuning values are set from the LUA menu (Gyro > Gyro Settings > Trick).
 */

void LevelController::initialize(gyro_mode_t mode)
{
    RateController::initialize(mode);

    fm_angle_settings.raw = gyroConfig->GetGyroFMode(mode)->raw; // Default settings for ALL

    const rx_config_gyro_PID_t *roll_pid_params = gyroConfig->GetGyroPID(GYRO_PID_GROUP_ANGLE, GYRO_AXIS_ROLL);
    const rx_config_gyro_PID_t *pitch_pid_params = gyroConfig->GetGyroPID(GYRO_PID_GROUP_ANGLE, GYRO_AXIS_PITCH);

    constexpr float roll_limit = 1.0f;
    constexpr float pitch_limit = 1.0f;

    configure_pid_gains(&pid_angle_roll, roll_pid_params, (int8_t)fm_angle_settings.val.gainRoll, roll_limit, -roll_limit);
    configure_pid_gains(&pid_angle_pitch, pitch_pid_params, (int8_t)fm_angle_settings.val.gainPitch, pitch_limit, -pitch_limit);

    ignore_input[0] = ignore_input[1] = ignore_input[2] = false;

    loadTrickSettings();
    resetTrick();
}

/**
 * Read the trick tuning from the gyro config, keeping each value in a safe range.
 */
void LevelController::loadTrickSettings()
{
    const rx_config_gyro_trick_t *t = gyroConfig->GetGyroTrick();

    trick_arm_ms = constrain((uint32_t)t->val.armTime, 1U, 10U) * 1000U;
    trick_stick_threshold = constrain((float)t->val.stickThreshold, 20.0f, 100.0f) / 100.0f;
    trick_roll_deflection = constrain((float)t->val.rollDeflection, 20.0f, 100.0f) / 100.0f;
    trick_loop_deflection = constrain((float)t->val.loopDeflection, 20.0f, 100.0f) / 100.0f;
    trick_roll_end = degToRad(360.0f - constrain((float)t->val.rollExit, 0.0f, 120.0f));
    trick_loop_end = degToRad(360.0f - constrain((float)t->val.loopExit, 0.0f, 120.0f));
    trick_roll_timeout_ms = constrain((uint32_t)t->val.rollTimeout, 1U, 15U) * 1000U;
    trick_loop_timeout_ms = constrain((uint32_t)t->val.loopTimeout, 1U, 15U) * 1000U;
}

void LevelController::setTrickButton(bool pressed)
{
    trick_button = pressed;
}

void LevelController::resetTrick()
{
    trick_state = TRICK_IDLE;
    trick_direction = 0.0f;
    trick_rotation = 0.0f;
}

void LevelController::startTrick(trick_state_t trick, float stick)
{
    DBGLN("Gyro: Trick start %s %s", trick == TRICK_ROLL ? "Roll" : "Loop", stick > 0 ? "+" : "-");
    trick_state = trick;
    trick_direction = stick > 0 ? 1.0f : -1.0f;
    trick_rotation = 0.0f;
    trick_start_ms = millis();
    trick_last_us = micros();
}

void LevelController::endTrick()
{
    DBGLN("Gyro: Trick end, rotation=%f deg", radToDeg(trick_rotation));
    resetTrick();
    // Start the level controllers fresh so they don't kick from stale state
    pid_angle_roll.reset();
    pid_angle_pitch.reset();
}

/**
 * Run the one button trick state machine.
 * Returns true while a trick is flying the plane (normal levelling is skipped).
 */
bool LevelController::updateTrick(const float input_rpy[], const float gyro_rpy[])
{
    const bool pressed = trick_button && !trick_button_prev;
    trick_button_prev = trick_button;

    // Tricks are only allowed in Auto-Level (Launch mode shares this controller)
    if (mode != GYRO_MODE_LEVEL)
    {
        if (trick_state != TRICK_IDLE)
            endTrick();
        return false;
    }

    const uint32_t now_ms = millis();

    switch (trick_state)
    {
    case TRICK_IDLE:
        if (pressed)
        {
            DBGLN("Gyro: Trick armed");
            trick_state = TRICK_ARMED;
            trick_start_ms = now_ms;
        }
        return false;

    case TRICK_ARMED:
    {
        if (pressed || (now_ms - trick_start_ms) > trick_arm_ms)
        {
            DBGLN("Gyro: Trick disarmed");
            resetTrick();
            return false;
        }

        // Only start a trick from (roughly) level flight
        if (isInverted(gyro.ahrs->angle_rpy) || isHighPitch(gyro.ahrs->angle_rpy))
            return false;

        const float roll = input_rpy[GYRO_AXIS_ROLL];
        const float pitch = input_rpy[GYRO_AXIS_PITCH];
        if (fabsf(roll) >= trick_stick_threshold && fabsf(roll) >= fabsf(pitch))
        {
            startTrick(TRICK_ROLL, roll);
            break;
        }
        if (fabsf(pitch) >= trick_stick_threshold)
        {
            startTrick(TRICK_LOOP, pitch);
            break;
        }
        return false;
    }

    case TRICK_ROLL:
    case TRICK_LOOP:
        break;
    }

    // A trick is running
    if (pressed)
    {
        DBGLN("Gyro: Trick cancelled");
        endTrick();
        return false;
    }

    const uint32_t now_us = micros();
    const float dt = (float)(now_us - trick_last_us) / 1000000.0f;
    trick_last_us = now_us;

    const bool roll = trick_state == TRICK_ROLL;
    const gyro_axis_t axis = roll ? GYRO_AXIS_ROLL : GYRO_AXIS_PITCH;
    trick_rotation += gyro_rpy[axis] * dt;

    const float end_rad = roll ? trick_roll_end : trick_loop_end;
    const uint32_t timeout_ms = roll ? trick_roll_timeout_ms : trick_loop_timeout_ms;
    if (fabsf(trick_rotation) >= end_rad || (now_ms - trick_start_ms) > timeout_ms)
    {
        endTrick();
        return false;
    }

    // Fly the trick: the stick is ignored on roll and pitch, the trick axis gets a fixed
    // deflection and the other axes keep the rate (damping) corrections.
    ignore_input[GYRO_AXIS_ROLL] = ignore_input[GYRO_AXIS_PITCH] = true;
    corr[axis] = trick_direction * (roll ? trick_roll_deflection : trick_loop_deflection);

    pid_angle_roll.reset();
    pid_angle_pitch.reset();
    return true;
}

void LevelController::calculate_pid(float input_rpy[], float gyro_rpy[], float ang_rpy[])
{
    RateController::calculate_pid(input_rpy, gyro_rpy, ang_rpy);

    if (updateTrick(input_rpy, gyro_rpy))
        return;

    // In Level mode, the Gyro has full control of [Pitch, Roll], not the command
    ignore_input[GYRO_AXIS_ROLL] = ignore_input[GYRO_AXIS_PITCH] = true;

    // Get Pitch/Roll angles adjusted to the Level trims  (global + current settings)
    float pitch_angle = -ang_rpy[GYRO_AXIS_PITCH] + degToRad(gyro_trim_decode(fm_settings.val.trimPitch));
    const float roll_angle = ang_rpy[GYRO_AXIS_ROLL] + degToRad(gyro_trim_decode(fm_settings.val.trimRoll));

    if (isInverted(ang_rpy))
    {
        // The pitch seems to be reported the same even when it is inverted in the roll axis
        pitch_angle = -pitch_angle; // reverse pitch
    }

    // Angle Demand
    // The stick tell the percentage of the max angle where we want the plane to be
    const float setpoint_pitch = input_rpy[GYRO_AXIS_PITCH] * degToRad(fm_angle_settings.val.maxAnglePitch);
    const float setpoint_roll = input_rpy[GYRO_AXIS_ROLL] * degToRad(fm_angle_settings.val.maxAngleRoll);

    pid_angle_pitch.calculate(setpoint_pitch, pitch_angle);
    pid_angle_roll.calculate(setpoint_roll, roll_angle);

    if (isInverted(ang_rpy))
    {
        pid_angle_pitch.reset();               // don't apply elevator corrections if inverted
        ignore_input[GYRO_AXIS_PITCH] = false; // But let the stick control it.
    }

    if (isHighPitch(ang_rpy))
    {
        pid_angle_roll.reset();               // Roll does not work that well in high pitch angles (80 deg)
        ignore_input[GYRO_AXIS_ROLL] = false; // Allow the stick to command roll
    }

    // Add angle correction to rate corrections adjusted to angle Gains
    corr[GYRO_AXIS_ROLL] += pid_angle_roll.output;
    corr[GYRO_AXIS_PITCH] += pid_angle_pitch.output;
}

#if defined(DEBUG_LOG)
void LevelController::printState()
{
    RateController::printState();
    DBGLN("IsInverted = %d,  IsHighPitch=%d", isInverted(gyro.ahrs->angle_rpy), isHighPitch(gyro.ahrs->angle_rpy));
    DBGLN("IgnoreCmd:  Roll:%d Pitch:%d ", ignore_input[GYRO_AXIS_ROLL], ignore_input[GYRO_AXIS_PITCH]);
    DBGLN("Ang Corr:   Roll:%f Pitch:%f", pid_angle_roll.output, pid_angle_pitch.output);
    DBGLN("Trick:      State:%d Rotation:%f deg", trick_state, radToDeg(trick_rotation));
}
#endif // DEBUG_LOG

#endif
