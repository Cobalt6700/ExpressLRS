#pragma once
#include "gyro.h"
#include "mode_rate.h"

class LevelController : public RateController
{
public:
    void initialize(gyro_mode_t mode) override;
    void calculate_pid(float input_rpy[], float gyro_rpy[], float ang_rpy[]) override;
    void setTrickButton(bool pressed) override;
#if defined(DEBUG_LOG)
    void printState() override;
#endif

protected:
    rx_config_gyro_fmode_t fm_angle_settings {};
    PID pid_angle_roll, pid_angle_pitch;

    // One button trick (Auto-Level only)
    enum trick_state_t
    {
        TRICK_IDLE,  // Waiting for the trick button
        TRICK_ARMED, // Button pressed, waiting for an aileron/elevator stick input
        TRICK_ROLL,  // Performing an axial roll
        TRICK_LOOP   // Performing a loop
    };

    trick_state_t trick_state = TRICK_IDLE;
    bool trick_button = false;      // Current button state
    bool trick_button_prev = false; // Button state on the previous PID cycle
    float trick_direction = 0.0f;   // +1/-1, the direction of the stick that started the trick
    float trick_rotation = 0.0f;    // Rotation completed so far (radians, signed)
    uint32_t trick_start_ms = 0;    // When the trick was armed / started
    uint32_t trick_last_us = 0;     // Last PID cycle time, used to integrate the gyro rate

    // Tuning, loaded from the gyro config (LUA Trick folder)
    uint32_t trick_arm_ms = 3000;
    float trick_stick_threshold = 0.6f;
    float trick_roll_deflection = 1.0f;
    float trick_loop_deflection = 0.8f;
    float trick_roll_end = 0.0f; // radians
    float trick_loop_end = 0.0f; // radians
    uint32_t trick_roll_timeout_ms = 3000;
    uint32_t trick_loop_timeout_ms = 6000;

    void loadTrickSettings();
    void resetTrick();
    void startTrick(trick_state_t trick, float stick);
    void endTrick();
    bool updateTrick(const float input_rpy[], const float gyro_rpy[]);
};
