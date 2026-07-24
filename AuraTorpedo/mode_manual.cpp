#include "Torpedo.h"


bool ModeManual::init(bool ignore_checks) {
    // set target altitude to zero for reporting
    position_control->set_pos_target_z_cm(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    torpedo.set_neutral_controls();

    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void ModeManual::run()
{
    // if not armed set throttle to zero and exit immediately
    if (!torpedo.motors.armed()) {
        torpedo.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    torpedo.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // AURA torpido: lateral eksen anlamsiz (holonomik degil) -> QGC joystick'in
    // lateral cubugu dogrudan PITCH olur (Roll/Pitch Toggle'a gerek kalmaz);
    // toggle'li pitch kanali da ustune eklenmeye devam eder.
    torpedo.motors.set_roll(channel_roll->norm_input());
    torpedo.motors.set_pitch(constrain_float(channel_pitch->norm_input() + channel_lateral->norm_input(), -1.0f, 1.0f));
    torpedo.motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P);
    torpedo.motors.set_throttle(channel_throttle->norm_input());
    torpedo.motors.set_forward(channel_forward->norm_input());
    torpedo.motors.set_lateral(0.0f);
}
