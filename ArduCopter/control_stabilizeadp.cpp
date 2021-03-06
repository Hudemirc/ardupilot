#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::stabilizeadp_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);
    attitude_control->initialize_gmatrix();

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::stabilizeadp_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);
    
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    attitude_control->set_motorConstantProjection(g.motorConstantProjection1);
    attitude_control->set_p_gainadp(g.p_gain_adp1/1000.0f);
    attitude_control->set_beta_gainadp(g.beta_gain_adp1/1000.0f);
    attitude_control->set_q_gainadp(g.q_gain_adp1);
    attitude_control->set_kpkdratioadp(g.kpkdratioadp1);
    attitude_control->set_reset_adp(g.adp_resetting);
    attitude_control->set_reset_adp_sin(g.adpsin_res);
    attitude_control->set_gmatrixadp1(g.gmatrixadp1,g.gmatrixadp2,g.gmatrixadp3,g.gmatrixadp4,g.gmatrixadp5,g.gmatrixadp6,0.0f,0.0f,0.0f,0.0f);
    attitude_control->set_lmatrixadp(g.lmatrixadp);
    attitude_control->set_gmatrix();
    attitude_control->set_lmatrix();
    

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
