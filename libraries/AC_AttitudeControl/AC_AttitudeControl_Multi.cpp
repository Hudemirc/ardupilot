#include "AC_AttitudeControl_Multi.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Multi::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.08 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: Percent
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FILT
    // @DisplayName: Roll axis rate controller input frequency in Hz
    // @Description: Roll axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.08 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: Percent
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FILT
    // @DisplayName: Pitch axis rate controller input frequency in Hz
    // @Description: Pitch axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.10 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 0.05
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: Percent
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FILT
    // @DisplayName: Yaw axis rate controller input frequency in Hz
    // @Description: Yaw axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Multi, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Multi, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Multi, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Multi, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    AP_GROUPEND
};

AC_AttitudeControl_Multi::AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt) :
    AC_AttitudeControl(ahrs, aparm, motors, dt),
    _motors_multi(motors),
    _pid_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_YAW_FILT_HZ, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Multi::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    float althold_lean_angle_max = acos(constrain_float(_throttle_in/(AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt/(_dt+_angle_limit_tc))*(althold_lean_angle_max-_althold_lean_angle_max);
}

void AC_AttitudeControl_Multi::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    }else{
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(2.0f*cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f/constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = throttle_in*inverted_factor*boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in,-1.0f,1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in*MAX(0.0f,1.0f-_throttle_rpy_mix)+_motors.get_throttle_hover()*_throttle_rpy_mix);
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_Multi::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f*_dt, _throttle_rpy_mix_desired-_throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f*_dt, _throttle_rpy_mix-_throttle_rpy_mix_desired);
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void AC_AttitudeControl_Multi::rate_controller_run()
{
    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    _motors.set_roll(rate_target_to_motor_roll(gyro_latest.x, _rate_target_ang_vel.x));
    _motors.set_pitch(rate_target_to_motor_pitch(gyro_latest.y, _rate_target_ang_vel.y));
    _motors.set_yaw(rate_target_to_motor_yaw(gyro_latest.z, _rate_target_ang_vel.z));
    
    control_monitor_update();
}

void AC_AttitudeControl_Multi::rate_controller_runadp()
{
    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();
    float _pid_roll_for_adp;
    float _pid_pitch_for_adp;
    float _pid_yaw_for_adp;
    float _thrust_for_adp;
    float _adp_roll_output;
    float _adp_pitch_output;
    float _adp_yaw_output;
    float _adp_thrust_output;
    float _omega1sq;
    float _omega2sq;
    float _omega3sq;
    float _omega4sq;
    float _motorConstantMatrix[16];
    float _tempterm;
    float _Gksi_roll[no_sinusoidal];
    float _Geta_roll[no_sinusoidal];
    float _oldbeta_roll[no_sinusoidal];
    float _oldeta0_roll[no_sinusoidal];
    float _oldbeta1_roll[no_sinusoidal];
    float _oldeta1_roll[no_sinusoidal];
    float _oldksi_roll[no_sinusoidal];

    Vector3f gyro_latest = _ahrs.get_gyro_latest();
  
    for (int i=0;i<no_sinusoidal;i++){
        _oldeta0_roll[i] = _eta0_roll[i];
        _oldksi_roll[i]  = _ksi_roll[i] ;
        _oldbeta_roll[i] = _beta_roll[i];
        _oldbeta1_roll[i] = _beta1_roll[i];
        _oldeta1_roll[i] = _eta1_roll[i];
    }
    _gyroxdata=gyro_latest.x;
    if (is_zero(_resetting_adp_sin_values)){
        for (int i=0;i<no_sinusoidal;i++){
            for (int j=0;j<no_sinusoidal;j++){
                _Gksi_roll[i]  += _Gmatrix[i*no_sinusoidal+j]*_ksi_roll[j];
                _Geta_roll[i]  += _Gmatrix[i*no_sinusoidal+j]*_eta1_roll[j];
            }
            _eta0_roll[i] = _eta0_roll[i] + _dt*(_Gksi_roll[i]-_Lmatrix[i]*(_motors.get_omega2sqadp()-_motors.get_omega1sqadp() ));
            if(isnan(_eta0_roll[i]) || (fabsf(_eta0_roll[i]) >10.0*10.0)) _eta0_roll[i]=_oldeta0_roll[i];
            _eta1_roll[i] = _eta1_roll[i] + _dt*(_Gksi_roll[i]-_Lmatrix[i]);
            if(isnan(_eta1_roll[i]) || (fabsf(_eta1_roll[i]) >10.0*10.0)) _eta0_roll[i]=_oldeta1_roll[i];
            _ksi_roll[i]  = _eta0_roll[i] + _Lmatrix[i] * _gyroxdata;
            if(isnan(_ksi_roll[i]) || ( fabsf(_ksi_roll[i]) >10.0*10.0)) _ksi_roll[i]=_oldksi_roll[i];
            _beta_roll[i] = _beta_roll[i]  - (-_error_roll_adp[0]/4.5f/_kpkdratioadp + _error_roll_adp[1] /0.0036f*_kpkdratioadp )*_ksi_roll[i] *_beta_gainadp*_dt;
            if(isnan(_beta_roll[i]) || ( fabsf(_beta_roll[i]) >10.0*10.0)) _beta_roll[i]=_oldbeta_roll[i];
            _beta1_roll[i] = _beta1_roll[i]  - (-_error_roll_adp[0]/4.5f/_kpkdratioadp + _error_roll_adp[1] /0.0036f*_kpkdratioadp )*_eta1_roll[i] *_beta_gainadp*_dt;
            if(isnan(_beta1_roll[i]) || ( fabsf(_beta1_roll[i]) >10.0*10.0)) _beta1_roll[i]=_oldbeta1_roll[i];
            _sin_dist_roll =_sin_dist_roll + _beta_roll[i]*_ksi_roll[i] + _beta1_roll[i]*_eta1_roll[i];
        }
    }else  {
        for (int i=0;i<no_sinusoidal;i++){
            _eta0_roll[i] = 0.0f;
            _eta1_roll[i] = 0.0f;
            _ksi_roll[i]  = 0.0f;
            _beta_roll[i] = 0.0f;
            _beta1_roll[i] = 0.0f;
            _sin_dist_roll =0.0f;
        
        }
    }

    _sin_dist_roll = constrain_float(_sin_dist_roll, -.3f, .3f);

    
    _pid_roll_for_adp=rate_target_to_motor_roll(gyro_latest.x, _rate_target_ang_vel.x)-_sin_dist_roll;
    _pid_pitch_for_adp=rate_target_to_motor_pitch(gyro_latest.y, _rate_target_ang_vel.y);
    _pid_yaw_for_adp=rate_target_to_motor_yaw(gyro_latest.z, _rate_target_ang_vel.z);
    _thrust_for_adp=_motors.get_throttle();
    control_monitor_update();

    _rollpidbefore=_pid_roll_for_adp;
    _pitchpidbefore=_pid_pitch_for_adp;
    _yawpidbefore=_pid_yaw_for_adp;
    _thrpidbefore=_thrust_for_adp;

    _tempterm=_p_adp1*_p_adp3*_q_adp2 + _p_adp2*_p_adp3*_q_adp1 + _p_adp1*_p_adp4*_q_adp2 +_p_adp2*_p_adp4*_q_adp1+_p_adp1*_p_adp3*_q_adp4 + _p_adp1*_p_adp4*_q_adp3 + _p_adp2*_p_adp3*_q_adp4 +_p_adp2*_p_adp4*_q_adp3;
    

    _motorConstantMatrix[0]   = -(_p_adp3*_q_adp2 + _p_adp4*_q_adp2 + _p_adp3*_q_adp4 + _p_adp4*_q_adp3)/_tempterm;
    _motorConstantMatrix[1]   =  (_p_adp2*_q_adp3 - _p_adp2*_q_adp4) / _tempterm;
    _motorConstantMatrix[2]   =  (_p_adp2*_p_adp3 + _p_adp2*_p_adp4) / _tempterm;
    _motorConstantMatrix[3]   =  (_p_adp2*_p_adp3*_q_adp4 + _p_adp2*_p_adp4*_q_adp3) /_tempterm;
    _motorConstantMatrix[4]   =  (_p_adp3*_q_adp1 + _p_adp4*_q_adp1 + _p_adp3*_q_adp4 + _p_adp4*_q_adp3)/_tempterm;
    _motorConstantMatrix[5]   =  (_p_adp1*_q_adp3 - _p_adp1*_q_adp4) / _tempterm;
    _motorConstantMatrix[6]   =  (_p_adp1*_p_adp3 + _p_adp1*_p_adp4) / _tempterm;
    _motorConstantMatrix[7]   =  (_p_adp1*_p_adp3*_q_adp4 + _p_adp1*_p_adp4*_q_adp3) / _tempterm;
    _motorConstantMatrix[8]   = -(_p_adp4*_q_adp1 - _p_adp4*_q_adp2) / _tempterm;
    _motorConstantMatrix[9]   =  (_p_adp1*_q_adp2 + _p_adp2*_q_adp1 + _p_adp1*_q_adp4 + _p_adp2*_q_adp4)/_tempterm;
    _motorConstantMatrix[10]  = -(_p_adp4*_p_adp1 + _p_adp4*_p_adp2) / _tempterm;
    _motorConstantMatrix[11]  =  (_p_adp4*_p_adp1*_q_adp2 + _p_adp4*_p_adp2*_q_adp1) /_tempterm;
    _motorConstantMatrix[12]  = -(_p_adp3*_q_adp1 - _p_adp3*_q_adp2) / _tempterm;
    _motorConstantMatrix[13]  = -(_p_adp1*_q_adp2 + _p_adp2*_q_adp1 + _p_adp1*_q_adp3 + _p_adp2*_q_adp3)/_tempterm;
    _motorConstantMatrix[14]  = -(_p_adp3*_p_adp1 + _p_adp3*_p_adp2 ) / (_tempterm);
    _motorConstantMatrix[15]  =  (_p_adp3*_p_adp1*_q_adp2 + _p_adp3*_p_adp2*_q_adp1) / _tempterm;


    _omega1sq=_motorConstantMatrix[0]  * _pid_roll_for_adp + _motorConstantMatrix[1]  * _pid_pitch_for_adp + _motorConstantMatrix[2]  * _pid_yaw_for_adp + _motorConstantMatrix[3]  * _thrust_for_adp;
    _omega2sq=_motorConstantMatrix[4]  * _pid_roll_for_adp + _motorConstantMatrix[5]  * _pid_pitch_for_adp + _motorConstantMatrix[6]  * _pid_yaw_for_adp + _motorConstantMatrix[7]  * _thrust_for_adp;
    _omega3sq=_motorConstantMatrix[8]  * _pid_roll_for_adp + _motorConstantMatrix[9]  * _pid_pitch_for_adp + _motorConstantMatrix[10] * _pid_yaw_for_adp + _motorConstantMatrix[11] * _thrust_for_adp;
    _omega4sq=_motorConstantMatrix[12] * _pid_roll_for_adp + _motorConstantMatrix[13] * _pid_pitch_for_adp + _motorConstantMatrix[14] * _pid_yaw_for_adp + _motorConstantMatrix[15] * _thrust_for_adp;


    _adp_roll_output   =  (_omega2sq - _omega1sq);
    _adp_pitch_output  =  (_omega3sq - _omega4sq);
    _adp_yaw_output    =  (_omega1sq + _omega2sq - _omega3sq - _omega4sq)/2.0f ;
    _adp_thrust_output =  (_omega1sq + _omega2sq + _omega3sq + _omega4sq) ;

    
    _p_adp1= _p_adp1 - _dt*(-_error_roll_adp[0]/4.5f/_kpkdratioadp + _error_roll_adp[1] /0.0036f*_kpkdratioadp ) * _motors.get_omega1sqadp() * _p_gainadp;
    _p_adp2= _p_adp2 + _dt*(-_error_roll_adp[0]/4.5f/_kpkdratioadp + _error_roll_adp[1] /0.0036f*_kpkdratioadp ) * _motors.get_omega2sqadp() * _p_gainadp;
    _p_adp3= _p_adp3 + _dt*(-_error_pitch_adp[0]/4.5f/_kpkdratioadp + _error_pitch_adp[1] /0.0036f*_kpkdratioadp ) * _motors.get_omega3sqadp() * _p_gainadp;
    _p_adp4= _p_adp4 - _dt*(-_error_pitch_adp[0]/4.5f/_kpkdratioadp + _error_pitch_adp[1] /0.0036f*_kpkdratioadp ) * _motors.get_omega4sqadp() * _p_gainadp;
    _q_adp1= _q_adp1 + _dt*(-_error_yaw_adp[0]/4.5f/_kpkdratioadp + _error_yaw_adp[1] /0.0036f*_kpkdratioadp ) * _motors.get_omega1sqadp() * _q_gainadp;
    _q_adp2= _q_adp2 + _dt*(-_error_yaw_adp[0]/4.5f/_kpkdratioadp + _error_yaw_adp[1] /0.0036f*_kpkdratioadp ) * _motors.get_omega2sqadp() * _q_gainadp;
    _q_adp3= _q_adp3 - _dt*(-_error_yaw_adp[0]/4.5f/_kpkdratioadp + _error_yaw_adp[1] /0.0036f*_kpkdratioadp ) * _motors.get_omega3sqadp() * _q_gainadp;
    _q_adp4= _q_adp4 - _dt*(-_error_yaw_adp[0]/4.5f/_kpkdratioadp + _error_yaw_adp[1] /0.0036f*_kpkdratioadp ) * _motors.get_omega4sqadp() * _q_gainadp;
    
    
    if (_resetting_adp_values>0.95 && _resetting_adp_values<1.05){
        _p_adp1=1.0f;
        _p_adp2=1.0f;
        _p_adp3=1.0f;
        _p_adp4=1.0f;
        _q_adp1=0.5f;
        _q_adp2=0.5f;
        _q_adp3=0.5f;
        _q_adp4=0.5f;
    }

    _p_adp1=constrain_float(_p_adp1, (1.0f-_motorConstantProjection), (1.0f+_motorConstantProjection));
    _p_adp2=constrain_float(_p_adp2, (1.0f-_motorConstantProjection), (1.0f+_motorConstantProjection));
    _p_adp3=constrain_float(_p_adp3, (1.0f-_motorConstantProjection), (1.0f+_motorConstantProjection));
    _p_adp4=constrain_float(_p_adp4, (1.0f-_motorConstantProjection), (1.0f+_motorConstantProjection));
    _q_adp1=constrain_float(_q_adp1, (1.0f-_motorConstantProjection)/2.0f, (1.0f+_motorConstantProjection)/2.0f);
    _q_adp2=constrain_float(_q_adp2, (1.0f-_motorConstantProjection)/2.0f, (1.0f+_motorConstantProjection)/2.0f);
    _q_adp3=constrain_float(_q_adp3, (1.0f-_motorConstantProjection)/2.0f, (1.0f+_motorConstantProjection)/2.0f);
    _q_adp4=constrain_float(_q_adp4, (1.0f-_motorConstantProjection)/2.0f, (1.0f+_motorConstantProjection)/2.0f);

    

    
    
    
   
    _rollpidafter=_adp_roll_output;
    _pitchpidafter=_adp_pitch_output;
    _yawpidafter=_adp_yaw_output;
    _thrpidafter=_adp_thrust_output;

    _motors.set_roll(_adp_roll_output);
    _motors.set_pitch(_adp_pitch_output);
    _motors.set_yaw(_adp_yaw_output);
    _motors.set_throttleadp(_adp_thrust_output);

    
    control_monitor_update();
}

// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_Multi::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > 4.0f) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(AC_ATTITUDE_CONTROL_MAN_DEFAULT);
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > 0.25f) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}
