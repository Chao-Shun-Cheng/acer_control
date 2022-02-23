/*
 *  Copyright (c) 2017, Acer Inc.
 *  All rights reserved.
 */


#include "vehicle.h"

static double accel_diff_sum = 0;
static double brake_diff_sum = 0;
double cycle_time = 0.0;


static void clear_diff()
{
    accel_diff_sum = 0;
    brake_diff_sum = 0;
}


double _accel_stroke_pid_control(double current_velocity, double cmd_velocity)
{
    double e;
    static double e_prev = 0;
    double e_i;
    double e_d;
    double ret;
    static double meas_prev = 0.0;
    static double diff_prev = 0.0;
    double prop, integ, diff;
    double lim_max_integ, lim_min_integ;

    // acclerate by releasing the brake pedal if pressed.
    if (v_info.brake_stroke > v_config._BRAKE_PEDAL_OFFSET || v_info.control_mode != 1 /*auto pilot mode*/) {
        /*
        double target_brake_stroke = v_info.brake_stroke - _BRAKE_RELEASE_STEP;
        if (target_brake_stroke < 0)
          target_brake_stroke = 0;
        ret = -target_brake_stroke; // if ret is negative, brake will be applied.
        */

        // v_info has some delay until applying the current state.
        // perhaps we can just return 0 (release brake pedal) here to avoid acceleration delay.
        ret = 0;

        /* reset PID variables. */
        e_prev = 0.0;
        meas_prev = 0.0;
        diff_prev = 0.0;
        clear_diff();
    } else {  // PID control
        double target_accel_stroke;

        e = cmd_velocity - current_velocity;

        // e_d = e - e_prev;

        // anti wind-up
        if ((accel_diff_sum == 0) && (current_velocity > 2))
            accel_diff_sum = cmd_velocity * v_config._K_ACCEL_I_GAIN;

        // if (fabs(e) >= 0.75)
        if (e >= 0.75)
            accel_diff_sum += e;
        else if (e <= 0.0)
            accel_diff_sum += 1.5 * e;

        if (accel_diff_sum > v_config._ACCEL_MAX_I) {
            e_i = v_config._ACCEL_MAX_I;
        } else {
            e_i = accel_diff_sum;
        }

#if 1
        // target_accel_stroke = _K_ACCEL_P * e + _K_ACCEL_I * e_i + _K_ACCEL_D * e_d;
        if (cmd_velocity > 10 /***10**20160905***/) {
            // Proportional Term
            prop = v_config._K_ACCEL_P_UNTIL20 * e;
            // Integral Term
            integ += 0.5 * v_config._K_ACCEL_I_UNTIL20 * v_config._T_sample * e_i;
            // Derivative term with LPF
            diff = (2.0 * v_config._K_ACCEL_D_UNTIL20 * (current_velocity - meas_prev) + (2.0 * v_config._tau_lpf - v_config._T_sample) * diff_prev) / (2.0 * v_config._tau_lpf + v_config._T_sample);
            
            // target_accel_stroke =
            //     v_config._K_ACCEL_P_UNTIL20 * e + v_config._K_ACCEL_I_UNTIL20 * e_i + v_config._K_ACCEL_D_UNTIL20 * e_d + v_config._K_ACCEL_OFFSET;
        } else {
            // Proportional Term
            prop = v_config._K_ACCEL_P_UNTIL10 * e;
            // Integral Term
            integ += 0.5 * v_config._K_ACCEL_I_UNTIL10 * v_config._T_sample * e_i;
            // Derivative term with LPF
            diff = (2.0 * v_config._K_ACCEL_D_UNTIL10 * (current_velocity - meas_prev) + (2.0 * v_config._tau_lpf - v_config._T_sample) * diff_prev) / (2.0 * v_config._tau_lpf + v_config._T_sample);

            // target_accel_stroke =
            //     v_config._K_ACCEL_P_UNTIL10 * e + v_config._K_ACCEL_I_UNTIL10 * e_i + v_config._K_ACCEL_D_UNTIL10 * e_d + v_config._K_ACCEL_OFFSET;
        }
       
        target_accel_stroke = prop + integ + diff + v_config._K_ACCEL_OFFSET;
#else
        printf("accel_p = %lf, accel_i = %lf, accel_d = %lf\n", shm_ptr->accel.P, shm_ptr->accel.I, shm_ptr->accel.D);
        target_accel_stroke = shm_ptr->accel.P * e + shm_ptr->accel.I * e_i + shm_ptr->accel.D * e_d;
#endif

        if (target_accel_stroke > v_config._ACCEL_PEDAL_MAX) {
            target_accel_stroke = v_config._ACCEL_PEDAL_MAX;
        } else if (target_accel_stroke < v_config._K_ACCEL_OFFSET) {
            target_accel_stroke = v_config._K_ACCEL_OFFSET;
        }
        // else if (target_accel_stroke < 0) {
        //  target_accel_stroke = 0;
        //}

        // cout << "e = " << e << endl;
        // cout << "e_i = " << e_i << endl;
        // cout << "e_d = " << e_d << endl;

        ret = target_accel_stroke;
        // Store for Later Use
        meas_prev = current_velocity;
        diff_prev = diff;
        e_prev = e;

#if 1 /* log */
        // ofstream ofs("/tmp/drv_accel.log", ios::app);
        ofstream ofs("/tmp/drv.log", ios::app);
        ofs << "accel:" << cmd_velocity << " " << current_velocity << " " << e << " " << e_i << " " << e_d << " " << target_accel_stroke << " "
            << endl;
#endif
    }

    return ret;
}


double _brake_stroke_pid_control(double current_velocity, double cmd_velocity)
{
    double e;
    static double e_prev = 0;
    static int brake_diff_index = 0;

    static double brake_diff_array[MAX_K_BRAKE_I_CYCLES] = {0};
    double e_i;
    double e_d;
    double ret;

    static double meas_prev = 0.0;
    static double diff_prev = 0.0;
    double prop, integ, diff;
    double lim_max_integ, lim_min_integ;

    // decelerate by releasing the accel pedal if pressed.
    if (v_info.accel_stroke > v_config._ACCEL_PEDAL_OFFSET) {
        // v_info has some delay until applying the current state.
        // perhaps we can just return 0 (release accel pedal) here to avoid deceleration delay.
        ret = 0;

        /* reset PID variables. */
        e_prev = 0.0;
        meas_prev = 0.0;
        diff_prev = 0.0;

        for (brake_diff_index = 0; brake_diff_index < v_config._K_BRAKE_I_CYCLES; brake_diff_index++) {
            brake_diff_array[brake_diff_index] = 0;
        }
        brake_diff_index = 0;
        clear_diff();
    } else {  // PID control
        double target_brake_stroke;

        // since this is braking, multiply -1.
        e = -1 * (cmd_velocity - current_velocity);
        if (e > 0 && e <= 1) {  // added @ 2016/Aug/29
            e = 0;
        }

        //e_d = e - e_prev;

        brake_diff_array[brake_diff_index++] = e;
        brake_diff_index %= v_config._K_BRAKE_I_CYCLES;
        brake_diff_sum = 0;
        for (int i = 0; i < v_config._K_BRAKE_I_CYCLES; i++) {
            brake_diff_sum += brake_diff_array[i];
        }


        if (brake_diff_sum > v_config._BRAKE_MAX_I) {
            e_i = v_config._BRAKE_MAX_I;
        } else {
            e_i = brake_diff_sum;
        }

        // Proportional Term
        prop = v_config._K_BRAKE_P * e;
        // Integral Term
        integ += 0.5 * v_config._K_BRAKE_I * v_config._T_sample * e_i;
        // Derivative term with LPF
        diff = (2.0 * v_config._K_BRAKE_D * (current_velocity - meas_prev) + (2.0 * v_config._tau_lpf - v_config._T_sample) * diff_prev) / (2.0 * v_config._tau_lpf + v_config._T_sample);
        
        // Anti-wind-up using Integrator Clamping
        if (v_config._BRAKE_PEDAL_MAX > prop)
            lim_max_integ = v_config._BRAKE_PEDAL_MAX - prop;
        else
            lim_max_integ = 0.0;

        if (0.0 < prop)
            lim_max_integ = -prop;
        else
            lim_min_integ = 0.0;
        
        // Constrain Integrator
        if (integ > lim_max_integ)
            integ = lim_max_integ;

        else if (integ < lim_min_integ)
            integ = lim_min_integ;
        
        target_brake_stroke = prop + integ + diff;

        // target_brake_stroke = v_config._K_BRAKE_P * e + v_config._K_BRAKE_I * e_i + v_config._K_BRAKE_D * e_d;

        if (target_brake_stroke > v_config._BRAKE_PEDAL_MAX) {
            target_brake_stroke = v_config._BRAKE_PEDAL_MAX;
        } else if (target_brake_stroke < 0) {
            target_brake_stroke = 0;
        }

        cout << "target: " << target_brake_stroke << endl;
        cout << "v_info: " << v_info.brake_stroke << endl;


        // cout << "e = " << e << endl;
        // cout << "e_i = " << e_i << endl;
        // cout << "e_d = " << e_d << endl;

        ret = target_brake_stroke;
        // Store for Later Use
        meas_prev = current_velocity;
        diff_prev = diff;
        e_prev = e;

#if 1 /* log */
        // ofstream ofs("/tmp/drv_brake.log", ios::app);
        ofstream ofs("/tmp/drv.log", ios::app);
        ofs << "de-accel:" << cmd_velocity << " " << current_velocity << " " << e << " " << e_i << " " << e_d << " " << target_brake_stroke << " "
            << v_info.brake_stroke << " " << v_info.accel_stroke << " " << e_prev << " "

            << endl;
#endif
    }

    return ret;
}


double _stopping_control(double current_velocity)
{
    double ret;
    static double old_brake_stroke = v_config._BRAKE_PEDAL_STOPPING_MED;
    int gain;

    // decelerate by using brake
    // if (current_velocity < 0.1) {
    if (current_velocity < 0.05) {  // acer: 0.1 -> 0.5
        // nearly at stop -> apply full brake. brake_stroke should reach BRAKE_PEDAL_MAX in one second.
        if (cycle_time <= 0.0)  // FIXME
            cycle_time = CAN_CMD_INTERVAL / 1000;
        gain = (int) (((double) v_config._BRAKE_PEDAL_STOPPING_MAX) * cycle_time);  // FIXME
        ret = old_brake_stroke + gain;
        if ((int) ret > v_config._BRAKE_PEDAL_STOPPING_MAX)
            ret = v_config._BRAKE_PEDAL_STOPPING_MAX;
        old_brake_stroke = ret;
    } else {
        // very low speed
        ret = v_config._BRAKE_PEDAL_STOPPING_MED;
        old_brake_stroke = ret;
    }

    return ret;
}

void set_drv_stroke(double accel_stroke)
{
    static uint8_t msg[1];
    // X-GENE
    if (v_config.vendor_ID == VENDOR_ID_XGENE) {
        xgeneCan_cmd.accel_stroke = accel_stroke;
        return;
    }

    // Acer Golf
    if (accel_stroke < 0)
        msg[0] = 0;
    else if (accel_stroke > 255)
        msg[0] = 255;
    else
        msg[0] = (uint8_t) accel_stroke;

    canbus_write(CAN_ID_CMD_ACCEL, (char *) msg, sizeof(msg));
}


void set_brake_stroke(double brake_stroke)
{
    static uint8_t msg[1];
    // X-GENE
    if (v_config.vendor_ID == VENDOR_ID_XGENE) {
        xgeneCan_cmd.brake_stroke = brake_stroke;
        return;
    }

    // Acer Golf
    if (brake_stroke < 0)
        msg[0] = 0;
    else if (brake_stroke > 255)
        msg[0] = 255;
    else
        msg[0] = (uint8_t) brake_stroke;

    canbus_write(CAN_ID_CMD_BRAKE, (char *) msg, sizeof(msg));
}

void PedalControl(double current_velocity, double cmd_velocity)
{
    // static uint8_t vel_buffer_size = 10;
    // double old_velocity = 0.0;
    double accel_stroke = 0.0;
    double brake_stroke = 0.0;

    if (cmd_velocity > v_config.SPEED_LIMIT)
        cmd_velocity = v_config.SPEED_LIMIT;

    if ((fabs(cmd_velocity) + 1.5) >= current_velocity && (cmd_velocity != 0.0)) {
        cout << "accelerate: current_velocity=" << current_velocity << ", cmd_velocity=" << cmd_velocity << endl;
        accel_stroke = _accel_stroke_pid_control(current_velocity, fabs(cmd_velocity));
        if (accel_stroke > 0) {
            cout << "SET_DRV_STROKE(" << accel_stroke << ")" << endl;
            set_brake_stroke(0);
            set_drv_stroke(accel_stroke);
        } else {
            cout << "SET_DRV_STROKE(0)" << endl;
            set_drv_stroke(0);
            cout << "SET_BRAKE_STROKE(" << -accel_stroke << ")" << endl;
            set_brake_stroke(-accel_stroke);
        }
    } else if ((fabs(cmd_velocity) + 1.5) < current_velocity && fabs(cmd_velocity) > 0.0) {
        cout << "decelerate: current_velocity=" << current_velocity << ", cmd_velocity=" << cmd_velocity << endl;
        brake_stroke = _brake_stroke_pid_control(current_velocity, fabs(cmd_velocity));
        cout << "brake_stroke===" << brake_stroke << ")" << endl;
        if (brake_stroke > 0) {
            cout << "SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
            set_brake_stroke(brake_stroke);
            set_drv_stroke(0);
        } else {
            cout << "SET_BRAKE_STROKE(0)" << endl;
            set_brake_stroke(0);
            cout << "SET_DRV_STROKE(" << -brake_stroke + (v_config._K_ACCEL_OFFSET) << ")" << endl;
            if (v_config.vendor_ID == VENDOR_ID_XGENE)
                set_drv_stroke(-brake_stroke + (v_config._K_ACCEL_OFFSET) /*workaround for xgene car*/);
            else
                set_drv_stroke(-brake_stroke);
        }
    } else if (cmd_velocity == 0.0 && current_velocity != 0.0) {
        cout << "stopping: current_velocity=" << current_velocity << ", cmd_velocity=" << cmd_velocity << endl;
        if (current_velocity < 4.0) {  // nearly stopping
            set_drv_stroke(0);
            brake_stroke = _stopping_control(current_velocity);
            cout << "SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
            set_brake_stroke(brake_stroke);
        } else {
            brake_stroke = _brake_stroke_pid_control(current_velocity, 0);
            if (brake_stroke > 0) {
                cout << "SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
                set_brake_stroke(brake_stroke);
                set_drv_stroke(0);
            } else {
                cout << "SET_DRV_STROKE(0)" << endl;
                set_drv_stroke(0);
                cout << "SET_DRV_STROKE(" << -brake_stroke << ")" << endl;
                set_drv_stroke(-brake_stroke);
            }
        }
    } else if (cmd_velocity == current_velocity) {
        // Acer add
        // keep DRV_STROKE or BRAKE_STROKE
        if (cmd_velocity == 0.0) {
            brake_stroke = _stopping_control(current_velocity);
            set_brake_stroke(brake_stroke);
            set_drv_stroke(0);
            // keep BRAKE_STROKE
        } else {
            // keep drv_stroke
            // get_vInfo_brake_stroke(); //?
            accel_stroke = get_vInfo_accel_stroke();
            set_brake_stroke(0);
            set_drv_stroke(accel_stroke);
        }
    } else {
        cout << "unknown: current_velocity=" << current_velocity << ", cmd_velocity=" << cmd_velocity
             << ", v_config.SPEED_LIMIT=" << v_config.SPEED_LIMIT << endl;
    }

#if 1 /* log */
    ofstream ofs("/tmp/driving.log", ios::app);
    ofs << cmd_velocity << " " << current_velocity << " " << endl;
#endif
}
