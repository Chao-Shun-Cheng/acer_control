/*
 *  Copyright (c) 2017, Acer Inc.
 *  All rights reserved.
 */


#include "vehicle.h"

static double accel_diff_sum_vc = 0;
static double accel_diff_sum_sc = 0;
static double brake_diff_sum_vc = 0;
static double brake_diff_sum_sc = 0;
double cycle_time = 0.0;


static void clear_diff()
{
    accel_diff_sum_vc = 0;
    accel_diff_sum_sc = 0;
    brake_diff_sum_vc = 0;
    brake_diff_sum_sc = 0;
}


double _accel_stroke_pid_control(double current_velocity, double cmd_velocity, double cmd_spacing, double current_spacing, double pred_out)
{
    double e, e_vc, e_sc;
    static double e_prev_vc = 0.0;
    static double e_prev_sc = 0.0;
    static double e_prev = 0.0;
    double e_i, e_i_vc, e_i_sc;
    double e_d, e_d_vc, e_d_sc;
    double ret;

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
        e_prev_vc = 0;
        e_prev_sc = 0;
        e_prev = 0;
        clear_diff();
    } else {  // PID control
        double target_accel_stroke;

        // true: active ; false: do Not active
        bool cacc_mode = true;

        // ################# Velocity Controller #################
        // Error current velocity
        e_vc = cmd_velocity - current_velocity;
        // Error derivative velocity
        e_d_vc = e_vc - e_prev_vc;
        // Error Integrator velocity
        if ((accel_diff_sum_vc == 0) && (current_velocity > 2))
            accel_diff_sum_vc = cmd_velocity * v_config._K_ACCEL_I_GAIN;
        if (e_vc >= 0.75)
            accel_diff_sum_vc += e_vc;
        else if (e_vc <= 0.0)
            accel_diff_sum_vc += 1.5 * e_vc;

        if (accel_diff_sum_vc > v_config._ACCEL_MAX_I) {
            e_i_vc = v_config._ACCEL_MAX_I;
        } else {
            e_i_vc = accel_diff_sum_vc;
        }

        // Velocity PID action
        double Prop_vc = v_config._K_ACCEL_P_UNTIL10 * e_vc;
        double Integ_vc = v_config._K_ACCEL_I_UNTIL10 * e_i_vc;
        double Diff_vc = v_config._K_ACCEL_D_UNTIL10 * e_d_vc;

        double PID_vc = Prop_vc + Integ_vc + Diff_vc;

        e_prev_vc = e_vc;

        // ################# Spacing Controller #################
        // Error current spacing
        e_sc = cmd_spacing - current_spacing;
        // Error derivative spacing
        e_d_sc = e_sc - e_prev_sc;
        // Error Integrator spacing
        if ((accel_diff_sum_sc == 0) && (current_spacing > 2))
            accel_diff_sum_sc = cmd_spacing * v_config._K_ACCEL_I_GAIN;
        if (e_sc >= 0.75)
            accel_diff_sum_sc += e_sc;
        else if (e_sc <= 0.0)
            accel_diff_sum_sc += 1.5 * e_sc;

        if (accel_diff_sum_sc > v_config._ACCEL_MAX_I) {
            e_i_sc = v_config._ACCEL_MAX_I;
        } else {
            e_i_sc = accel_diff_sum_sc;
        }

        // Spacing PID action
        if (-e_sc >= 50.0)
        {
            e_sc = -50;
        }
            
        double Prop_sc = -v_config._K_ACCEL_P_UNTIL10 * e_sc;
        double Integ_sc = -v_config._K_ACCEL_I_UNTIL10 * e_i_sc;
        double Diff_sc = -v_config._K_ACCEL_D_UNTIL10 * e_d_sc;

        double PID_sc = Prop_sc + Integ_sc + Diff_sc;

        e_prev_sc = e_sc;

        // ################# Determine the Mode #################
        double lim_max_integ = 0.0;
        double lim_min_integ = 0.0;
        if (PID_sc <= PID_vc)
        { // Spacing Controller

            // Anti-wind-up via Dynamic Integrator Clamping Spacing
            if (v_config._ACCEL_PEDAL_MAX > Prop_sc)
                lim_max_integ = v_config._ACCEL_PEDAL_MAX - Prop_sc;
            else
                lim_max_integ = 0.0;

            if (v_config._K_ACCEL_OFFSET < Prop_sc)
                lim_min_integ = v_config._K_ACCEL_OFFSET - Prop_sc;
            else
                lim_min_integ = 0.0;
            
            // Constraint Integrator Spacing
            if (Integ_sc > lim_max_integ)
                Integ_sc = lim_max_integ;
            else if (Integ_sc < lim_min_integ)
                Integ_sc = lim_min_integ;

            target_accel_stroke = Prop_sc + Integ_sc + Diff_sc;// + v_config._K_ACCEL_OFFSET;

            if ((current_velocity * 2.6) < 1.0)
            {
                e_i_sc = 0.0;
                accel_diff_sum_sc = 0.0;
                e_prev_sc = 0.0;
            }

            // Clear other mode values
            accel_diff_sum_vc = 0;
            e_prev_vc = 0;
        }
        else
        { // Velocity Controller

            // Anti-wind-up via Dynamic Integrator Clamping Velocity
            if (v_config._ACCEL_PEDAL_MAX > Prop_vc)
                lim_max_integ = v_config._ACCEL_PEDAL_MAX - Prop_vc;
            else
                lim_max_integ = 0.0;

            if (v_config._K_ACCEL_OFFSET < Prop_vc)
                lim_min_integ = v_config._K_ACCEL_OFFSET - Prop_vc;
            else
                lim_min_integ = 0.0;
            
            // Constraint Integrator Velocity
            if (Integ_vc > lim_max_integ)
                Integ_vc = lim_max_integ;
            else if (Integ_vc < lim_min_integ)
                Integ_vc = lim_min_integ;

            target_accel_stroke = Prop_vc + Integ_vc + Diff_vc + v_config._K_ACCEL_OFFSET;

            // Clear other mode values
            accel_diff_sum_sc = 0;
            e_prev_sc = 0;
        }     

        // CACC Mode
        if (cacc_mode)
        {
            target_accel_stroke += pred_out;
        }
        

        // // if (fabs(e) >= 0.75)
        

#if 1
        // target_accel_stroke = _K_ACCEL_P * e + _K_ACCEL_I * e_i + _K_ACCEL_D * e_d;
        // if (cmd_velocity > 10 /***10**20160905***/) {
        //     target_accel_stroke =
        //         v_config._K_ACCEL_P_UNTIL20 * e_vc + v_config._K_ACCEL_I_UNTIL20 * e_i_vc + v_config._K_ACCEL_D_UNTIL20 * e_d_vc + v_config._K_ACCEL_OFFSET;
        // } else {
        //     target_accel_stroke =
        //         v_config._K_ACCEL_P_UNTIL10 * e_vc + v_config._K_ACCEL_I_UNTIL10 * e_i_vc + v_config._K_ACCEL_D_UNTIL10 * e_d_vc + v_config._K_ACCEL_OFFSET;
        // }
        
#else
        printf("accel_p = %lf, accel_i = %lf, accel_d = %lf\n", shm_ptr->accel.P, shm_ptr->accel.I, shm_ptr->accel.D);
        target_accel_stroke = shm_ptr->accel.P * e_vc + shm_ptr->accel.I * e_i_vc + shm_ptr->accel.D * e_d_vc;
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
        

#if 1 /* log */
        // ofstream ofs("/tmp/drv_accel.log", ios::app);
        ofstream ofs("/tmp/drv.log", ios::app);
        ofs << "accel:" << cmd_velocity << " " << current_velocity << " " << e_vc << " " << e_i_vc << " " << e_d_vc << " " << target_accel_stroke << " "
            << endl;
#endif
    }

    return ret;
}


double _brake_stroke_pid_control(double current_velocity, double cmd_velocity, double cmd_spacing, double current_spacing, double pred_out)
{
    double e, e_vc, e_sc;
    static double e_prev_vc = 0;
    static double e_prev_sc = 0;
    static double e_prev = 0;
    static int brake_diff_index_vc = 0;
    static int brake_diff_index_sc = 0;

    static double brake_diff_array_vc[MAX_K_BRAKE_I_CYCLES] = {0};
    static double brake_diff_array_sc[MAX_K_BRAKE_I_CYCLES] = {0};
    double e_i, e_i_vc, e_i_sc;
    double e_d, e_d_vc, e_d_sc;
    double ret;

    // decelerate by releasing the accel pedal if pressed.
    if (v_info.accel_stroke > v_config._ACCEL_PEDAL_OFFSET) {
        // v_info has some delay until applying the current state.
        // perhaps we can just return 0 (release accel pedal) here to avoid deceleration delay.
        ret = 0;

        /* reset PID variables. */
        e_prev_vc = 0;
        e_prev_sc = 0;
        e_prev = 0;

        for (brake_diff_index_vc = 0; brake_diff_index_vc < v_config._K_BRAKE_I_CYCLES; brake_diff_index_vc++) {
            brake_diff_array_vc[brake_diff_index_vc] = 0;
        }
        brake_diff_index_vc = 0;

        for (brake_diff_index_sc = 0; brake_diff_index_sc < v_config._K_BRAKE_I_CYCLES; brake_diff_index_sc++) {
            brake_diff_array_sc[brake_diff_index_sc] = 0;
        }
        brake_diff_index_sc = 0;
        
        clear_diff();
    } else {  // PID control
        double target_brake_stroke;

        // true: active; false: do Not active
        bool cacc_mode = true;

        // ################# Velocity Controller #################
        // Error current velocity
        // since this is braking, multiply -1.
        e_vc = -1 * (cmd_velocity - current_velocity);
        if (e_vc > 0 && e_vc <= 1) {  // added @ 2016/Aug/29
            e_vc = 0;
        }
        // Error Derivative velocity
        e_d_vc = e_vc - e_prev_vc;
        // Error Integrator velocity
        brake_diff_array_vc[brake_diff_index_vc++] = e_vc;
        brake_diff_index_vc %= v_config._K_BRAKE_I_CYCLES;
        brake_diff_sum_vc = 0;
        for (int i = 0; i < v_config._K_BRAKE_I_CYCLES; i++) {
            brake_diff_sum_vc += brake_diff_array_vc[i];
        }
        if (brake_diff_sum_vc > v_config._BRAKE_MAX_I) {
            e_i_vc = v_config._BRAKE_MAX_I;
        } else {
            e_i_vc = brake_diff_sum_vc;
        }

        // Velocity PID action
        double Prop_vc = v_config._K_BRAKE_P * e_vc;
        double Integ_vc = v_config._K_BRAKE_I * e_i_vc;
        double Diff_vc = v_config._K_BRAKE_D * e_d_vc;

        double PID_vc = Prop_vc + Integ_vc + Diff_vc;

        e_prev_vc = e_vc;

        // ################# Spacing Controller #################
        // Error current Spacing
        e_sc = -1*(cmd_spacing - current_spacing);
        // if (e_sc > 0 && e_sc <= 1) {  // added @ 2016/Aug/29
        //     e_sc = 0;
        // }
        // Error Derivative Spacing
        e_d_sc = e_sc - e_prev_sc;
        brake_diff_array_sc[brake_diff_index_sc++] = e_sc;
        brake_diff_index_sc %= v_config._K_BRAKE_I_CYCLES;
        brake_diff_sum_sc = 0;
        for (int i = 0; i < v_config._K_BRAKE_I_CYCLES; i++) {
            brake_diff_sum_sc += brake_diff_array_sc[i];
        }
        if (brake_diff_sum_sc > v_config._BRAKE_MAX_I) {
            e_i_sc = v_config._BRAKE_MAX_I;
        } else {
            e_i_sc = brake_diff_sum_sc;
        }

        // Spacing PID action
        if (-e_sc >= 50.0)
        {
            e_sc = -50;
        }
        double Prop_sc = -v_config._K_BRAKE_P * e_sc;
        double Integ_sc = -v_config._K_BRAKE_I * e_i_sc;
        double Diff_sc = -v_config._K_BRAKE_D * e_d_sc;

        double PID_sc = Prop_sc + Integ_sc + Diff_sc;

        e_prev_sc = e_sc;

        // ################# Determine the Mode #################
        double lim_max_integ = 0.0;
        double lim_min_integ = 0.0;
        if (PID_sc <= PID_vc)
        {// Spacing Controller

            // Anti-wind-up via Dynamic Integrator Clamping Velocity
            if (v_config._BRAKE_PEDAL_MAX > Prop_sc)
                lim_max_integ = v_config._BRAKE_PEDAL_MAX - Prop_sc;
            else
                lim_max_integ = 0.0;

            if (0 < Prop_sc)
                lim_min_integ = -Prop_sc;
            else
                lim_min_integ = 0.0;
            
            // Constraint Integrator Velocity
            if (Integ_sc > lim_max_integ)
                Integ_sc = lim_max_integ;
            else if (Integ_sc < lim_min_integ)
                Integ_sc = lim_min_integ;

            target_brake_stroke = Prop_sc + Integ_sc + Diff_sc + v_config._BRAKE_PEDAL_OFFSET;

            if ((current_velocity * 2.6) < 1)
            {
                e_i_sc = 0.0;
                e_prev_sc = 0.0;
                brake_diff_index_sc = 0.0;
                brake_diff_sum_sc = 0.0;
            }

            // Clear all other modes
            e_i_vc = 0.0;
            e_prev_vc = 0.0;
            brake_diff_index_vc = 0.0;
            brake_diff_sum_vc = 0.0;
        }
        else
        { // Velocity Controller

            // Anti-wind-up via Dynamic Integrator Clamping Velocity
            if (v_config._BRAKE_PEDAL_MAX > Prop_vc)
                lim_max_integ = v_config._BRAKE_PEDAL_MAX - Prop_vc;
            else
                lim_max_integ = 0.0;

            if (0 < Prop_vc)
                lim_min_integ = -Prop_vc;
            else
                lim_min_integ = 0.0;
            
            // Constraint Integrator Velocity
            if (Integ_vc > lim_max_integ)
                Integ_vc = lim_max_integ;
            else if (Integ_vc < lim_min_integ)
                Integ_vc = lim_min_integ;

            target_brake_stroke = Prop_vc + Integ_vc + Diff_vc + v_config._BRAKE_PEDAL_OFFSET;

            // Clear all other modes
            e_i_sc = 0.0;
            e_prev_sc = 0.0;
            brake_diff_index_sc = 0.0;
            brake_diff_sum_sc = 0.0;
        }

        // CACC Mode
        if (cacc_mode)
        {
            target_brake_stroke += pred_out;
        }

        // target_brake_stroke = v_config._K_BRAKE_P * e_vc + v_config._K_BRAKE_I * e_i_vc + v_config._K_BRAKE_D * e_d_vc;
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

        printf("e_sc = %lf\n", e_sc);

#if 1 /* log */
        // ofstream ofs("/tmp/drv_brake.log", ios::app);
        ofstream ofs("/tmp/drv.log", ios::app);
        ofs << "de-accel:" << cmd_velocity << " " << current_velocity << " " << e_vc << " " << e_i_vc << " " << e_d_vc << " " << target_brake_stroke << " "
            << v_info.brake_stroke << " " << v_info.accel_stroke << " " << e_prev_vc << " "

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

void PedalControl(double current_velocity, double cmd_velocity, double current_spacing, double pred_out, double leading_velocity)
{
    // static uint8_t vel_buffer_size = 10;
    // double old_velocity = 0.0;
    double accel_stroke = 0.0;
    double brake_stroke = 0.0;

    double cmd_spacing = 5.0 + (0.8 * (current_velocity * 2.6));
    double error_spacing = cmd_spacing - current_spacing;
    if (cmd_velocity > v_config.SPEED_LIMIT)
        cmd_velocity = v_config.SPEED_LIMIT;

    if ((cmd_velocity == 0.0 && current_velocity != 0.0) || (leading_velocity <= 1.5 && error_spacing > 1.0) ) {
        cout << RED << "stop: current_velocity=" << current_velocity << ", cmd_velocity=" << cmd_velocity << "current_spacing=" << current_spacing << ", cmd_spacing=" << cmd_spacing << ", pred_out=" << pred_out << RESET << endl;
        if ((current_velocity * 2.6) < 4.0) {  // nearly stopping
            set_drv_stroke(0);
            brake_stroke = _stopping_control(current_velocity);
            cout << "SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
            set_brake_stroke(brake_stroke);
        } else {
            brake_stroke = _brake_stroke_pid_control(current_velocity, 0, cmd_spacing, current_spacing, pred_out);
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
    } else if ((fabs(cmd_velocity) + (1.5/2.6)) >= current_velocity && (cmd_velocity != 0.0)) {
        cout << GREEN << "accelerate: current_velocity=" << current_velocity << ", pred_out=" << pred_out << ", error_spacing=" << error_spacing << RESET << endl;

        accel_stroke = _accel_stroke_pid_control(current_velocity, fabs(cmd_velocity), cmd_spacing, current_spacing, pred_out);
        if (accel_stroke > 0) {
            cout << CYAN << "SET_DRV_STROKE(" << accel_stroke << ")" << RESET << endl;
            set_brake_stroke(0);
            set_drv_stroke(accel_stroke);
        } else {
            cout << "SET_DRV_STROKE(0)" << endl;
            set_drv_stroke(0);
            cout << "SET_BRAKE_STROKE(" << -accel_stroke << ")" << endl;
            set_brake_stroke(-accel_stroke);
        }
    } else if ((fabs(cmd_velocity) + (1.5/2.6)) < current_velocity && fabs(cmd_velocity) > 0.0) {
        cout << YELLOW << "decelerate: current_velocity=" << current_velocity << ", pred_out=" << pred_out << ", error_spacing=" << error_spacing << RESET << endl;
        brake_stroke = _brake_stroke_pid_control(current_velocity, fabs(cmd_velocity), cmd_spacing, current_spacing, pred_out);
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
             << ", v_config.SPEED_LIMIT=" << v_config.SPEED_LIMIT << "current_spacing=" << current_spacing <<  ", cmd_spacing=" << cmd_spacing << ", pred_out=" << pred_out << endl;
    }

#if 1 /* log */
    ofstream ofs("/tmp/driving.log", ios::app);
    ofs << cmd_velocity << " " << current_velocity << " " << endl;
#endif
}
