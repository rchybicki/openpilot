#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2790427624331697182);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4270927420831008952);
void pose_H_mod_fun(double *state, double *out_586936508799435637);
void pose_f_fun(double *state, double dt, double *out_6348406745166125280);
void pose_F_fun(double *state, double dt, double *out_2025743393358252300);
void pose_h_4(double *state, double *unused, double *out_2320996590281528671);
void pose_H_4(double *state, double *unused, double *out_8426047911114248855);
void pose_h_10(double *state, double *unused, double *out_1276261552857593432);
void pose_H_10(double *state, double *unused, double *out_3386729453987378675);
void pose_h_13(double *state, double *unused, double *out_1056904525204352590);
void pose_H_13(double *state, double *unused, double *out_6808422337262969960);
void pose_h_14(double *state, double *unused, double *out_4613729524718250821);
void pose_H_14(double *state, double *unused, double *out_5343259478818876559);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}