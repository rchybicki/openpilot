#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6098042004296785388);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3068516717447087923);
void car_H_mod_fun(double *state, double *out_5410945619253105879);
void car_f_fun(double *state, double dt, double *out_1004237871095338647);
void car_F_fun(double *state, double dt, double *out_6714649306439139438);
void car_h_25(double *state, double *unused, double *out_4265863487104550406);
void car_H_25(double *state, double *unused, double *out_5114460421482585932);
void car_h_24(double *state, double *unused, double *out_1395221982550150496);
void car_H_24(double *state, double *unused, double *out_1556534822680262796);
void car_h_30(double *state, double *unused, double *out_771257319668328524);
void car_H_30(double *state, double *unused, double *out_2596127462975337305);
void car_h_26(double *state, double *unused, double *out_6231505384124737255);
void car_H_26(double *state, double *unused, double *out_8855963740356642156);
void car_h_27(double *state, double *unused, double *out_2133178030309298740);
void car_H_27(double *state, double *unused, double *out_6629824010298932575);
void car_h_29(double *state, double *unused, double *out_1504271497944723610);
void car_H_29(double *state, double *unused, double *out_9131925407295801946);
void car_h_28(double *state, double *unused, double *out_2264774050972330272);
void car_H_28(double *state, double *unused, double *out_7168295135730475695);
void car_h_31(double *state, double *unused, double *out_628413958850528056);
void car_H_31(double *state, double *unused, double *out_8964572231119557984);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}