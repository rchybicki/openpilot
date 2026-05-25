#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2790427624331697182) {
   out_2790427624331697182[0] = delta_x[0] + nom_x[0];
   out_2790427624331697182[1] = delta_x[1] + nom_x[1];
   out_2790427624331697182[2] = delta_x[2] + nom_x[2];
   out_2790427624331697182[3] = delta_x[3] + nom_x[3];
   out_2790427624331697182[4] = delta_x[4] + nom_x[4];
   out_2790427624331697182[5] = delta_x[5] + nom_x[5];
   out_2790427624331697182[6] = delta_x[6] + nom_x[6];
   out_2790427624331697182[7] = delta_x[7] + nom_x[7];
   out_2790427624331697182[8] = delta_x[8] + nom_x[8];
   out_2790427624331697182[9] = delta_x[9] + nom_x[9];
   out_2790427624331697182[10] = delta_x[10] + nom_x[10];
   out_2790427624331697182[11] = delta_x[11] + nom_x[11];
   out_2790427624331697182[12] = delta_x[12] + nom_x[12];
   out_2790427624331697182[13] = delta_x[13] + nom_x[13];
   out_2790427624331697182[14] = delta_x[14] + nom_x[14];
   out_2790427624331697182[15] = delta_x[15] + nom_x[15];
   out_2790427624331697182[16] = delta_x[16] + nom_x[16];
   out_2790427624331697182[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4270927420831008952) {
   out_4270927420831008952[0] = -nom_x[0] + true_x[0];
   out_4270927420831008952[1] = -nom_x[1] + true_x[1];
   out_4270927420831008952[2] = -nom_x[2] + true_x[2];
   out_4270927420831008952[3] = -nom_x[3] + true_x[3];
   out_4270927420831008952[4] = -nom_x[4] + true_x[4];
   out_4270927420831008952[5] = -nom_x[5] + true_x[5];
   out_4270927420831008952[6] = -nom_x[6] + true_x[6];
   out_4270927420831008952[7] = -nom_x[7] + true_x[7];
   out_4270927420831008952[8] = -nom_x[8] + true_x[8];
   out_4270927420831008952[9] = -nom_x[9] + true_x[9];
   out_4270927420831008952[10] = -nom_x[10] + true_x[10];
   out_4270927420831008952[11] = -nom_x[11] + true_x[11];
   out_4270927420831008952[12] = -nom_x[12] + true_x[12];
   out_4270927420831008952[13] = -nom_x[13] + true_x[13];
   out_4270927420831008952[14] = -nom_x[14] + true_x[14];
   out_4270927420831008952[15] = -nom_x[15] + true_x[15];
   out_4270927420831008952[16] = -nom_x[16] + true_x[16];
   out_4270927420831008952[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_586936508799435637) {
   out_586936508799435637[0] = 1.0;
   out_586936508799435637[1] = 0.0;
   out_586936508799435637[2] = 0.0;
   out_586936508799435637[3] = 0.0;
   out_586936508799435637[4] = 0.0;
   out_586936508799435637[5] = 0.0;
   out_586936508799435637[6] = 0.0;
   out_586936508799435637[7] = 0.0;
   out_586936508799435637[8] = 0.0;
   out_586936508799435637[9] = 0.0;
   out_586936508799435637[10] = 0.0;
   out_586936508799435637[11] = 0.0;
   out_586936508799435637[12] = 0.0;
   out_586936508799435637[13] = 0.0;
   out_586936508799435637[14] = 0.0;
   out_586936508799435637[15] = 0.0;
   out_586936508799435637[16] = 0.0;
   out_586936508799435637[17] = 0.0;
   out_586936508799435637[18] = 0.0;
   out_586936508799435637[19] = 1.0;
   out_586936508799435637[20] = 0.0;
   out_586936508799435637[21] = 0.0;
   out_586936508799435637[22] = 0.0;
   out_586936508799435637[23] = 0.0;
   out_586936508799435637[24] = 0.0;
   out_586936508799435637[25] = 0.0;
   out_586936508799435637[26] = 0.0;
   out_586936508799435637[27] = 0.0;
   out_586936508799435637[28] = 0.0;
   out_586936508799435637[29] = 0.0;
   out_586936508799435637[30] = 0.0;
   out_586936508799435637[31] = 0.0;
   out_586936508799435637[32] = 0.0;
   out_586936508799435637[33] = 0.0;
   out_586936508799435637[34] = 0.0;
   out_586936508799435637[35] = 0.0;
   out_586936508799435637[36] = 0.0;
   out_586936508799435637[37] = 0.0;
   out_586936508799435637[38] = 1.0;
   out_586936508799435637[39] = 0.0;
   out_586936508799435637[40] = 0.0;
   out_586936508799435637[41] = 0.0;
   out_586936508799435637[42] = 0.0;
   out_586936508799435637[43] = 0.0;
   out_586936508799435637[44] = 0.0;
   out_586936508799435637[45] = 0.0;
   out_586936508799435637[46] = 0.0;
   out_586936508799435637[47] = 0.0;
   out_586936508799435637[48] = 0.0;
   out_586936508799435637[49] = 0.0;
   out_586936508799435637[50] = 0.0;
   out_586936508799435637[51] = 0.0;
   out_586936508799435637[52] = 0.0;
   out_586936508799435637[53] = 0.0;
   out_586936508799435637[54] = 0.0;
   out_586936508799435637[55] = 0.0;
   out_586936508799435637[56] = 0.0;
   out_586936508799435637[57] = 1.0;
   out_586936508799435637[58] = 0.0;
   out_586936508799435637[59] = 0.0;
   out_586936508799435637[60] = 0.0;
   out_586936508799435637[61] = 0.0;
   out_586936508799435637[62] = 0.0;
   out_586936508799435637[63] = 0.0;
   out_586936508799435637[64] = 0.0;
   out_586936508799435637[65] = 0.0;
   out_586936508799435637[66] = 0.0;
   out_586936508799435637[67] = 0.0;
   out_586936508799435637[68] = 0.0;
   out_586936508799435637[69] = 0.0;
   out_586936508799435637[70] = 0.0;
   out_586936508799435637[71] = 0.0;
   out_586936508799435637[72] = 0.0;
   out_586936508799435637[73] = 0.0;
   out_586936508799435637[74] = 0.0;
   out_586936508799435637[75] = 0.0;
   out_586936508799435637[76] = 1.0;
   out_586936508799435637[77] = 0.0;
   out_586936508799435637[78] = 0.0;
   out_586936508799435637[79] = 0.0;
   out_586936508799435637[80] = 0.0;
   out_586936508799435637[81] = 0.0;
   out_586936508799435637[82] = 0.0;
   out_586936508799435637[83] = 0.0;
   out_586936508799435637[84] = 0.0;
   out_586936508799435637[85] = 0.0;
   out_586936508799435637[86] = 0.0;
   out_586936508799435637[87] = 0.0;
   out_586936508799435637[88] = 0.0;
   out_586936508799435637[89] = 0.0;
   out_586936508799435637[90] = 0.0;
   out_586936508799435637[91] = 0.0;
   out_586936508799435637[92] = 0.0;
   out_586936508799435637[93] = 0.0;
   out_586936508799435637[94] = 0.0;
   out_586936508799435637[95] = 1.0;
   out_586936508799435637[96] = 0.0;
   out_586936508799435637[97] = 0.0;
   out_586936508799435637[98] = 0.0;
   out_586936508799435637[99] = 0.0;
   out_586936508799435637[100] = 0.0;
   out_586936508799435637[101] = 0.0;
   out_586936508799435637[102] = 0.0;
   out_586936508799435637[103] = 0.0;
   out_586936508799435637[104] = 0.0;
   out_586936508799435637[105] = 0.0;
   out_586936508799435637[106] = 0.0;
   out_586936508799435637[107] = 0.0;
   out_586936508799435637[108] = 0.0;
   out_586936508799435637[109] = 0.0;
   out_586936508799435637[110] = 0.0;
   out_586936508799435637[111] = 0.0;
   out_586936508799435637[112] = 0.0;
   out_586936508799435637[113] = 0.0;
   out_586936508799435637[114] = 1.0;
   out_586936508799435637[115] = 0.0;
   out_586936508799435637[116] = 0.0;
   out_586936508799435637[117] = 0.0;
   out_586936508799435637[118] = 0.0;
   out_586936508799435637[119] = 0.0;
   out_586936508799435637[120] = 0.0;
   out_586936508799435637[121] = 0.0;
   out_586936508799435637[122] = 0.0;
   out_586936508799435637[123] = 0.0;
   out_586936508799435637[124] = 0.0;
   out_586936508799435637[125] = 0.0;
   out_586936508799435637[126] = 0.0;
   out_586936508799435637[127] = 0.0;
   out_586936508799435637[128] = 0.0;
   out_586936508799435637[129] = 0.0;
   out_586936508799435637[130] = 0.0;
   out_586936508799435637[131] = 0.0;
   out_586936508799435637[132] = 0.0;
   out_586936508799435637[133] = 1.0;
   out_586936508799435637[134] = 0.0;
   out_586936508799435637[135] = 0.0;
   out_586936508799435637[136] = 0.0;
   out_586936508799435637[137] = 0.0;
   out_586936508799435637[138] = 0.0;
   out_586936508799435637[139] = 0.0;
   out_586936508799435637[140] = 0.0;
   out_586936508799435637[141] = 0.0;
   out_586936508799435637[142] = 0.0;
   out_586936508799435637[143] = 0.0;
   out_586936508799435637[144] = 0.0;
   out_586936508799435637[145] = 0.0;
   out_586936508799435637[146] = 0.0;
   out_586936508799435637[147] = 0.0;
   out_586936508799435637[148] = 0.0;
   out_586936508799435637[149] = 0.0;
   out_586936508799435637[150] = 0.0;
   out_586936508799435637[151] = 0.0;
   out_586936508799435637[152] = 1.0;
   out_586936508799435637[153] = 0.0;
   out_586936508799435637[154] = 0.0;
   out_586936508799435637[155] = 0.0;
   out_586936508799435637[156] = 0.0;
   out_586936508799435637[157] = 0.0;
   out_586936508799435637[158] = 0.0;
   out_586936508799435637[159] = 0.0;
   out_586936508799435637[160] = 0.0;
   out_586936508799435637[161] = 0.0;
   out_586936508799435637[162] = 0.0;
   out_586936508799435637[163] = 0.0;
   out_586936508799435637[164] = 0.0;
   out_586936508799435637[165] = 0.0;
   out_586936508799435637[166] = 0.0;
   out_586936508799435637[167] = 0.0;
   out_586936508799435637[168] = 0.0;
   out_586936508799435637[169] = 0.0;
   out_586936508799435637[170] = 0.0;
   out_586936508799435637[171] = 1.0;
   out_586936508799435637[172] = 0.0;
   out_586936508799435637[173] = 0.0;
   out_586936508799435637[174] = 0.0;
   out_586936508799435637[175] = 0.0;
   out_586936508799435637[176] = 0.0;
   out_586936508799435637[177] = 0.0;
   out_586936508799435637[178] = 0.0;
   out_586936508799435637[179] = 0.0;
   out_586936508799435637[180] = 0.0;
   out_586936508799435637[181] = 0.0;
   out_586936508799435637[182] = 0.0;
   out_586936508799435637[183] = 0.0;
   out_586936508799435637[184] = 0.0;
   out_586936508799435637[185] = 0.0;
   out_586936508799435637[186] = 0.0;
   out_586936508799435637[187] = 0.0;
   out_586936508799435637[188] = 0.0;
   out_586936508799435637[189] = 0.0;
   out_586936508799435637[190] = 1.0;
   out_586936508799435637[191] = 0.0;
   out_586936508799435637[192] = 0.0;
   out_586936508799435637[193] = 0.0;
   out_586936508799435637[194] = 0.0;
   out_586936508799435637[195] = 0.0;
   out_586936508799435637[196] = 0.0;
   out_586936508799435637[197] = 0.0;
   out_586936508799435637[198] = 0.0;
   out_586936508799435637[199] = 0.0;
   out_586936508799435637[200] = 0.0;
   out_586936508799435637[201] = 0.0;
   out_586936508799435637[202] = 0.0;
   out_586936508799435637[203] = 0.0;
   out_586936508799435637[204] = 0.0;
   out_586936508799435637[205] = 0.0;
   out_586936508799435637[206] = 0.0;
   out_586936508799435637[207] = 0.0;
   out_586936508799435637[208] = 0.0;
   out_586936508799435637[209] = 1.0;
   out_586936508799435637[210] = 0.0;
   out_586936508799435637[211] = 0.0;
   out_586936508799435637[212] = 0.0;
   out_586936508799435637[213] = 0.0;
   out_586936508799435637[214] = 0.0;
   out_586936508799435637[215] = 0.0;
   out_586936508799435637[216] = 0.0;
   out_586936508799435637[217] = 0.0;
   out_586936508799435637[218] = 0.0;
   out_586936508799435637[219] = 0.0;
   out_586936508799435637[220] = 0.0;
   out_586936508799435637[221] = 0.0;
   out_586936508799435637[222] = 0.0;
   out_586936508799435637[223] = 0.0;
   out_586936508799435637[224] = 0.0;
   out_586936508799435637[225] = 0.0;
   out_586936508799435637[226] = 0.0;
   out_586936508799435637[227] = 0.0;
   out_586936508799435637[228] = 1.0;
   out_586936508799435637[229] = 0.0;
   out_586936508799435637[230] = 0.0;
   out_586936508799435637[231] = 0.0;
   out_586936508799435637[232] = 0.0;
   out_586936508799435637[233] = 0.0;
   out_586936508799435637[234] = 0.0;
   out_586936508799435637[235] = 0.0;
   out_586936508799435637[236] = 0.0;
   out_586936508799435637[237] = 0.0;
   out_586936508799435637[238] = 0.0;
   out_586936508799435637[239] = 0.0;
   out_586936508799435637[240] = 0.0;
   out_586936508799435637[241] = 0.0;
   out_586936508799435637[242] = 0.0;
   out_586936508799435637[243] = 0.0;
   out_586936508799435637[244] = 0.0;
   out_586936508799435637[245] = 0.0;
   out_586936508799435637[246] = 0.0;
   out_586936508799435637[247] = 1.0;
   out_586936508799435637[248] = 0.0;
   out_586936508799435637[249] = 0.0;
   out_586936508799435637[250] = 0.0;
   out_586936508799435637[251] = 0.0;
   out_586936508799435637[252] = 0.0;
   out_586936508799435637[253] = 0.0;
   out_586936508799435637[254] = 0.0;
   out_586936508799435637[255] = 0.0;
   out_586936508799435637[256] = 0.0;
   out_586936508799435637[257] = 0.0;
   out_586936508799435637[258] = 0.0;
   out_586936508799435637[259] = 0.0;
   out_586936508799435637[260] = 0.0;
   out_586936508799435637[261] = 0.0;
   out_586936508799435637[262] = 0.0;
   out_586936508799435637[263] = 0.0;
   out_586936508799435637[264] = 0.0;
   out_586936508799435637[265] = 0.0;
   out_586936508799435637[266] = 1.0;
   out_586936508799435637[267] = 0.0;
   out_586936508799435637[268] = 0.0;
   out_586936508799435637[269] = 0.0;
   out_586936508799435637[270] = 0.0;
   out_586936508799435637[271] = 0.0;
   out_586936508799435637[272] = 0.0;
   out_586936508799435637[273] = 0.0;
   out_586936508799435637[274] = 0.0;
   out_586936508799435637[275] = 0.0;
   out_586936508799435637[276] = 0.0;
   out_586936508799435637[277] = 0.0;
   out_586936508799435637[278] = 0.0;
   out_586936508799435637[279] = 0.0;
   out_586936508799435637[280] = 0.0;
   out_586936508799435637[281] = 0.0;
   out_586936508799435637[282] = 0.0;
   out_586936508799435637[283] = 0.0;
   out_586936508799435637[284] = 0.0;
   out_586936508799435637[285] = 1.0;
   out_586936508799435637[286] = 0.0;
   out_586936508799435637[287] = 0.0;
   out_586936508799435637[288] = 0.0;
   out_586936508799435637[289] = 0.0;
   out_586936508799435637[290] = 0.0;
   out_586936508799435637[291] = 0.0;
   out_586936508799435637[292] = 0.0;
   out_586936508799435637[293] = 0.0;
   out_586936508799435637[294] = 0.0;
   out_586936508799435637[295] = 0.0;
   out_586936508799435637[296] = 0.0;
   out_586936508799435637[297] = 0.0;
   out_586936508799435637[298] = 0.0;
   out_586936508799435637[299] = 0.0;
   out_586936508799435637[300] = 0.0;
   out_586936508799435637[301] = 0.0;
   out_586936508799435637[302] = 0.0;
   out_586936508799435637[303] = 0.0;
   out_586936508799435637[304] = 1.0;
   out_586936508799435637[305] = 0.0;
   out_586936508799435637[306] = 0.0;
   out_586936508799435637[307] = 0.0;
   out_586936508799435637[308] = 0.0;
   out_586936508799435637[309] = 0.0;
   out_586936508799435637[310] = 0.0;
   out_586936508799435637[311] = 0.0;
   out_586936508799435637[312] = 0.0;
   out_586936508799435637[313] = 0.0;
   out_586936508799435637[314] = 0.0;
   out_586936508799435637[315] = 0.0;
   out_586936508799435637[316] = 0.0;
   out_586936508799435637[317] = 0.0;
   out_586936508799435637[318] = 0.0;
   out_586936508799435637[319] = 0.0;
   out_586936508799435637[320] = 0.0;
   out_586936508799435637[321] = 0.0;
   out_586936508799435637[322] = 0.0;
   out_586936508799435637[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6348406745166125280) {
   out_6348406745166125280[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6348406745166125280[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6348406745166125280[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6348406745166125280[3] = dt*state[12] + state[3];
   out_6348406745166125280[4] = dt*state[13] + state[4];
   out_6348406745166125280[5] = dt*state[14] + state[5];
   out_6348406745166125280[6] = state[6];
   out_6348406745166125280[7] = state[7];
   out_6348406745166125280[8] = state[8];
   out_6348406745166125280[9] = state[9];
   out_6348406745166125280[10] = state[10];
   out_6348406745166125280[11] = state[11];
   out_6348406745166125280[12] = state[12];
   out_6348406745166125280[13] = state[13];
   out_6348406745166125280[14] = state[14];
   out_6348406745166125280[15] = state[15];
   out_6348406745166125280[16] = state[16];
   out_6348406745166125280[17] = state[17];
}
void F_fun(double *state, double dt, double *out_2025743393358252300) {
   out_2025743393358252300[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2025743393358252300[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2025743393358252300[2] = 0;
   out_2025743393358252300[3] = 0;
   out_2025743393358252300[4] = 0;
   out_2025743393358252300[5] = 0;
   out_2025743393358252300[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2025743393358252300[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2025743393358252300[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2025743393358252300[9] = 0;
   out_2025743393358252300[10] = 0;
   out_2025743393358252300[11] = 0;
   out_2025743393358252300[12] = 0;
   out_2025743393358252300[13] = 0;
   out_2025743393358252300[14] = 0;
   out_2025743393358252300[15] = 0;
   out_2025743393358252300[16] = 0;
   out_2025743393358252300[17] = 0;
   out_2025743393358252300[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2025743393358252300[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2025743393358252300[20] = 0;
   out_2025743393358252300[21] = 0;
   out_2025743393358252300[22] = 0;
   out_2025743393358252300[23] = 0;
   out_2025743393358252300[24] = 0;
   out_2025743393358252300[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2025743393358252300[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2025743393358252300[27] = 0;
   out_2025743393358252300[28] = 0;
   out_2025743393358252300[29] = 0;
   out_2025743393358252300[30] = 0;
   out_2025743393358252300[31] = 0;
   out_2025743393358252300[32] = 0;
   out_2025743393358252300[33] = 0;
   out_2025743393358252300[34] = 0;
   out_2025743393358252300[35] = 0;
   out_2025743393358252300[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2025743393358252300[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2025743393358252300[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2025743393358252300[39] = 0;
   out_2025743393358252300[40] = 0;
   out_2025743393358252300[41] = 0;
   out_2025743393358252300[42] = 0;
   out_2025743393358252300[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2025743393358252300[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2025743393358252300[45] = 0;
   out_2025743393358252300[46] = 0;
   out_2025743393358252300[47] = 0;
   out_2025743393358252300[48] = 0;
   out_2025743393358252300[49] = 0;
   out_2025743393358252300[50] = 0;
   out_2025743393358252300[51] = 0;
   out_2025743393358252300[52] = 0;
   out_2025743393358252300[53] = 0;
   out_2025743393358252300[54] = 0;
   out_2025743393358252300[55] = 0;
   out_2025743393358252300[56] = 0;
   out_2025743393358252300[57] = 1;
   out_2025743393358252300[58] = 0;
   out_2025743393358252300[59] = 0;
   out_2025743393358252300[60] = 0;
   out_2025743393358252300[61] = 0;
   out_2025743393358252300[62] = 0;
   out_2025743393358252300[63] = 0;
   out_2025743393358252300[64] = 0;
   out_2025743393358252300[65] = 0;
   out_2025743393358252300[66] = dt;
   out_2025743393358252300[67] = 0;
   out_2025743393358252300[68] = 0;
   out_2025743393358252300[69] = 0;
   out_2025743393358252300[70] = 0;
   out_2025743393358252300[71] = 0;
   out_2025743393358252300[72] = 0;
   out_2025743393358252300[73] = 0;
   out_2025743393358252300[74] = 0;
   out_2025743393358252300[75] = 0;
   out_2025743393358252300[76] = 1;
   out_2025743393358252300[77] = 0;
   out_2025743393358252300[78] = 0;
   out_2025743393358252300[79] = 0;
   out_2025743393358252300[80] = 0;
   out_2025743393358252300[81] = 0;
   out_2025743393358252300[82] = 0;
   out_2025743393358252300[83] = 0;
   out_2025743393358252300[84] = 0;
   out_2025743393358252300[85] = dt;
   out_2025743393358252300[86] = 0;
   out_2025743393358252300[87] = 0;
   out_2025743393358252300[88] = 0;
   out_2025743393358252300[89] = 0;
   out_2025743393358252300[90] = 0;
   out_2025743393358252300[91] = 0;
   out_2025743393358252300[92] = 0;
   out_2025743393358252300[93] = 0;
   out_2025743393358252300[94] = 0;
   out_2025743393358252300[95] = 1;
   out_2025743393358252300[96] = 0;
   out_2025743393358252300[97] = 0;
   out_2025743393358252300[98] = 0;
   out_2025743393358252300[99] = 0;
   out_2025743393358252300[100] = 0;
   out_2025743393358252300[101] = 0;
   out_2025743393358252300[102] = 0;
   out_2025743393358252300[103] = 0;
   out_2025743393358252300[104] = dt;
   out_2025743393358252300[105] = 0;
   out_2025743393358252300[106] = 0;
   out_2025743393358252300[107] = 0;
   out_2025743393358252300[108] = 0;
   out_2025743393358252300[109] = 0;
   out_2025743393358252300[110] = 0;
   out_2025743393358252300[111] = 0;
   out_2025743393358252300[112] = 0;
   out_2025743393358252300[113] = 0;
   out_2025743393358252300[114] = 1;
   out_2025743393358252300[115] = 0;
   out_2025743393358252300[116] = 0;
   out_2025743393358252300[117] = 0;
   out_2025743393358252300[118] = 0;
   out_2025743393358252300[119] = 0;
   out_2025743393358252300[120] = 0;
   out_2025743393358252300[121] = 0;
   out_2025743393358252300[122] = 0;
   out_2025743393358252300[123] = 0;
   out_2025743393358252300[124] = 0;
   out_2025743393358252300[125] = 0;
   out_2025743393358252300[126] = 0;
   out_2025743393358252300[127] = 0;
   out_2025743393358252300[128] = 0;
   out_2025743393358252300[129] = 0;
   out_2025743393358252300[130] = 0;
   out_2025743393358252300[131] = 0;
   out_2025743393358252300[132] = 0;
   out_2025743393358252300[133] = 1;
   out_2025743393358252300[134] = 0;
   out_2025743393358252300[135] = 0;
   out_2025743393358252300[136] = 0;
   out_2025743393358252300[137] = 0;
   out_2025743393358252300[138] = 0;
   out_2025743393358252300[139] = 0;
   out_2025743393358252300[140] = 0;
   out_2025743393358252300[141] = 0;
   out_2025743393358252300[142] = 0;
   out_2025743393358252300[143] = 0;
   out_2025743393358252300[144] = 0;
   out_2025743393358252300[145] = 0;
   out_2025743393358252300[146] = 0;
   out_2025743393358252300[147] = 0;
   out_2025743393358252300[148] = 0;
   out_2025743393358252300[149] = 0;
   out_2025743393358252300[150] = 0;
   out_2025743393358252300[151] = 0;
   out_2025743393358252300[152] = 1;
   out_2025743393358252300[153] = 0;
   out_2025743393358252300[154] = 0;
   out_2025743393358252300[155] = 0;
   out_2025743393358252300[156] = 0;
   out_2025743393358252300[157] = 0;
   out_2025743393358252300[158] = 0;
   out_2025743393358252300[159] = 0;
   out_2025743393358252300[160] = 0;
   out_2025743393358252300[161] = 0;
   out_2025743393358252300[162] = 0;
   out_2025743393358252300[163] = 0;
   out_2025743393358252300[164] = 0;
   out_2025743393358252300[165] = 0;
   out_2025743393358252300[166] = 0;
   out_2025743393358252300[167] = 0;
   out_2025743393358252300[168] = 0;
   out_2025743393358252300[169] = 0;
   out_2025743393358252300[170] = 0;
   out_2025743393358252300[171] = 1;
   out_2025743393358252300[172] = 0;
   out_2025743393358252300[173] = 0;
   out_2025743393358252300[174] = 0;
   out_2025743393358252300[175] = 0;
   out_2025743393358252300[176] = 0;
   out_2025743393358252300[177] = 0;
   out_2025743393358252300[178] = 0;
   out_2025743393358252300[179] = 0;
   out_2025743393358252300[180] = 0;
   out_2025743393358252300[181] = 0;
   out_2025743393358252300[182] = 0;
   out_2025743393358252300[183] = 0;
   out_2025743393358252300[184] = 0;
   out_2025743393358252300[185] = 0;
   out_2025743393358252300[186] = 0;
   out_2025743393358252300[187] = 0;
   out_2025743393358252300[188] = 0;
   out_2025743393358252300[189] = 0;
   out_2025743393358252300[190] = 1;
   out_2025743393358252300[191] = 0;
   out_2025743393358252300[192] = 0;
   out_2025743393358252300[193] = 0;
   out_2025743393358252300[194] = 0;
   out_2025743393358252300[195] = 0;
   out_2025743393358252300[196] = 0;
   out_2025743393358252300[197] = 0;
   out_2025743393358252300[198] = 0;
   out_2025743393358252300[199] = 0;
   out_2025743393358252300[200] = 0;
   out_2025743393358252300[201] = 0;
   out_2025743393358252300[202] = 0;
   out_2025743393358252300[203] = 0;
   out_2025743393358252300[204] = 0;
   out_2025743393358252300[205] = 0;
   out_2025743393358252300[206] = 0;
   out_2025743393358252300[207] = 0;
   out_2025743393358252300[208] = 0;
   out_2025743393358252300[209] = 1;
   out_2025743393358252300[210] = 0;
   out_2025743393358252300[211] = 0;
   out_2025743393358252300[212] = 0;
   out_2025743393358252300[213] = 0;
   out_2025743393358252300[214] = 0;
   out_2025743393358252300[215] = 0;
   out_2025743393358252300[216] = 0;
   out_2025743393358252300[217] = 0;
   out_2025743393358252300[218] = 0;
   out_2025743393358252300[219] = 0;
   out_2025743393358252300[220] = 0;
   out_2025743393358252300[221] = 0;
   out_2025743393358252300[222] = 0;
   out_2025743393358252300[223] = 0;
   out_2025743393358252300[224] = 0;
   out_2025743393358252300[225] = 0;
   out_2025743393358252300[226] = 0;
   out_2025743393358252300[227] = 0;
   out_2025743393358252300[228] = 1;
   out_2025743393358252300[229] = 0;
   out_2025743393358252300[230] = 0;
   out_2025743393358252300[231] = 0;
   out_2025743393358252300[232] = 0;
   out_2025743393358252300[233] = 0;
   out_2025743393358252300[234] = 0;
   out_2025743393358252300[235] = 0;
   out_2025743393358252300[236] = 0;
   out_2025743393358252300[237] = 0;
   out_2025743393358252300[238] = 0;
   out_2025743393358252300[239] = 0;
   out_2025743393358252300[240] = 0;
   out_2025743393358252300[241] = 0;
   out_2025743393358252300[242] = 0;
   out_2025743393358252300[243] = 0;
   out_2025743393358252300[244] = 0;
   out_2025743393358252300[245] = 0;
   out_2025743393358252300[246] = 0;
   out_2025743393358252300[247] = 1;
   out_2025743393358252300[248] = 0;
   out_2025743393358252300[249] = 0;
   out_2025743393358252300[250] = 0;
   out_2025743393358252300[251] = 0;
   out_2025743393358252300[252] = 0;
   out_2025743393358252300[253] = 0;
   out_2025743393358252300[254] = 0;
   out_2025743393358252300[255] = 0;
   out_2025743393358252300[256] = 0;
   out_2025743393358252300[257] = 0;
   out_2025743393358252300[258] = 0;
   out_2025743393358252300[259] = 0;
   out_2025743393358252300[260] = 0;
   out_2025743393358252300[261] = 0;
   out_2025743393358252300[262] = 0;
   out_2025743393358252300[263] = 0;
   out_2025743393358252300[264] = 0;
   out_2025743393358252300[265] = 0;
   out_2025743393358252300[266] = 1;
   out_2025743393358252300[267] = 0;
   out_2025743393358252300[268] = 0;
   out_2025743393358252300[269] = 0;
   out_2025743393358252300[270] = 0;
   out_2025743393358252300[271] = 0;
   out_2025743393358252300[272] = 0;
   out_2025743393358252300[273] = 0;
   out_2025743393358252300[274] = 0;
   out_2025743393358252300[275] = 0;
   out_2025743393358252300[276] = 0;
   out_2025743393358252300[277] = 0;
   out_2025743393358252300[278] = 0;
   out_2025743393358252300[279] = 0;
   out_2025743393358252300[280] = 0;
   out_2025743393358252300[281] = 0;
   out_2025743393358252300[282] = 0;
   out_2025743393358252300[283] = 0;
   out_2025743393358252300[284] = 0;
   out_2025743393358252300[285] = 1;
   out_2025743393358252300[286] = 0;
   out_2025743393358252300[287] = 0;
   out_2025743393358252300[288] = 0;
   out_2025743393358252300[289] = 0;
   out_2025743393358252300[290] = 0;
   out_2025743393358252300[291] = 0;
   out_2025743393358252300[292] = 0;
   out_2025743393358252300[293] = 0;
   out_2025743393358252300[294] = 0;
   out_2025743393358252300[295] = 0;
   out_2025743393358252300[296] = 0;
   out_2025743393358252300[297] = 0;
   out_2025743393358252300[298] = 0;
   out_2025743393358252300[299] = 0;
   out_2025743393358252300[300] = 0;
   out_2025743393358252300[301] = 0;
   out_2025743393358252300[302] = 0;
   out_2025743393358252300[303] = 0;
   out_2025743393358252300[304] = 1;
   out_2025743393358252300[305] = 0;
   out_2025743393358252300[306] = 0;
   out_2025743393358252300[307] = 0;
   out_2025743393358252300[308] = 0;
   out_2025743393358252300[309] = 0;
   out_2025743393358252300[310] = 0;
   out_2025743393358252300[311] = 0;
   out_2025743393358252300[312] = 0;
   out_2025743393358252300[313] = 0;
   out_2025743393358252300[314] = 0;
   out_2025743393358252300[315] = 0;
   out_2025743393358252300[316] = 0;
   out_2025743393358252300[317] = 0;
   out_2025743393358252300[318] = 0;
   out_2025743393358252300[319] = 0;
   out_2025743393358252300[320] = 0;
   out_2025743393358252300[321] = 0;
   out_2025743393358252300[322] = 0;
   out_2025743393358252300[323] = 1;
}
void h_4(double *state, double *unused, double *out_2320996590281528671) {
   out_2320996590281528671[0] = state[6] + state[9];
   out_2320996590281528671[1] = state[7] + state[10];
   out_2320996590281528671[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8426047911114248855) {
   out_8426047911114248855[0] = 0;
   out_8426047911114248855[1] = 0;
   out_8426047911114248855[2] = 0;
   out_8426047911114248855[3] = 0;
   out_8426047911114248855[4] = 0;
   out_8426047911114248855[5] = 0;
   out_8426047911114248855[6] = 1;
   out_8426047911114248855[7] = 0;
   out_8426047911114248855[8] = 0;
   out_8426047911114248855[9] = 1;
   out_8426047911114248855[10] = 0;
   out_8426047911114248855[11] = 0;
   out_8426047911114248855[12] = 0;
   out_8426047911114248855[13] = 0;
   out_8426047911114248855[14] = 0;
   out_8426047911114248855[15] = 0;
   out_8426047911114248855[16] = 0;
   out_8426047911114248855[17] = 0;
   out_8426047911114248855[18] = 0;
   out_8426047911114248855[19] = 0;
   out_8426047911114248855[20] = 0;
   out_8426047911114248855[21] = 0;
   out_8426047911114248855[22] = 0;
   out_8426047911114248855[23] = 0;
   out_8426047911114248855[24] = 0;
   out_8426047911114248855[25] = 1;
   out_8426047911114248855[26] = 0;
   out_8426047911114248855[27] = 0;
   out_8426047911114248855[28] = 1;
   out_8426047911114248855[29] = 0;
   out_8426047911114248855[30] = 0;
   out_8426047911114248855[31] = 0;
   out_8426047911114248855[32] = 0;
   out_8426047911114248855[33] = 0;
   out_8426047911114248855[34] = 0;
   out_8426047911114248855[35] = 0;
   out_8426047911114248855[36] = 0;
   out_8426047911114248855[37] = 0;
   out_8426047911114248855[38] = 0;
   out_8426047911114248855[39] = 0;
   out_8426047911114248855[40] = 0;
   out_8426047911114248855[41] = 0;
   out_8426047911114248855[42] = 0;
   out_8426047911114248855[43] = 0;
   out_8426047911114248855[44] = 1;
   out_8426047911114248855[45] = 0;
   out_8426047911114248855[46] = 0;
   out_8426047911114248855[47] = 1;
   out_8426047911114248855[48] = 0;
   out_8426047911114248855[49] = 0;
   out_8426047911114248855[50] = 0;
   out_8426047911114248855[51] = 0;
   out_8426047911114248855[52] = 0;
   out_8426047911114248855[53] = 0;
}
void h_10(double *state, double *unused, double *out_1276261552857593432) {
   out_1276261552857593432[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1276261552857593432[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1276261552857593432[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3386729453987378675) {
   out_3386729453987378675[0] = 0;
   out_3386729453987378675[1] = 9.8100000000000005*cos(state[1]);
   out_3386729453987378675[2] = 0;
   out_3386729453987378675[3] = 0;
   out_3386729453987378675[4] = -state[8];
   out_3386729453987378675[5] = state[7];
   out_3386729453987378675[6] = 0;
   out_3386729453987378675[7] = state[5];
   out_3386729453987378675[8] = -state[4];
   out_3386729453987378675[9] = 0;
   out_3386729453987378675[10] = 0;
   out_3386729453987378675[11] = 0;
   out_3386729453987378675[12] = 1;
   out_3386729453987378675[13] = 0;
   out_3386729453987378675[14] = 0;
   out_3386729453987378675[15] = 1;
   out_3386729453987378675[16] = 0;
   out_3386729453987378675[17] = 0;
   out_3386729453987378675[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3386729453987378675[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3386729453987378675[20] = 0;
   out_3386729453987378675[21] = state[8];
   out_3386729453987378675[22] = 0;
   out_3386729453987378675[23] = -state[6];
   out_3386729453987378675[24] = -state[5];
   out_3386729453987378675[25] = 0;
   out_3386729453987378675[26] = state[3];
   out_3386729453987378675[27] = 0;
   out_3386729453987378675[28] = 0;
   out_3386729453987378675[29] = 0;
   out_3386729453987378675[30] = 0;
   out_3386729453987378675[31] = 1;
   out_3386729453987378675[32] = 0;
   out_3386729453987378675[33] = 0;
   out_3386729453987378675[34] = 1;
   out_3386729453987378675[35] = 0;
   out_3386729453987378675[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3386729453987378675[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3386729453987378675[38] = 0;
   out_3386729453987378675[39] = -state[7];
   out_3386729453987378675[40] = state[6];
   out_3386729453987378675[41] = 0;
   out_3386729453987378675[42] = state[4];
   out_3386729453987378675[43] = -state[3];
   out_3386729453987378675[44] = 0;
   out_3386729453987378675[45] = 0;
   out_3386729453987378675[46] = 0;
   out_3386729453987378675[47] = 0;
   out_3386729453987378675[48] = 0;
   out_3386729453987378675[49] = 0;
   out_3386729453987378675[50] = 1;
   out_3386729453987378675[51] = 0;
   out_3386729453987378675[52] = 0;
   out_3386729453987378675[53] = 1;
}
void h_13(double *state, double *unused, double *out_1056904525204352590) {
   out_1056904525204352590[0] = state[3];
   out_1056904525204352590[1] = state[4];
   out_1056904525204352590[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6808422337262969960) {
   out_6808422337262969960[0] = 0;
   out_6808422337262969960[1] = 0;
   out_6808422337262969960[2] = 0;
   out_6808422337262969960[3] = 1;
   out_6808422337262969960[4] = 0;
   out_6808422337262969960[5] = 0;
   out_6808422337262969960[6] = 0;
   out_6808422337262969960[7] = 0;
   out_6808422337262969960[8] = 0;
   out_6808422337262969960[9] = 0;
   out_6808422337262969960[10] = 0;
   out_6808422337262969960[11] = 0;
   out_6808422337262969960[12] = 0;
   out_6808422337262969960[13] = 0;
   out_6808422337262969960[14] = 0;
   out_6808422337262969960[15] = 0;
   out_6808422337262969960[16] = 0;
   out_6808422337262969960[17] = 0;
   out_6808422337262969960[18] = 0;
   out_6808422337262969960[19] = 0;
   out_6808422337262969960[20] = 0;
   out_6808422337262969960[21] = 0;
   out_6808422337262969960[22] = 1;
   out_6808422337262969960[23] = 0;
   out_6808422337262969960[24] = 0;
   out_6808422337262969960[25] = 0;
   out_6808422337262969960[26] = 0;
   out_6808422337262969960[27] = 0;
   out_6808422337262969960[28] = 0;
   out_6808422337262969960[29] = 0;
   out_6808422337262969960[30] = 0;
   out_6808422337262969960[31] = 0;
   out_6808422337262969960[32] = 0;
   out_6808422337262969960[33] = 0;
   out_6808422337262969960[34] = 0;
   out_6808422337262969960[35] = 0;
   out_6808422337262969960[36] = 0;
   out_6808422337262969960[37] = 0;
   out_6808422337262969960[38] = 0;
   out_6808422337262969960[39] = 0;
   out_6808422337262969960[40] = 0;
   out_6808422337262969960[41] = 1;
   out_6808422337262969960[42] = 0;
   out_6808422337262969960[43] = 0;
   out_6808422337262969960[44] = 0;
   out_6808422337262969960[45] = 0;
   out_6808422337262969960[46] = 0;
   out_6808422337262969960[47] = 0;
   out_6808422337262969960[48] = 0;
   out_6808422337262969960[49] = 0;
   out_6808422337262969960[50] = 0;
   out_6808422337262969960[51] = 0;
   out_6808422337262969960[52] = 0;
   out_6808422337262969960[53] = 0;
}
void h_14(double *state, double *unused, double *out_4613729524718250821) {
   out_4613729524718250821[0] = state[6];
   out_4613729524718250821[1] = state[7];
   out_4613729524718250821[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5343259478818876559) {
   out_5343259478818876559[0] = 0;
   out_5343259478818876559[1] = 0;
   out_5343259478818876559[2] = 0;
   out_5343259478818876559[3] = 0;
   out_5343259478818876559[4] = 0;
   out_5343259478818876559[5] = 0;
   out_5343259478818876559[6] = 1;
   out_5343259478818876559[7] = 0;
   out_5343259478818876559[8] = 0;
   out_5343259478818876559[9] = 0;
   out_5343259478818876559[10] = 0;
   out_5343259478818876559[11] = 0;
   out_5343259478818876559[12] = 0;
   out_5343259478818876559[13] = 0;
   out_5343259478818876559[14] = 0;
   out_5343259478818876559[15] = 0;
   out_5343259478818876559[16] = 0;
   out_5343259478818876559[17] = 0;
   out_5343259478818876559[18] = 0;
   out_5343259478818876559[19] = 0;
   out_5343259478818876559[20] = 0;
   out_5343259478818876559[21] = 0;
   out_5343259478818876559[22] = 0;
   out_5343259478818876559[23] = 0;
   out_5343259478818876559[24] = 0;
   out_5343259478818876559[25] = 1;
   out_5343259478818876559[26] = 0;
   out_5343259478818876559[27] = 0;
   out_5343259478818876559[28] = 0;
   out_5343259478818876559[29] = 0;
   out_5343259478818876559[30] = 0;
   out_5343259478818876559[31] = 0;
   out_5343259478818876559[32] = 0;
   out_5343259478818876559[33] = 0;
   out_5343259478818876559[34] = 0;
   out_5343259478818876559[35] = 0;
   out_5343259478818876559[36] = 0;
   out_5343259478818876559[37] = 0;
   out_5343259478818876559[38] = 0;
   out_5343259478818876559[39] = 0;
   out_5343259478818876559[40] = 0;
   out_5343259478818876559[41] = 0;
   out_5343259478818876559[42] = 0;
   out_5343259478818876559[43] = 0;
   out_5343259478818876559[44] = 1;
   out_5343259478818876559[45] = 0;
   out_5343259478818876559[46] = 0;
   out_5343259478818876559[47] = 0;
   out_5343259478818876559[48] = 0;
   out_5343259478818876559[49] = 0;
   out_5343259478818876559[50] = 0;
   out_5343259478818876559[51] = 0;
   out_5343259478818876559[52] = 0;
   out_5343259478818876559[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_2790427624331697182) {
  err_fun(nom_x, delta_x, out_2790427624331697182);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4270927420831008952) {
  inv_err_fun(nom_x, true_x, out_4270927420831008952);
}
void pose_H_mod_fun(double *state, double *out_586936508799435637) {
  H_mod_fun(state, out_586936508799435637);
}
void pose_f_fun(double *state, double dt, double *out_6348406745166125280) {
  f_fun(state,  dt, out_6348406745166125280);
}
void pose_F_fun(double *state, double dt, double *out_2025743393358252300) {
  F_fun(state,  dt, out_2025743393358252300);
}
void pose_h_4(double *state, double *unused, double *out_2320996590281528671) {
  h_4(state, unused, out_2320996590281528671);
}
void pose_H_4(double *state, double *unused, double *out_8426047911114248855) {
  H_4(state, unused, out_8426047911114248855);
}
void pose_h_10(double *state, double *unused, double *out_1276261552857593432) {
  h_10(state, unused, out_1276261552857593432);
}
void pose_H_10(double *state, double *unused, double *out_3386729453987378675) {
  H_10(state, unused, out_3386729453987378675);
}
void pose_h_13(double *state, double *unused, double *out_1056904525204352590) {
  h_13(state, unused, out_1056904525204352590);
}
void pose_H_13(double *state, double *unused, double *out_6808422337262969960) {
  H_13(state, unused, out_6808422337262969960);
}
void pose_h_14(double *state, double *unused, double *out_4613729524718250821) {
  h_14(state, unused, out_4613729524718250821);
}
void pose_H_14(double *state, double *unused, double *out_5343259478818876559) {
  H_14(state, unused, out_5343259478818876559);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
