param BasicParameters{i in {1..15}};
param Bondary{i in {1..6}};

param Nfe := BasicParameters[1];
param vmax := BasicParameters[2];
param amax := BasicParameters[3];
param phymax := BasicParameters[4];
param wmax := BasicParameters[5];
param xmin := BasicParameters[6];
param xmax := BasicParameters[7];
param ymin := BasicParameters[8];
param ymax := BasicParameters[9];
param Nobs := BasicParameters[10];
param lb := BasicParameters[11];
param lr := BasicParameters[12];
param lw := BasicParameters[13];
param lwf := BasicParameters[14];
param vehicle_area := BasicParameters[15]; 
#param trust_region := BasicParameters[16]; 

param Obs_area{i in {1..Nobs}};
param Obs_vnum{i in {1..Nobs}};
param Obs_vex{i in {1..Nobs}, j in {1..Obs_vnum[i]}, k in {1..2}};
param p_sign{i in {1..Nfe}, j in {1..Nobs}, k in {1..Obs_vnum[j]-1}, m in {1..4}};
param a_sign{i in {1..Nfe}, j in {1..Nobs}, k in {1..Obs_vnum[j]-1}};
param b_sign{i in {1..Nfe}, j in {1..Nobs}, k in {1..Obs_vnum[j]-1}};
param c_sign{i in {1..Nfe}, j in {1..Nobs}, k in {1..Obs_vnum[j]-1}};
param d_sign{i in {1..Nfe}, j in {1..Nobs}, k in {1..Obs_vnum[j]-1}};
param obs_mark{j in {1..Nobs}, i in {1..Nfe}, k in {1..Obs_vnum[j]-1}};
param vehicle_mark{i in {1..Nfe}, j in {1..Nobs}};
param IW{i in {1..Nfe}, j in {1..3}};
param trust_region{i in {1..Nfe}};

var x{i in {1..Nfe}};
var y{i in {1..Nfe}};
var theta{i in {1..Nfe}};
var v{i in {1..Nfe}};
var a{i in {1..Nfe}};
var phy{i in {1..Nfe}};
var w{i in {1..Nfe}};
var tf >= 0.1;
var hi = tf / (Nfe - 1);
var AX{i in {1..Nfe}};
var BX{i in {1..Nfe}};
var CX{i in {1..Nfe}};
var DX{i in {1..Nfe}};
var AY{i in {1..Nfe}};
var BY{i in {1..Nfe}};
var CY{i in {1..Nfe}};
var DY{i in {1..Nfe}};

minimize obj:
tf;

s.t. hi_limit:
#0.001 <= hi <= 0.249;
0.001 <= hi <= 0.3;

################	Task	##################
s.t. init_x:
x[1] = Bondary[1];
s.t. init_y :
y[1] = Bondary[2];
s.t. init_theta_sin :
sin(theta[1]) = sin(Bondary[3]);
s.t. init_theta_cos :
cos(theta[1]) = cos(Bondary[3]);
s.t. init_v :
v[1] = 0;
#s.t. init_a :
#a[1] = 0;
s.t. init_phy :
phy[1] = 0;
#s.t. init_w :
#w[1] = 0;

s.t. end_x :
x[Nfe] = Bondary[4];
s.t. end_y :
y[Nfe] = Bondary[5];
s.t. end_theta_sin:
sin(theta[Nfe]) = sin(Bondary[6]);
s.t. end_theta_cos:
cos(theta[Nfe]) = cos(Bondary[6]);
s.t. end_v :
v[Nfe] = 0;
s.t. end_a :
a[Nfe] = 0;
s.t. end_phy :
phy[Nfe] = 0;
s.t. end_w :
w[Nfe] = 0;

##############	Kinematic	################
s.t. diff_x {i in {2..Nfe}}:
x[i] = x[i-1] + hi * v[i] * cos(theta[i]);
s.t. diff_y {i in {2..Nfe}}:
y[i] = y[i-1] + hi * v[i] * sin(theta[i]);
s.t. diff_v {i in {2..Nfe}}:
v[i] = v[i-1] + hi * a[i-1];
s.t. diff_theta {i in {2..Nfe}}:
theta[i] = theta[i-1] + hi * tan(phy[i]) * v[i] / lw;
s.t. diff_phy {i in {2..Nfe}}:
phy[i] = phy[i-1] + hi * w[i-1];

##############	Collision-Avoidance	################
s.t. Vehicle_A_X {i in {1..Nfe}}:
AX[i] = x[i] + lwf * cos(theta[i]) - (0.5 * lb) * sin(theta[i]);
s.t. Vehicle_B_x {i in {1..Nfe}}:
BX[i] = x[i] + lwf * cos(theta[i]) + (0.5 * lb) * sin(theta[i]);
s.t. Vehicle_C_x {i in {1..Nfe}}:
CX[i] = x[i] - lr * cos(theta[i]) + (0.5 * lb) * sin(theta[i]);
s.t. Vehicle_D_x {i in {1..Nfe}}:
DX[i] = x[i] - lr * cos(theta[i]) - (0.5 * lb) * sin(theta[i]);
s.t. Vehicle_A_y {i in {1..Nfe}}:
AY[i] = y[i] + lwf * sin(theta[i]) + (0.5 * lb) * cos(theta[i]);
s.t. Vehicle_B_y {i in {1..Nfe}}:
BY[i] = y[i] + lwf * sin(theta[i]) - (0.5 * lb) * cos(theta[i]);
s.t. Vehicle_C_y {i in {1..Nfe}}:
CY[i] = y[i] - lr * sin(theta[i]) - (0.5 * lb) * cos(theta[i]);
s.t. Vehicle_D_y {i in {1..Nfe}}:
DY[i] = y[i] - lr * sin(theta[i]) + (0.5 * lb) * cos(theta[i]);

s.t. P_out_of_ABCD {i in {1..Nfe}, j in {1..Nobs}, k in {1..Obs_vnum[j]-1}}:
obs_mark[j,i,k] * (0.5 * p_sign[i,j,k,1] * (Obs_vex[j,k,1] * AY[i] + AX[i] * BY[i] + BX[i] * Obs_vex[j,k,2] - Obs_vex[j,k,1] * BY[i] - AX[i] * Obs_vex[j,k,2] - BX[i] * AY[i]) +
0.5 * p_sign[i,j,k,2] * (Obs_vex[j,k,1] * BY[i] + BX[i] * CY[i] + CX[i] * Obs_vex[j,k,2] - Obs_vex[j,k,1] * CY[i] - BX[i] * Obs_vex[j,k,2] - CX[i] * BY[i]) +
0.5 * p_sign[i,j,k,3] * (Obs_vex[j,k,1] * CY[i] + CX[i] * DY[i] + DX[i] * Obs_vex[j,k,2] - Obs_vex[j,k,1] * DY[i] - CX[i] * Obs_vex[j,k,2] - DX[i] * CY[i]) +
0.5 * p_sign[i,j,k,4] * (Obs_vex[j,k,1] * AY[i] + AX[i] * DY[i] + DX[i] * Obs_vex[j,k,2] - Obs_vex[j,k,1] * DY[i] - AX[i] * Obs_vex[j,k,2] - DX[i] * AY[i])) >= obs_mark[j,i,k] * (vehicle_area + 0.1);

s.t.A_out_of_P {i in {1..Nfe}, j in {1..Nobs}}:
vehicle_mark[i,j] * sum{k in {1..Obs_vnum[j]-1}}(0.5 * a_sign[i,j,k] * (AX[i] * Obs_vex[j,k,2] + Obs_vex[j,k,1] * Obs_vex[j,k+1,2] + Obs_vex[j,k+1,1] * AY[i] - 
AX[i] * Obs_vex[j,k+1,2] - Obs_vex[j,k,1] * AY[i] - Obs_vex[j,k+1,1] * Obs_vex[j,k,2] )) >= vehicle_mark[i,j] * (Obs_area[j] + 0.1);

s.t.B_out_of_P {i in {1..Nfe}, j in {1..Nobs}}:
vehicle_mark[i,j] * sum{k in {1..Obs_vnum[j]-1}}(0.5 * b_sign[i,j,k] * (BX[i] * Obs_vex[j,k,2] + Obs_vex[j,k,1] * Obs_vex[j,k+1,2] + Obs_vex[j,k+1,1] * BY[i] - 
BX[i] * Obs_vex[j,k+1,2] - Obs_vex[j,k,1] * BY[i] - Obs_vex[j,k+1,1] * Obs_vex[j,k,2] )) >= vehicle_mark[i,j] * (Obs_area[j] + 0.1);

s.t.C_out_of_P {i in {1..Nfe}, j in {1..Nobs}}:
vehicle_mark[i,j] * sum{k in {1..Obs_vnum[j]-1}}(0.5 * c_sign[i,j,k] * (CX[i] * Obs_vex[j,k,2] + Obs_vex[j,k,1] * Obs_vex[j,k+1,2] + Obs_vex[j,k+1,1] * CY[i] - 
CX[i] * Obs_vex[j,k+1,2] - Obs_vex[j,k,1] * CY[i] - Obs_vex[j,k+1,1] * Obs_vex[j,k,2] )) >= vehicle_mark[i,j] * (Obs_area[j] + 0.1);

s.t.D_out_of_P {i in {1..Nfe}, j in {1..Nobs}}:
vehicle_mark[i,j] * sum{k in {1..Obs_vnum[j]-1}}(0.5 * d_sign[i,j,k] * (DX[i] * Obs_vex[j,k,2] + Obs_vex[j,k,1] * Obs_vex[j,k+1,2] + Obs_vex[j,k+1,1] * DY[i] - 
DX[i] * Obs_vex[j,k+1,2] - Obs_vex[j,k,1] * DY[i] - Obs_vex[j,k+1,1] * Obs_vex[j,k,2] )) >= vehicle_mark[i,j] * (Obs_area[j] + 0.1) ;


s.t. XX_Box{i in {2..Nfe-1}}:
IW[i, 1] - trust_region[i] <= x[i] <= IW[i, 1] + trust_region[i];

s.t. YY_Box{i in {2..Nfe-1}}:
IW[i, 2] - trust_region[i] <= y[i] <= IW[i, 2] + trust_region[i];

s.t. TT_Box{i in {2..Nfe-1}}:
IW[i, 3] - phymax <= theta[i] <= IW[i, 3] + phymax;


##############	Boundary	################
s.t. Boundary_phy {i in {1..Nfe}}:
-phymax <= phy[i] <= phymax;
s.t. Boundary_v {i in {1..Nfe}}:
-vmax <= v[i] <= vmax;
s.t. Boundary_w {i in {1..Nfe}}:
-wmax <= w[i] <= wmax;
s.t. Boundary_a {i in {1..Nfe}}:
-amax <= a[i] <= amax;

data;
param: Bondary := include NLPFolder\Bondary;
param: BasicParameters := include NLPFolder\BasicParameters;
param: Obs_area := include NLPFolder\Obs_area;
param: Obs_vnum := include NLPFolder\Obs_vnum;
param: Obs_vex := include NLPFolder\Obs_vex;
param: p_sign := include NLPFolder\p_sign;
param: a_sign := include NLPFolder\a_sign;
param: b_sign := include NLPFolder\b_sign;
param: c_sign := include NLPFolder\c_sign;
param: d_sign := include NLPFolder\d_sign;
param: obs_mark := include NLPFolder\obs_mark;
param: vehicle_mark := include NLPFolder\vehicle_mark;
param: IW := include NLPFolder\IW;
param: trust_region := include NLPFolder\trust_region;