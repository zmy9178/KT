function [dYdt,ax_body_1] = dynsys(t,Y,params)
% DYNSYS 稳降阶段的 10 状态动力学方程。
%
% 状态 Y = [beta, theta, u, w, x_body, vx_body, che_vx, che_vy, che_x, che_y]。
% beta/theta 为侧倾角/俯仰角，u/w 为角速度，x_body/vx_body 为体轴位移和速度，
% che_* 是稳降阶段车辆相对空间位移/速度辅助状态。
%
% 本函数混用 cosd/sind 的角度制姿态转换和常规代数计算。迁移时需明确所有角度单位，
% 尤其不要把 beta/theta 误当作弧度传入三角函数。

beta = Y(1);
theta = Y(2);

u = Y(3);
w = Y(4);

x = Y(5);
vx_body = Y(6);
che_vx = Y(7);
che_vy = Y(8);
che_x = Y(9);
che_y = Y(10);

R = [-cosd(theta)*cosd(beta),-sind(beta),-sind(theta)*cosd(beta);
    -cosd(theta)*sind(beta),cosd(beta),-sind(theta)*sind(beta);
    -sind(theta),0,cosd(theta)];

global drop_envi
density_air = drop_envi.AirDensity;
D = 6.5*1E-3;    %大气温度梯度系数；
T0= drop_envi.T+273; %气温273k；
weiyi = R'*[Y(5);0;0];
if -weiyi(3,1)+params.Z<=params.H-drop_envi.Land_hight
    h = params.H-(-weiyi(3,1) + params.Z);
    rho = density_air*(1-D*h/T0)^4.26;%%空气密度(随高度变化)
else
    h = drop_envi.Land_hight;
end
rho = density_air*(1-D*h/T0)^4.26; %%空气密度(随高度变化)；

m = params.m;
M = params.M;
L = params.L;
CA = params.CA;
CA1 = params.CA1;
CA2 = params.CA2;
CA3 = params.CA3;
Ic = params.Ic;
g = params.g;
CA22 = params.CA22;
CA33 = params.CA33;
aera_mainchute=params.aera_mainchute;
wind_x = params.wind_x;
wind_y = params.wind_y;
wind_z = params.wind_z;

wind_body = R*[wind_x;wind_y;wind_z];

wind_x_body = wind_body(1);
wind_y_body = wind_body(2);
wind_z_body = wind_body(3);
wv = R'*[vx_body;0;0];
V_rel_x_body = -wv(3,1)-wind_x_body;
V_rel_y_body = che_vy-wind_y_body;
V_rel_z_body = che_vx+wind_z_body;
VwM = w*(m/(m+M))*L+wind_z_body;
Vwm = w*(M/(m+M))*L-wind_z_body;
VuM = u*(m/(m+M))*L+wind_y_body;
Vum = u*(M/(m+M))*L-wind_y_body;

Delta_x_sq = V_rel_x_body*abs(V_rel_x_body);
Delta_y_sq = V_rel_y_body*abs(V_rel_y_body);
Delta_z_sq = V_rel_z_body*abs(V_rel_z_body);
Delta_wM_sq = VwM*abs(VwM);
Delta_wm_sq = Vwm*abs(Vwm);
Delta_uM_sq = VuM*abs(VuM);
Delta_um_sq = Vum*abs(Vum);

du_dt = (-0.5*rho*Delta_uM_sq*CA2*(m/(m+M))*L-0.5*rho*Delta_um_sq*(M/(m+M))*L*CA22)/Ic;
dbeta_dt =u;
dw_dt =( M*g*cosd(theta)*(m/(m+M))*L-0.5*rho*Delta_wm_sq*(M/(m+M))*L*CA33)/Ic;
if abs(dw_dt) > 0.005
    dtheta_dt = w;
else
    dtheta_dt = 0;
end


ax_body = ((M+m)*g*sind(theta)-0.5*rho*Delta_x_sq*CA-0.5*rho*Delta_x_sq*CA1)/(M+m);
ay_body = (-0.5*rho*Delta_y_sq*CA22-0.5*rho*Delta_y_sq*CA2)/(M+m);
az_body = (-0.5*rho*Delta_z_sq*CA33-0.5*rho*Delta_z_sq*CA3)/(M+m);
ax_body_1 =-0.5*rho*Delta_x_sq*aera_mainchute;
dx_body_dt = vx_body;

dvx_body_dt = ax_body;

dYdt = zeros(6,1);

dYdt(1) = dbeta_dt;
dYdt(2) = dtheta_dt;

dYdt(3) = du_dt;
dYdt(4) = dw_dt;

dYdt(5) = dx_body_dt;

dYdt(6) = dvx_body_dt;

dYdt(7) = az_body;
dYdt(8) = ay_body;
dYdt(9) = che_vx;
dYdt(10) = che_vy;
end                                                        
