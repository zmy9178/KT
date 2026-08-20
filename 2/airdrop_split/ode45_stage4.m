function dy = ode45_stage4(t,y)
% ODE45_STAGE4 减速伞充气阶段的微分方程右端。
%
% 输入 y = [Xd, Yd, theta, Vw]。
% 按时间增长减速伞有效面积 Sj，并引入附加质量 Msf 与其变化率 dMsf。
% 返回车辆沿轨迹方向的运动方程。

%%%参数赋初值：
global Vehicle drop_envi mainchute decechute data_stage_3
density_air = drop_envi.AirDensity;
grav = drop_envi.Grav;
drop_hight = drop_envi.drophight;
resist_hull = Vehicle.resist_hull;
drop_mass = Vehicle.dropmass;
jiansusan_aera = decechute.aera;
hull_L = Vehicle.hull_L;
hull_W = Vehicle.hull_W; 
hull_H = Vehicle.hull_H; 
jiansusan_mass = decechute.mass;
jiansu_kf = decechute.kf;
jiansusan_num = mainchute.ChuteNum;
jiansusan_c = decechute.resist;
k_ks = mainchute.k_ks;
VL1 = data_stage_3(5,1);
CAjst = jiansusan_c * jiansusan_aera ;           %%主伞阻力系数；
t1 = k_ks*sqrt(CAjst)/VL1;
%%%%定义参数%%%%
D = 6.5*1E-3;    %大气温度梯度系数；
T0= 273;         %气温273k；
h = drop_hight - y(2);
rho = density_air*(1-D*h/T0)^4.26;  %%空气密度(随高度变化)；
g = grav;                           %%重力加速度；
yita = 1.05 - 0.05 * jiansusan_num; %伞群效率；
CAw = resist_hull;     %%空投物阻力系数；
Mw = drop_mass*1000;    %%空投车辆质量；
% Mys = mass_parachuter;    %%牵引伞及伞衣套质量；
% Mms = mass_chuteclothes;  %主伞质量；
Mjs = jiansusan_mass; %减速伞质量；
dy = zeros(4,1);
Sq = (hull_H*abs(-pi/2+y(3)))+hull_L*cos(-pi/2+y(3))*hull_W;
m =  length(t);
ks = jiansusan_aera;
for i = 1:m
    if t(i) <= t1
        Sm = ks * t(i)/t1;
        Dsm = ks /t1;
    else
        Sm = ks;                    %减速伞特征面积；
        Dsm = 0;
    end
end
CAjs = jiansusan_c * yita * Sm * jiansusan_num; %%减速伞阻力系数；
Qjs = 0.5 * rho * CAjs * y(4)^2;                %%减速伞气动阻力；
Qwq = 0.5 * rho * CAw * Sq * y(4)^2;            %%空投车辆运动方向气动阻力；
Msf =  rho * jiansu_kf * CAjs^(3/2);               %减速伞附加质量；
dMsf = (3/2) * rho * jiansu_kf * CAjs^0.5 * Dsm;   %附加质量变化率；
M_act = Mw + Mjs ; %空投实际质量；
M_f = Msf;       %牵引伞和减速伞附加质量；
M_all = M_act + M_f;    %空投系统总质量；

dy(1) = 1*y(4)*cos(y(3));                                                 %y(1)表示空投车辆X向位移Xd；
dy(2) = 1*y(4)*sin(y(3));                                                 %y(2)表示空投车辆Y向位移Yd；
dy(3) = 1*g*cos(y(3))/y(4);                                               %y(3)表示姿态角；
dy(4) = g*sin(y(3))*M_act/M_all - (Qjs+Qwq)/M_all - y(4)*dMsf/M_all;      %y(4)表示空投车辆速度Vw；
% dy(5) = y(6);   %y(5)表示空降车俯仰角
% dy(6) = Tw/hull_Izz; %y(5)表示空降车俯仰角速度     
end

