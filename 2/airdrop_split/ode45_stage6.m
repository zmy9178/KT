function dy = ode45_stage6(t,y)
% ODE45_STAGE6 主伞充气阶段的微分方程右端。
%
% 输入 y = [Xd, Yd, theta, Vw]。
% 使用全局 t21 描述主伞二次充气时长，并按 t1/t2/t3 三段计算主伞有效面积 Sm。
% 包含主伞阻力、减速伞阻力、车体阻力、附加质量及其变化率。

%%参数赋初值
global Vehicle drop_envi mainchute decechute data_stage_55  t21 
density_air = drop_envi.AirDensity;
grav = drop_envi.Grav;
resist_hull = Vehicle.resist_hull;
drop_mass = Vehicle.dropmass;
jiansusan_aera = decechute.aera;
hull_L = Vehicle.hull_L;
hull_W = Vehicle.hull_W; 
hull_H = Vehicle.hull_H; 
jiansusan_mass = decechute.mass;
jiansusan_num = mainchute.ChuteNum;
jiansusan_c = decechute.resist;
mass_chuteclothes = mainchute.ChuteMass;
aera_mainchute = mainchute.ChuteAera;
k_ks = mainchute.k_ks;
drop_hight = drop_envi.drophight;
number_mainchute = mainchute.ChuteNum;
c_mainchute = mainchute.resist_ca;
kf = mainchute.kf;
khalf = mainchute.khalf;
step_open  = mainchute.StepOpen;

%%%%定义参数%%%%
D = 6.5*1E-3;    %大气温度梯度系数；
T0= 273;         %气温273k；
h = drop_hight - y(2);
rho = density_air*(1-D*h/T0)^4.26;  %%空气密度(随高度变化)；
g = grav;           %%重力加速度；
yita = 1.05 - 0.05 * number_mainchute; %伞群效率；
CAw = resist_hull;     %%空投物阻力系数；
Mw = drop_mass*1000;    %%空投车辆质量；
Mms = mass_chuteclothes;    %主伞质量；
Mjs = jiansusan_mass ; %减速伞质量；
% b1 = hull_cmy-ini_loady;
% a1 = hull_cmx;
dy = zeros(4,1);
Sq =  (hull_H*abs(-pi/2+y(3)))+hull_L*cos(-pi/2+y(3))*hull_W;
%%%基础计算
m =  length(t);
% VL = data_stage_5(5,1)-data_stage_5(4,1);
CAmst = c_mainchute * aera_mainchute;              %%主伞阻力系数；
ks = aera_mainchute * khalf;
VL1 = data_stage_55(5,1);
t1 = k_ks*sqrt(khalf*CAmst)/VL1;
t2 = t1+step_open;
t3 = t2 + t21;
for i = 1:m
    if t(i) <= t1
        Sm = ks * t(i)/t1;
        Dsm = ks/t1;
    elseif t(i) <= t2
        Sm = ks;
        Dsm = 0;
    elseif t(i) <= t3
        Sm =  ks + aera_mainchute * (1-khalf) * ((t(i)-t2)/t21)^2;
        Dsm = 2.5*aera_mainchute * (1-khalf)*((t(i)-t2)/t21);
    else
        Sm = aera_mainchute;                    %主伞特征面积；
        Dsm = 0;
    end
end
CAms = c_mainchute * yita * Sm * number_mainchute;              %%主伞阻力系数；
yita_jiansu = 1.05 - 0.05 * jiansusan_num ; %减速伞伞群效率；
CAjs = jiansusan_c * yita_jiansu * jiansusan_aera * jiansusan_num;  %%减速伞阻力系数；
Qjs = 0.5 * rho * CAjs * y(4)^2;             %%减速伞气动阻力；
Qms = 0.5 * rho * CAms * y(4)^2;             %%主伞气动阻力；
Qwq = 0.5 * rho * CAw * Sq * y(4)^2;         %%空投车辆运动方向气动阻力；
Msf =  rho * kf * CAms^(3/2);                %主伞附加质量；
dMsf = (3/2) * rho * kf * CAms^0.5 * Dsm;    %附加质量变化率；

M_act = Mw + Mjs + Mms ; %空投实际质量；
M_f = Msf ;              %主伞附加质量；
M_all = M_act + M_f;    %空投系统总质量；

dy(1) = 1*y(4)*cos(y(3));                                                     %y(1)表示空投车辆X向位移Xd；
dy(2) = 1*y(4)*sin(y(3));                                                     %y(2)表示空投车辆Y向位移Yd；
dy(3) = 1*g*cos(y(3))/y(4);                                                   %y(3)表示姿态角；
dy(4) = g*sin(y(3))*M_act/M_all - (Qms+Qwq+Qjs)/M_all - y(4)*dMsf/M_all;      %y(4)表示空投车辆速度Vw；
% dy(5) = y(6);   %y(5)表示空降车俯仰角
% dy(6) = Tw/hull_Izz; %y(5)表示空降车俯仰角速度                                                                                           
end

