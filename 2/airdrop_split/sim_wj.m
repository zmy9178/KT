function [time_wj,data_wj,Xx,Yy,Zz,Vx,Vy,Vz] = sim_wj(data_mscq,wj_para,data_66)
% SIM_WJ 主伞充气结束后的稳降阶段。
%
% 物理含义：将主伞-车辆系统简化为带摆角的稳降动力学模型，考虑水平风、侧向风，
% 并在积分后把体轴系位移/速度转换为空间坐标系输出。
%
% 输入：
%   data_mscq: 主伞充气结束状态。
%   wj_para: [wind_x, wind_y, wind_z]，当前顶层 wind_z 固定为 0。
%   data_66: 主伞充气阶段速度/轨迹角历程，用末点初始化稳降水平速度。
%
% 输出包含稳降时间、内部状态、空间位移 Xx/Yy/Zz 和速度 Vx/Vy/Vz。

global drop_envi mainchute Vehicle pullchute decechute 
x_wind =wj_para(1);%X向风速
y_wind = wj_para(2);%Y向风速
z_wind =wj_para(3);%Z向风速
sita = data_mscq(1,3)*180/pi;%俯仰角；
beta = 0;%侧倾角β
w = data_mscq(1,7);%俯仰角速度
u = 0;%侧倾角速度
Vx = data_mscq(1,4);%空降车速度（物伞坐标系）
X = data_mscq(1,1);%车体X向位移（地面坐标系）
Y = 0;%车体侧向位移
Z = data_mscq(1,2);%车体垂直位移
g = drop_envi.Grav;%g = 9.8;   %重力加速度
H = drop_envi.drophight;    %高度
drop_mass = Vehicle.dropmass;
mass_ms_side = mainchute.SideMass; %主伞伞边加强带质量
width_vehicle = Vehicle.hull_W;%车辆宽度；
height_vehicle = Vehicle.hull_H;%车辆高度；
length_vehicle = Vehicle.hull_L;%车辆长度；
ca_vehicle = 0.6;%车辆阻力系数；
cn_vehicle = 0.6;%车辆阻力系数；
cm_vehicle = 0.4;%车辆阻力系数；
ca_mainchute = mainchute.resist_ca;%主伞轴向阻力系数
cn_mainchute = mainchute.resist_cn;%主伞法向阻力系数；
cm_mainchute = mainchute.resist_cm;%主伞侧向阻力系数；
numb_mainchute = mainchute.ChuteNum; %主伞数量
mass_ms_rope = mainchute.RopeMass; %主伞伞绳质量
mass_ms_cloth = mainchute.ChuteMass;%主伞伞衣质量
aera_mainchute = mainchute.ChuteAera ;%主伞特征面积
L = mainchute.RopeLength;%伞绳长度
A = aera_mainchute *numb_mainchute;%主伞群特征面积
CA = aera_mainchute  *ca_mainchute * (1.05 - numb_mainchute * 0.05)*numb_mainchute;%主伞群轴向特征阻力系数
CA22 = aera_mainchute  *cn_mainchute * (1.05 - numb_mainchute * 0.05)*numb_mainchute;%主伞群侧向特征阻力系数
CA33 = aera_mainchute  *cm_mainchute * (1.05 - numb_mainchute * 0.05)*numb_mainchute;%主伞群法向特征阻力系数
CA3 = ca_vehicle*length_vehicle*width_vehicle;%车体法向阻力特征系数
CA2 = cn_vehicle*length_vehicle*height_vehicle;%车体侧向阻力特征系数
CA1 = cm_vehicle*width_vehicle*height_vehicle;%车体轴向阻力特征系数
m = (mass_ms_rope + mass_ms_side  + mass_ms_cloth)*numb_mainchute;%主伞群总质量
M = Vehicle.dropmass*1000-m-pullchute.mass-(decechute.ChuteSideMass+decechute.mass+decechute.RopeMass*decechute.RopeNum);  %空降车质量
Ic = m*M*(L^2)/(m+M);%系统的转动惯量
params = struct();
params.H = H;
params.drop_mass = drop_mass;
params.m = m;
params.M = M;
params.L = L;
params.g = g;
params.aera_mainchute = aera_mainchute;
params.CA = CA;
params.CA1 = CA1;
params.CA2 = CA2;
params.CA3 = CA3;
params.CA22 = CA22;
params.CA33 = CA33;
params.Ic = Ic;
params.wind_x = x_wind;
params.wind_y = y_wind;
params.wind_z = z_wind;
params.Z = Z;
beta0 = beta;
theta0 = sita;
u0 = u;
w0 = w;
x_body0 = 0;
vx_body0 = Vx;
A = data_66(length(data_66),1)*cosd(data_66(length(data_66),2));

Y0 = [beta0;theta0;u0;w0;x_body0;vx_body0;A;0;0;0];
simtime = [0 200];
[time,data] = ode45(@(t,Y)dynsys(t,Y,params),simtime,Y0);
ax = zeros(length(time),1);
for i = 1:length(time)
    [~,ax_body_val] = dynsys(time(i),data(i,:)',params);
    ax(i) = ax_body_val;
end
% n = length(time);
% Xx = zeros(n,1);
% Yy = zeros(n,1);
% Zz = zeros(n,1);
% time_wj = zeros(n,1);
% data_wj = zeros(n,8);
% Vx = zeros(n,1);
% Vy = zeros(n,1);
% Vz = zeros(n,1);
for i = 1:length(time)
    theta_w = data(i,2);
    beta_w = data(i,1);
    R = [-cosd(theta_w)*cosd(beta_w),-sind(beta_w),-sind(theta_w)*cosd(beta_w);
    -cosd(theta_w)*sind(beta_w),cosd(beta_w),-sind(theta_w)*sind(beta_w);
    -sind(theta_w),0,cosd(theta_w)];
    weyi = R'*[data(i,5);0;0];
    V_w = R'*[data(i,6);0;0];
    if -weyi(3,1)+Z<=params.H
        %+drop_envi.Land_hight;
        time_wj(i) = time(i);
        data_wj(i,1) = data(i,1);
        data_wj(i,2) = data(i,2);
        data_wj(i,3) = data(i,3);
        data_wj(i,4) = data(i,4);
        data_wj(i,5) = data(i,5);
        data_wj(i,6) = data(i,6);
        data_wj(i,7) = ax(i)/g;
        data_wj(i,8) = atan(data(i,2)/data(i,1));
        Vx(i) = data(i,7);
        Vy(i) = data(i,8);
        Vz(i) = -V_w(3,1);
        Xx(i) = data(i,9)+X;
        Yy(i) = data(i,10)+Y;
        Zz(i) = -weyi(3,1)+Z;
    end
end
end
