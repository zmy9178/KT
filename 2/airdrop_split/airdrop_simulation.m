function [time_all,data_all] = airdrop_simulation(kte_p,vel_p,qys_p,jss_p,zhs_p,sim_p)
% AIRDROP_SIMULATION 空投全过程仿真顶层编排函数。
%
% 本函数按固定顺序串联 7 个物理阶段：出舱滑动、出舱翻转、减速伞拉直、
% 减速伞充气、主伞拉直、主伞充气、稳降。每个阶段以前一阶段的终止状态
% 作为下一阶段初始条件，最后统一拼接为 time_all 和 data_all。
%
% 输入参数仍保持原 MATLAB 向量接口，方便与旧调用方和 Excel 数据对齐：
%   kte_p: 环境参数，长度 9。
%   vel_p: 空降车参数，长度 19。
%   qys_p: 牵引伞参数，长度 4。
%   jss_p: 减速伞参数，长度 11。
%   zhs_p: 主伞参数，长度 19。
%   sim_p: 仿真时间与采样步长，长度 2。
%
% 输出：
%   time_all: 完整时间轴。开伞前按 sim.step 固定采样，稳降段为 ode45 自适应时间。
%   data_all: 9 列输出，[X位移,Y/高度,Z位移,X速度,Y速度,Z速度,俯仰角,侧倾角,过载]。
%
% 迁移提示：当前函数大量写入 global 结构体。迁移到 Python/C++ 时，建议先创建
% SimulationContext，并把 drop_envi、Vehicle、pullchute、decechute、mainchute、sim
% 作为显式上下文传入各阶段函数，避免隐藏依赖。

%%本程序用于计算牵引出舱、开伞过程仿真计算。
% 坐标原点为空降车车首最前端的中点。
%%输入参数说明：
% kte_p = [0,800,320,0,5.75,-1.24,6,7,25];
% vel_p = [15.2,4.96E4,2.64E4,-3.279,-0.407,5.6,3.2,2.2,0.506,4,1.614,-0.34,-0.698,-1.209,-4.9,-5.75,-4.9,0.83,0.05];
% qys_p = [20.2,0.8,14,5.55];
% jss_p = [10,0.8,6.8,0.01,2,8,22,4,1,8,0.9];
% zhs_p = [760,9,58.767,0.026,6.618,1,0.8,0.8,0.8,0.6,200,56,24.734,80.425,1,0.16,23.5,4.3,5];
% sim_p = [50,0.001];

%输出参数说明：
%time_max——自出舱至主伞充气结束总时间（s）；
%data_max(1)——KT时间历程中空降车最大的水平速度（m/s）；
%data_max(2)——KT时间历程中空降车最大的垂向速度（m/s）；
%data_max(3)——KT时间历程中空降车最大的侧向速度（m/s）；
%data_max(4)——KT时间历程中空降车最大的牵引绳过载（g）；
%data_max(5)——KT时间历程中空降车最大的俯仰角（deg）；
%data_max(6)——KT时间历程中空降车最大的侧倾角（deg）；
%
%data_all(1)——KT时间历程中空降车的（X向）水平速度（m/s）；
%data_all(2)——KT时间历程中空降车的（Y向）垂向速度（m/s）；
%data_all(3)——KT时间历程中空降车的（Z向）侧向速度（m/s）；
%data_all(4)——KT时间历程中空降车的车体速度（m/s）；
%data_all(5)——KT时间历程中空降车的（X向）水平位移（m）；
%data_all(6)——KT时间历程中空降车的（Y向）垂向位移（m）；
%data_all(7)——KT时间历程中空降车的（Z向）侧向位移（m）；
%data_all(8)——KT时间历程中空降车的轨迹角（deg）；
%data_all(9)——KT时间历程中空降车的俯仰角（deg）；
%data_all(10)——KT时间历程中空降车的侧倾角（deg）；
%data_all(11)——KT时间历程中空降车的俯仰角角速度（deg/s）；
%data_all(12)——KT时间历程中空降车的侧倾角角速度（deg/s）；
%data_all(13)——KT时间历程中牵引绳过载（g）；
%time_all,data_all为输出参数的时间历程，用于绘制曲线（主伞充气过程）。
%%%空投环境参数结构体定义：
global drop_envi Vehicle pullchute decechute mainchute sim
land_hight = kte_p(1);                %%空投着陆场海拔，默认为0（m）；
Earth_Mass = 5.97E24;
Earth_Radius = 6370000;
G = 6.67E-11;
Rs = 287;
p0 = 101325;                   
P = p0*exp((-1*land_hight/7924));
T = 273.15 + kte_p(9);
AirDensity = P / (Rs * T);
grav = (G*Earth_Mass)/(Earth_Radius+land_hight)^2;
drop_envi.Land_hight = land_hight ;
drop_envi.Grav = grav;                %%重力加速度，默认为9.817;
drop_envi.Earth_R = Earth_Radius;     %%地球半径，默认为6370000;
drop_envi.AirDensity = AirDensity;    %%空气密度，默认为1.29;
drop_envi.drophight = kte_p(2);       %%空投高度，默认为800;
drop_envi.AirplaneVelo = kte_p(3);    %%空投运输机航速，默认为320;
drop_envi.Cargo_angle = kte_p(4);     %%运输机货舱与水平面夹角，默认为0;
drop_envi.LoadPosi_x = abs(kte_p(5));   %%空降车货舱内X装载坐标，默认为-5.75;
drop_envi.LoadPosi_y = kte_p(6);      %%空降车货舱内Y装载坐标，默认为-1.24;
drop_envi.wind_x = kte_p(7);          %%水平风速;
drop_envi.wind_y = kte_p(8);          %%侧向风速;
drop_envi.T = kte_p(9);               %%地面温度;
%%%空投环境参数结构体结束。
 %%%空降车参数结构体定义：
Vehicle.dropmass = vel_p(1);     %空投总质量，默认为15.2t;
Vehicle.Izz = vel_p(2);          %空降车Z向转动惯量，默认为4.96E4；
Vehicle.Ixx = vel_p(3);          %空降车X向转动惯量，默认为2.64E4；
Vehicle.hull_cmx = -1*vel_p(4);  %空降车质心位置X向坐标，默认为-3.279；
Vehicle.hull_cmy = vel_p(5);     %空降车质心位置X向坐标，默认为-0.407；   
Vehicle.hull_L = vel_p(6);       %空降车长度；默认为5.6；
Vehicle.hull_W = vel_p(7);       %空降车宽度，默认为3.2；
Vehicle.hull_H = vel_p(8);       %空降车高度，默认为2.2；
Vehicle.cmpull_y = vel_p(9);     %吊钩与车辆重心的垂直距离，默认为0.506；
Vehicle.cmpull_H = vel_p(10);    %吊钩与吊点的垂直距离，默认为4；
Vehicle.cmpull_L = vel_p(11);    %吊钩与车辆重心的水平距离，默认为1.614；
Vehicle.pull_x = -1*vel_p(12);   %牵引伞牵引点X坐标，默认为-0.34；
Vehicle.pull_y = vel_p(13);      %牵引伞牵引点Y坐标，默认为-0.698；
Vehicle.track_fdx = vel_p(14);   %下支履带前端X坐标，默认为-1.209；
Vehicle.track_fdy = vel_p(15);   %下支履带前端Y坐标，默认为-4.9；
Vehicle.track_bdx = vel_p(16);   %下支履带后端X坐标，默认为-5.75；
Vehicle.track_bdy = vel_p(17);   %下支履带后端Y坐标，默认为-4.9；
Vehicle.resist_hull = vel_p(18); %空降车空气阻力系数，默认为0.83；
Vehicle.mu = vel_p(19);          %空降车与货舱地板之间的滑动摩擦系数，默认为0.05；
%%%空降车参数结构体end.
%%%牵引伞参数结构体定义：
pullchute.mass = qys_p(1);         %牵引伞质量，默认为20.2；
pullchute.resist = qys_p(2);       %牵引伞阻力系数，默认为0.8；
pullchute.area = qys_p(3);         %牵引伞特征面积，默认为14；
pullchute.RopeLength = qys_p(4);   %牵引伞伞绳长度，默认为5.55；
%%%牵引伞参数结构体end
%%%减速伞参数结构体定义：
decechute.mass =  jss_p(1);            %减速伞伞衣质量,默认为10；
decechute.resist = jss_p(2);           %减速伞阻力系数,默认为0.8；
decechute.aera = jss_p(3);             %减速伞等效面积,默认为6.8；
decechute.ChuteWidth =  jss_p(4);      %减速伞伞边宽度,默认为0.01；
decechute.ChuteSideMass =  jss_p(5);   %减速伞伞边质量,默认为2；
decechute.RopeNum =  jss_p(6);         %减速伞伞绳数量,默认为8；
decechute.RopeLength = jss_p(7);       %减速伞伞绳长度,默认为22；
decechute.RopeMass =  jss_p(8);        %减速伞伞绳质量,默认为4；
decechute.kf =  jss_p(9);              %减速伞附加质量系数,默认为1;
decechute.ks =  jss_p(10);             %减速伞充气系数,默认为8；
decechute.ksn = jss_p(11);             %减速伞开伞充气指数,默认为0.9；
%%%减速伞参数结构体结束。
%%%主伞参数结构体定义：
mainchute.ChuteAera = zhs_p(1);         %主伞特征面积，默认值为760
mainchute.ChuteNum =  zhs_p(2);         %主伞数量，默认值为9
mainchute.ChuteMass = zhs_p(3);         %主伞伞衣质量，默认值为58.767
mainchute.SideWidth = zhs_p(4);         %主伞伞边宽度，默认值为0.026 
mainchute.SideMass =  zhs_p(5);         %主伞伞边质量，默认值为6.618
mainchute.resist_ca = zhs_p(6);         %主伞法向阻力系数，默认值为1.0
mainchute.resist_cn = zhs_p(7);         %主伞周向阻力系数，默认值为0.8
mainchute.resist_cm = zhs_p(8);         %主伞径向阻力系数，默认值为0.8
mainchute.ChuteCoverResist = zhs_p(9);  %主伞伞衣套阻力系数，默认值为0.8
mainchute.PulloutResist = zhs_p(10);    %主伞已拉出物体阻力系数，默认值为0.6
mainchute.PulloutForce = zhs_p(11);     %主伞拉出阻力，默认值为100 ;
mainchute.RopeNum = zhs_p(12);          %主伞伞绳数量，默认值为56
mainchute.RopeMass = zhs_p(13);         %主伞伞绳质量，默认值为24.734
mainchute.RopeLength = zhs_p(14);       %主伞伞绳长度，默认值为80.425
mainchute.kf = zhs_p(15);               %主伞附加质量系数，默认值为1
mainchute.khalf = zhs_p(16);            %主伞收口比，默认值为0.16
mainchute.k_ks = zhs_p(17);             %主伞第1次充气λm1，默认值为23.5
mainchute.n_ks = zhs_p(18);             %第2次充气λm2，默认值为4.3
mainchute.StepOpen = zhs_p(19);         %2次充气间隔时间，默认值为5
%%%仿真时间步长结构体定义：
sim.time = sim_p(1);   %仿真时间：默认值为50；
sim.step = sim_p(2);      %仿真时间步长：默认值为0.001；
%%%仿真时间步长结构体定义end.
% density_rope = mass_rope / length_rope;
% density_chuteside = density_chuteside_single * number_mainchute;
% length_chuteall = chuteside_width + length_rope + diam_chutechothes/2;
%%%%%%%%%    接收用户输入初始参数 ——仿真计算参数及初始条件   %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%出舱过程计算：
% 接收参数：
hull_cmx = Vehicle.hull_cmx;
hull_cmy = Vehicle.hull_cmy;
ini_loadx = Vehicle.track_fdx;
track_fdy = Vehicle.track_fdy;
track_fdx = Vehicle.track_fdx;
track_bdx = Vehicle.track_bdx;
plane_angle = drop_envi.Cargo_angle;
sim_time = sim.time/5;
step =  sim.step; 
%% 接收参数结束。
%% 出舱过程计算：
para_pdong = [hull_cmx,hull_cmy,ini_loadx,track_fdy,plane_angle,sim_time,step];
para_fanzh = [hull_cmx,hull_cmy,track_fdx,track_fdy,track_bdx,plane_angle,sim_time,step];
[time_pd,data_pd,time_1,data_1] = stage_moving(para_pdong);
[time_fz,data_fz,time_2,data_2] = stage_revote(data_pd,para_fanzh);
%% 出舱过程计算结束。
[~,n1] = size(data_1);
[~,n2] = size(data_2);
t1 = find(time_1==time_pd);
t2 = find(time_2==time_fz);

data_cc1 = zeros(t1,n2);
data_cc1(1:t1,1:n1) = data_1(1:t1,1:n1);
data_cc2 = data_2(1:t2,:);

data_cc = cat(1,data_cc1,data_cc2);
% plot(time_cc,data_cc)
% data_cc(:,1) %%%空降车x向位移
% data_cc(:,2) %%%空降车X向速度
% data_cc(:,3) %%%空降车Y向位移；
% data_cc(:,4) %%%空降车Y向速度
% data_cc(:,5) %%%空降车俯仰角；
% data_cc(:,6) %%%空降车俯仰角速度。
%%出舱过程计算end
%%减速伞打开过程计算：
% 接收参数：
pullchute_L = pullchute.RopeLength;
velo_plane = drop_envi.AirplaneVelo;
hull_Izz = Vehicle.Izz;
hull_H = Vehicle.hull_H;
hull_L = Vehicle.hull_L;
hull_W = Vehicle.hull_W;
ini_loady = Vehicle.track_fdy;
track_fdy = ini_loady;
density_air = drop_envi.AirDensity;
cmpull_y = Vehicle.cmpull_y;
cmpull_L = Vehicle.cmpull_L;
cmpull_H = Vehicle.cmpull_H;
force_pullout = mainchute.PulloutForce;
number_rope = decechute.RopeNum;
resist_hull = Vehicle.resist_hull;
jiansusan_length = decechute.RopeLength;
jiansusan_aera = decechute.aera;
jiansusan_num = mainchute.ChuteNum;
jiasu_width = decechute.ChuteWidth;
jiasurope_mass = decechute.RopeMass;
jiansusan_mass = decechute.mass;
jiansu_density_side = decechute.ChuteSideMass / decechute.ChuteWidth;
drop_hight = drop_envi.drophight;
jiansusan_c = decechute.resist;
jiansu_kf = decechute.kf;
grav = drop_envi.Grav;
k_ks = decechute.ks;
step = sim.step;
sim_time = sim.time;
% 接收参数结束。
%%减速伞打开过程计算：
js_strait_para = [pullchute_L,velo_plane,hull_Izz,hull_H,hull_L,hull_W,hull_cmy...
   ini_loady,density_air,hull_cmx,cmpull_y,cmpull_L,cmpull_H, force_pullout...
   number_rope,track_fdy,resist_hull,jiansusan_length,jiansusan_aera...
   jiansusan_num,jiasu_width,jiasurope_mass,jiansu_density_side,jiansusan_mass...
    step,sim_time];
js_chuqi_para = [hull_cmx,resist_hull,hull_cmy,ini_loady,hull_H,hull_L,hull_W...
    hull_Izz,cmpull_y,cmpull_L,cmpull_H,jiansusan_num,jiansusan_mass,jiansusan_aera,...
    drop_hight,jiansusan_c,jiansu_kf,density_air,grav,k_ks,sim_time,step];
[time_jslz,data_jslz,time_3,data_3,data_33] = jiansu_straight(data_fz,js_strait_para);
global data_stage_3
data_stage_3 = data_jslz;
[time_jscq,data_jscq,time_4,data_4,data_44] = jiansu_chongqi(data_jslz,js_chuqi_para);
data_v3 = zeros(length(data_33),2);
data_v4 = zeros(length(data_44),2);
for i = 1:length(data_33)
    data_v3(i,1) = data_33(i,1)*cosd(data_33(i,2));
    data_v3(i,2) = data_33(i,1)*sind(data_33(i,2));
end
for i = 1:length(data_44)
    data_v4(i,1) = data_44(i,1)*cosd(data_44(i,2));
    data_v4(i,2) = data_44(i,1)*sind(data_44(i,2));
end
%%减速伞打开过程计算结束。
%%主伞打开过程计算：
%%接收参数：
aera_mainchute = mainchute.ChuteAera;
diam_chutechothes = 2*sqrt(aera_mainchute/pi);
mass_chuteclothes = mainchute.ChuteMass;
c_mainchute = mainchute.resist_ca;
number_mainchute = mainchute.ChuteNum;
step_open = mainchute.StepOpen;
khalf = mainchute.khalf;
n_ks = mainchute.n_ks;
mass_parachuter = pullchute.mass;
drop_mass = Vehicle.dropmass;
length_rope = decechute.RopeLength;
chuteside_width = mainchute.SideWidth;
length_chuteall = chuteside_width + length_rope + diam_chutechothes/2;

ms_strait_para = [hull_cmx,hull_cmy,ini_loady,track_fdy,cmpull_y,cmpull_L,cmpull_H...
   hull_Izz,hull_H,hull_L,hull_W,resist_hull,jiansusan_mass,diam_chutechothes...
   mass_chuteclothes,pullchute_L,velo_plane,density_air,force_pullout,number_rope...
   length_rope,jiansusan_length,jiansusan_aera,jiansusan_num,length_chuteall,...
   jiasu_width,jiasurope_mass,jiansu_density_side,sim_time,step];
ms_chuqi_para = [drop_hight,drop_mass,cmpull_y,cmpull_L,cmpull_H,resist_hull,...
   hull_cmy,ini_loady,hull_cmx,hull_H,hull_L,hull_W,hull_Izz,mass_parachuter,...
   mass_chuteclothes,aera_mainchute,c_mainchute,number_mainchute,grav,density_air...
   step_open,khalf,k_ks,n_ks,sim_time,step];
[time_mslz,data_mslz,time_5,data_5,data_55] = zhusan_straight(data_jscq',ms_strait_para,data_jslz);
global data_stage_55
data_stage_55 = data_jslz;
[time_mscq,data_mscq,time_6,data_6,data_66] = zhusan_chongqi(data_mslz,ms_chuqi_para);
data_v5 = zeros(length(data_55),2);
data_v6 = zeros(length(data_66),2);
for i = 1:length(data_55)
    data_v5(i,1) = data_55(i,1)*cosd(data_55(i,2));
    data_v5(i,2) = data_55(i,1)*sind(data_55(i,2));
end
for i = 1:length(data_66)
    data_v6(i,1) = data_66(i,1)*cosd(data_66(i,2));
    data_v6(i,2) = data_66(i,1)*sind(data_66(i,2));
end
%%稳降过程计算：
global ax_body_global
wj_para = [drop_envi.wind_x,drop_envi.wind_y,0];
[time_wj,data_wj,Xx,Yy,Zz,Vx,Vy,Vz] = sim_wj(data_mscq,wj_para,data_66);
%indices = round(linspace(1,length(ax_body_global),length(time_wj)));
%ax = ax_body_global(indices,:);
length(data_v3);
%%稳降过程计算end
%%主伞打开过程计算end

KjVehicle_Dx = cat(1,-1*data_cc(:,1),data_3(:,1),data_4(:,1),data_5(:,1),data_6(:,1));
KjVehicle_Dy = cat(1,data_cc(:,3),data_3(:,2),data_4(:,2),data_5(:,2),data_6(:,2));
KjVehicle_Velo_x = cat(1,velo_plane/3.6+data_cc(:,2),data_v3(:,1),data_v4(:,1),data_v5(:,1),data_v6(:,1));%%
KjVehicle_Velo_y = cat(1,data_cc(:,4),data_v3(:,2),data_v4(:,2),data_v5(:,2),data_v6(:,2));%%
zero_data_cc = zeros(length(data_cc),1);
Vehicle_Chute_Angle = cat(1,zero_data_cc,data_3(:,3),data_4(:,3),data_5(:,3),data_6(:,3));
Vehicle_Pitch_Angle = cat(1,data_cc(:,5),data_3(:,7),data_4(:,5),data_5(:,7),data_6(:,6));%
Vehicle_Pitch_Anglew = cat(1,data_cc(:,6),data_3(:,8),data_4(:,6),data_5(:,8),data_6(:,7));
Chute_Rope_Force = cat(1,zero_data_cc,data_3(:,9),data_4(:,7),data_5(:,9),data_6(:,5));
KjVehicle_Velo = cat(1,velo_plane/3.6+data_cc(:,2),data_33(:,1),data_44(:,1),data_55(:,1),data_66(:,1));
time_1 = [0:step:(length(KjVehicle_Dx)-1)*step]';
time_all = [time_1;time_wj'+max(time_1)];

data = [KjVehicle_Dx,KjVehicle_Dy,Vehicle_Chute_Angle*180/pi,KjVehicle_Velo_x,...
    Chute_Rope_Force/9.8/drop_mass/1000,Vehicle_Pitch_Angle*180/pi,Vehicle_Pitch_Anglew*180/pi,KjVehicle_Velo_y,KjVehicle_Velo];
Vx_all = cat(1,data(:,4),Vx');                                  %X向速度（水平速度）
Vy_all = cat(1,data(:,8),Vz');                                  %Y向速度（垂直速度）
Vz_all = cat(1,zeros(size(data(:,4))),Vy');                     %Z向速度（侧向速度）

Xx_all = cat(1,KjVehicle_Dx,Xx');                               %X向位移（水平位移）
Yy_all = cat(1,drop_hight-data(:,2),drop_hight-Zz');            %Y向位移（垂向位移）
Zz_all = cat(1,zeros(size(KjVehicle_Dx)),Yy');                  %Z向位移（侧向位移）

theta_all = cat(1,data(:,6),90-data_wj(:,2));                   %俯仰角(角度制)
beta_all = cat(1,zeros(size(data(:,6))),data_wj(:,1));          %侧倾角(角度制)


Force_all = cat(1,data(:,5),abs(data_wj(:,7)/drop_mass/1000));  %牵引绳过载（/g）
data_all = [Xx_all,Yy_all,Zz_all,Vx_all,Vy_all,Vz_all,theta_all,beta_all,Force_all];
max(theta_all);
%%%绘图；
 %figure(1)
% figure('Position',[80,0,1400,800]);
%    subplot(3,3,1) 
%     plot(time_all,data_all(:,1),'r');
%     set(gca,'XLim',[0 max(time_all)]);
%        xlabel('时间/s');
%       ylabel('位移/m');
%       title('空降车X向位移时间历程');
%       grid on 
%       subplot(3,3,2) 
%         plot(time_all,data_all(:,3),'r');
%         set(gca,'XLim',[0 max(time_all)]);
%          xlabel('时间/s');
%         ylabel('位移/m');
%         title('空降车Z向位移时间历程')
%        grid on 
%     subplot(3,3,3) 
%         plot(time_all,data_all(:,2),'r');
%         set(gca,'XLim',[0 max(time_all)]);
%         xlabel('时间/s');
%        ylabel('位移/m');
%        title('空降车Y向位移时间历程')
%        grid on 
%   subplot(3,3,4)
%         plot(time_all,data_all(:,4),'r');
%         set(gca,'XLim',[0 max(time_all)]);
%         xlabel('时间/s');
%         ylabel('速度/m/s');
%         title('空降车X向速度时间历程')
%         grid on 
%    subplot(3,3,5) 
%         plot(time_all,data_all(:,6),'r');
%         set(gca,'XLim',[0 max(time_all)]);
%         xlabel('时间/s');
%         ylabel('速度/m/s');
%         title('空降车Z向速度时间历程')
%         grid on 
%     subplot(3,3,6) 
%         plot(time_all,data_all(:,5),'r');
%         set(gca,'XLim',[0 max(time_all)]);
%         xlabel('时间/s');
%         ylabel('速度/m/s');
%         title('空降车Y向速度时间历程')
%         grid on 
%     subplot(3,3,7) 
%         plot(time_all,data_all(:,7),'r');
%         set(gca,'XLim',[0 max(time_all)]);
%         xlabel('时间/s');
%         ylabel('角度/deg');
%         title('空降车俯仰角时间历程');
%         grid on 
%     subplot(3,3,8) 
%         plot(time_all,data_all(:,8),'r');
%         set(gca,'XLim',[0 max(time_all)]);
%         xlabel('时间/s');
%         ylabel('角度/deg');
%         title('空降车侧倾角时间历程')
%         grid on 
%     subplot(3,3,9) 
%          plot(time_all,data_all(:,9),'r');
%         set(gca,'XLim',[0 max(time_all)]);
%         xlabel('时间/s');
%        ylabel('开伞过载/g');
%        title('开伞过载曲线')
%        grid on 
 %saveas(figure(1),'主伞开伞');
end

