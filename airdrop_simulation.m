function [time_all,data_all] = airdrop_simulation(kte_p,vel_p,qys_p,jss_p,zhs_p,sim_p)
%%本程序用于计算牵引出舱、开伞过程仿真计算。
% 坐标原点为空降车车首最前端的中点。
%%输入参数说明：
% kte_p = [0,800,320,0,5.75,-1.24,6,7,25];
% vel_p = [15.2,4.96E4,2.64E4,-3.279,-0.407,5.6,3.2,2.2,0.506,4,1.614,-0.34,-0.698,-1.209,-4.9,-5.75,-4.9,0.83,0.05];
% qys_p = [20.2,0.8,14,5.55];
% jss_p = [10,0.8,6.8,0.01,2,8,22,4,1,8,0.9];
% zhs_p = [760,9,58.767,0.026,6.618,1,0.8,0.8,0.8,0.6,200,56,24.734,80.425,1,0.16,23.5,4.3,5];
% sim_p = [50,0.001];
% [time_all ,data_all]=airdrop_simulation(kte_p,vel_p,qys_p,jss_p,zhs_p,sim_p);

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
debug_path = 'D:\shixi_zmy\debug_stages.mat';
save(debug_path, ...
     'time_1','data_1',...
     'time_2','data_2',...
     'time_3','data_3',...
     'time_4','data_4',...
     'time_5','data_5',...
     'time_6','data_6');
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


Force_all = cat(1,data(:,5),abs(data_wj(:,7)/drop_mass/1000)); %牵引绳过载（/g）
data_all = [Xx_all,Yy_all,Zz_all,Vx_all,Vy_all,Vz_all,theta_all,beta_all,Force_all];
save(debug_path, 'data_all', '-append');
save('D:shixi_zmy\debug_stages_full.mat', ...
      'kte_p','vel_p','qys_p','jss_p','zhs_p','sim_p', ...
      'time_pd','data_pd','time_fz','data_fz', ...
      'time_1','data_1','time_2','data_2', ...
      'time_jslz','data_jslz','time_3','data_3','data_33', ...
      'time_jscq','data_jscq','time_4','data_4','data_44', ...
      'time_mslz','data_mslz','time_5','data_5','data_55', ...
      'time_mscq','data_mscq','time_6','data_6','data_66', ...
      'time_wj','data_wj','data_cc','data_all');
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

function [time_pd,data_pd,time_1,data_1] = stage_moving(para_pdong)
%%%参数赋初值；
global Vehicle drop_envi sim
lp = Vehicle.hull_cmx;
hull_cmy = Vehicle.hull_cmy;
l0 = Vehicle.track_fdx;
track_fdy = Vehicle.track_fdy;
plane_angle = drop_envi.Cargo_angle;
sim_time = sim.time;
step = sim.step;
%%%参数赋初值结束
%%%参数赋初值；
hm = hull_cmy -track_fdy ;       %车辆重心至下平面的距离(mm)
alfa = plane_angle*pi/180;   %货舱与水平面夹角，逆时针为正；
simtime = [0:step:sim_time/4]; 
%设定初始值
initial_value= zeros(4,1);
initial_value(1,1) = 0;%x位移初值
initial_value(2,1) = 0;%X向速度初值；
initial_value(3,1) = -1*(l0 + lp)*sin(alfa) - hm*cos(alfa);%Y向位移初值；
initial_value(4,1) = 0;%Y向速度初值
fun = @ode45_stage1;
[time_1,data_1] = ode45(fun,simtime,initial_value);
 n = length(time_1);
 j = 0;
for i =1:n
   if  data_1(i,1)+ drop_envi.LoadPosi_x<=0%改动处
       j = i;
        break;
   end  
end
 time_pd = time_1(j);
 data_pd = zeros(4,1);
 data_pd(1,1) = data_1(j,1);
 data_pd(2,1) = data_1(j,2);
 data_pd(3,1) = data_1(j,3);
 data_pd(4,1) = data_1(j,4);
 end

 % --- Executes on button press in stage_inairmainchute.
function [time_mscq,data_mscq,time,data,data_2] = zhusan_chongqi(data_stage_4,ms_chuqi_para)
% hObject    handdata_stage_3ddata_stage_4,ata_stage_4,nairmainchute (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 drop_hight = ms_chuqi_para(1);
 drop_mass = ms_chuqi_para(2);
 cmpull_y = ms_chuqi_para(3);
 cmpull_L = ms_chuqi_para(4);
 cmpull_H = ms_chuqi_para(5);
 resist_hull = ms_chuqi_para(6);
 hull_cmy = ms_chuqi_para(7);
 ini_loady = ms_chuqi_para(8);
 hull_cmx = ms_chuqi_para(9);
 hull_H = ms_chuqi_para(10);
 hull_L = ms_chuqi_para(11);
 hull_W = ms_chuqi_para(12);
 hull_Izz = ms_chuqi_para(13);
 mass_parachuter = ms_chuqi_para(14);
 mass_chuteclothes = ms_chuqi_para(15);
 aera_mainchute = ms_chuqi_para(16);
 c_mainchute = ms_chuqi_para(17);
 number_mainchute = ms_chuqi_para(18);
 grav = ms_chuqi_para(19);
 density_air = ms_chuqi_para(20);
 step_open = ms_chuqi_para(21);
 khalf = ms_chuqi_para(22);
 k_ks = ms_chuqi_para(23);
 n_ks  = ms_chuqi_para(24);
 sim_time = ms_chuqi_para(25);
 step = ms_chuqi_para(26);

D = 6.5*1E-3;    %大气温度梯度系数；
h = drop_hight - data_stage_4(2,1);
simtime = [0:step:sim_time]';
initial_data = zeros(4,1);
initial_data(1,1) = data_stage_4(1,1);  %X向位移
initial_data(2,1) = data_stage_4(2,1);  %Y向位移；
initial_data(3,1) = data_stage_4(3,1);  %轨迹角
initial_data(4,1) = data_stage_4(5,1);  %空降车速度；
% ini4ial_data(5,1) = data_sta4e_4(6,1);  %车辆俯仰角；
% initial_data(6,1) = data_stage_4(7,1);  %车辆俯仰角速度；
fun = @ode45_stage6;
[time_6,data_6] = ode45(fun,simtime,initial_data);
data_66 = [];
data_66 = data_6;
yita = 1.05 - 0.05 * number_mainchute; %伞群效率；
%yita2 = 1.05 - 0.05 * number_mainchute; %伞群效率；
T0= 273; %气温273k；
rho = density_air*(1-D*h/T0)^4.26; %%空气密度(随高度变化)；
Mw = drop_mass*1000;               %%空投车辆质量；
Mys = mass_parachuter;             %%牵引伞及伞衣套质量；
Mms = mass_chuteclothes;           %%主伞质量；
g = grav;          %%重力加速度；
m =length(time_6);
h = drop_hight;
t = time_6;
M_act = Mw + Mys + Mms ; %空投实际质量；
M_all = M_act;    %空投系统总质量；
dv = zeros(m,1);
Fk = zeros(m,1);
Fk2 = zeros(m,1);
VL1 = data_stage_4(5,1);
CAms = c_mainchute * aera_mainchute;            %%主伞阻力系数；
%CAms2 = c_mainchute * aera_mainchute*yita2*number_mainchute;            %%主伞阻力系数；
t1 = k_ks*sqrt(khalf*CAms)/VL1;
t2 = t1+step_open;
ks = aera_mainchute * khalf;
C = 5000;
for i = 2:m
    if t(i) > t2
        VL2 = data_6(i-1,4);
        break;
    end
end     
global t21
t21 = n_ks*(sqrt(CAms)-sqrt(khalf*CAms))/VL2;  %%二次充气时间计算
t3 = t2 + t21;
for i = 2:m
    if t(i) > t3
        n = i;
        break;
    end
end 
% time_stage4 = t(n);
time_mscq = max(t);
for i=2:m
    if t(i) <= t1
        Sm = aera_mainchute * khalf * t(i)/t1;
        Dsm = aera_mainchute * khalf/t1;
    elseif t(i) <= t2
        Sm = ks;
        Dsm = 0;
    elseif t(i) <= t2 + t1
        Sm =  ks + aera_mainchute * (1-khalf) * ((t(i)-t2)/t21);
        Dsm = aera_mainchute * (1-khalf)/t21;
    else
        Sm = aera_mainchute;    %主伞特征面积；
        Dsm = 0;
    end
    CAms = c_mainchute * yita * Sm;   %%主伞阻力系数；
    dv(i) = (data_6(i-1,4)-data_6(i,4))/(t(i,1)-t(i-1,1));
    Fk(i) = M_all*dv(i) + 0.5*rho*data_6(i,4)^2*CAms  + data_6(i,4)*Dsm + (Mys + Mms)*g*sin(data_6(i,3))+M_all*g;
    %Fk2(i) = (Mys + Mms)*dv(i) + 0.5*rho*data_6(i,4)^2*c_mainchute  + number_mainchute*data_6(i,4)*Dsm + (Mys + Mms)*g*sin(data_6(i,3));
end
 ang_vehicle = zeros(n,1);
 angw_vehicle = zeros(n,1);
 ang_vehicle(1) = data_stage_4(7,1);                                     %空降车俯仰角度；
 angw_vehicle(1) = data_stage_4(8,1);                                    %空降车俯仰角速度；
 gama = atan(cmpull_y/cmpull_L);
 Lat = cmpull_H/sin(gama);

 CAw = resist_hull;     %%空投物阻力系数；
 b1 = hull_cmy-ini_loady;
 a1 = hull_cmx;
 for i = 2:m
     Vmx(i-1) = data_6(i-1,4).* ang_vehicle(i-1);
     Sx(i-1) = (hull_H*cos(ang_vehicle(i-1)) +hull_L*sin(ang_vehicle(i-1) ))*hull_W;
     Shb(i-1) = (b1*cos(ang_vehicle(i-1))+a1*sin(ang_vehicle(i-1)))*hull_W;      %重心以下的迎风面积
     Sht(i-1) =  Sx(i-1) - Shb(i-1);
     Qwb(i-1) = 0.5*density_air*CAw*Shb(i-1)^2*Vmx(i-1)^2/(4*hull_W);              %空降车质心以下气动阻力矩
     Qwt(i-1) = 0.5*density_air*CAw*Sht(i-1)^2*Vmx(i-1)^2/(4*hull_W);              %空降车质心以上气动阻力矩
     QFL(i-1) = Fk(i-1)*Lat*(sin(ang_vehicle(i-1)+data_6(i-1,3)-pi/2))+C*angw_vehicle(i-1)*Lat;
   if ang_vehicle(i-1) + data_6(i-1,3) == pi/2
       Tw(i-1) = Qwt(i-1) - Qwb(i-1);
   else
       Tw(i-1) = Qwt(i-1) - Qwb(i-1)- QFL(i-1);
   end
   acc_angle(i-1) = 1*Tw(i-1)/hull_Izz; 
   angw_vehicle(i) = angw_vehicle(i-1) + acc_angle(i-1) *(time_6(i)-time_6(i-1));
   ang_vehicle(i) = ang_vehicle(i-1) + angw_vehicle(i-1)*(time_6(i)-time_6(i-1));
 end
 j = length(Fk);
 stop_time_i = 0;
 kk = diff(Fk);
 for i = 8/step:j-1
   if abs(kk(i)) <= 1
         stop_time_i = i;
         break;
   end
 end
 time_mscq = time_6(stop_time_i);
 data_mscq = [data_6(stop_time_i,:),Fk(stop_time_i),ang_vehicle(stop_time_i),...
     angw_vehicle(stop_time_i)];
 % ang_vehicle = ang_vehicle - gama;
 %%%%绘图%%%%%%
 time = time_6(1:stop_time_i);
 data = [data_6(1:stop_time_i,:),Fk(1:stop_time_i),ang_vehicle(1:stop_time_i),...
 angw_vehicle(1:stop_time_i)];
data_2 = [data_66(1:stop_time_i,4),data_66(1:stop_time_i,3)];
  end
% --- Executes on button press in mainchutestreight_stage.

function [time_fz data_fz time_2 data_2] = stage_revote(data_stage_1,para_fanzh)

% Hint: get(hObject,'Value') ends toggle state of radiobutton1
hull_cmx = para_fanzh(1);
hull_cmy = para_fanzh(2);
track_fdx = para_fanzh(3);
track_fdy = para_fanzh(4);
track_bdx = para_fanzh(5);
plane_angle = para_fanzh(6);
sim_time = para_fanzh(7);
step = para_fanzh(8);
%%%%输入参数：
%%%% sim_time ——仿真时间；
%%%% initial_value1——第1阶段微分方程初始值；x向位移，x向速度、y向位移、y向速度;
%%%% initial_value2——补充第2阶段微分方程初始值；俯仰角，俯仰角速度;
%%%% 返回参数：
%%%% 返回参数：——微分方程初始值；该阶段终止时刻，下1阶段初值;
%#codegen
hm = hull_cmy-track_fdy;        %车辆重心至下平面的距离(mm)
lt = -track_fdx + track_bdx;       % 履带接地长；
lp = -track_fdx + hull_cmx;        % 车辆质心坐标距离下支履带前支点的距离；
alfa = plane_angle * pi/180;
initial_value = zeros(6,1);
initial_value(1,1) = 0;%data_stage_1(1,1);
initial_value(2,1) = 0;%data_stage_1(2,1);
initial_value(3,1) = data_stage_1(3,1);
initial_value(4,1) = data_stage_1(4,1);
initial_value(5,1) = alfa;
initial_value(6,1) = 0;
simtime = [0:step:sim_time]';
fun = @ode45_stage2;
[time_2,data_2] = ode45(fun,simtime,initial_value);
time = time_2;
data = data_2;
n = 0;
gama =  atan(hm/(lt-lp));
po =sqrt(hm^2+(lt-lp)^2);
x_cm_back = data(:,1) + po*cos(data(:,5)-gama);%+ data_stage_1(1,1)
% lop = (data(:,1)-hm*tan(data(:,5)))./cos(data(:,5))- lt + lp - hm*tan(data(:,5));
for i = 1:length(time)
    if x_cm_back(i) <= 0
        n = i;
        break
    end         
end
time_fz = time(n);
data_fz = zeros(1,6);
data_fz(1,1) = data(n,1)+data_stage_1(1,1);
data_fz(1,2) = data(n,2)+data_stage_1(2,1);
data_fz(1,3) = data(n,3);
data_fz(1,4) = data(n,4);
data_fz(1,5) = data(n,5);        %货舱地板与水平面夹角;
data_fz(1,6) = data(n,6);
 end
% time_stage1,velo_stage1

% --- Executes on button press in stage_straight.
function [time_jslz,data_jslz,time_3,data_out,data_out_2] = jiansu_straight(data_stage_2,js_strait_para)
% hObject    handle to stage_straight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 pullchute_L = js_strait_para(1);
 velo_plane = js_strait_para(2);
 hull_Izz = js_strait_para(3);
 hull_H = js_strait_para(4);
 hull_L = js_strait_para(5);
 hull_W = js_strait_para(6);
 hull_cmy = js_strait_para(7);
 ini_loady =js_strait_para(8);
 density_air = js_strait_para(9);
 hull_cmx = js_strait_para(10);
 cmpull_y = js_strait_para(11);
 cmpull_L = js_strait_para(12);
 cmpull_H = js_strait_para(13);
 force_pullout = js_strait_para(14);
 track_fdy = js_strait_para(16);
 resist_hull = js_strait_para(17);
 jiansusan_length = js_strait_para(18);
 jiansusan_aera =js_strait_para(19);
 jiasu_width = js_strait_para(21);
 jiasurope_mass = js_strait_para(22);
 jiansu_density_side = js_strait_para(23);
 jiansusan_mass = js_strait_para(24);
 step = js_strait_para(25);
 sim_time = js_strait_para(26);
 cmy = hull_cmx;
jiansusan_diam = sqrt(jiansusan_aera/pi)*2;
%参数转换
% nume = pull_y-ini_loady;
nume = data_stage_2(1,3) - (track_fdy - cmy) ;
deno = cmpull_H + pullchute_L ;
initial_data = zeros(0,1);
initial_data(1,1) = -1*data_stage_2(1,1);                      %y(1)表示空降车X向位移；
initial_data(2,1) = -1*data_stage_2(1,3);                      %y(2)表示空降车Y向位移；
initial_data(3,1) = atan(nume/deno);                        %y(3)表示空降车轨迹角；
lvx = velo_plane/3.6 + data_stage_2(1,2);
lvy = data_stage_2(1,4);
initial_data(4,1) = lvx*cos(initial_data(3,1))+lvy*sin(initial_data(3,1)); %y(4)表示降落伞速度；
initial_data(5,1) = initial_data(4,1); %y(5)表示空降车速度；
initial_data(6,1) = 0.000;                                                   %伞绳拉出长度。
% initial_data(7,1) = 0.1;                                                   %伞绳拉出速度
% initial_data(8,1) = 0.1;                                                   %回收物与伞袋的距离。
% initial_data(9,1) = data_stage_2(1,5);                                     %空降车俯仰角度；
% initial_data(10,1) = data_stage_2(1,6);                                    %空降车俯仰角速度；
Lsh = jiansusan_length;  %%减速伞伞绳长；
Lxt = jiansusan_length + jiansusan_diam/2 + jiasu_width; %%减速伞系统全长；
Fsh = force_pullout;    %%拉出阻力；
b = jiasu_width;     %%伞边宽度；
b1 = hull_cmy-ini_loady;
a1 = hull_cmx;
Msh1 = jiasurope_mass/jiansusan_length;  %%伞绳的质量密度；
Msy1 = jiansu_density_side;    %%伞边的质量密度；
CAw = resist_hull;     %%空投物阻力系数；
%%%%%%参数定义；
simtime = [0:step:sim_time]';
fun = @ode45_stage3;
[time_3,data_3] = ode45(fun,simtime,initial_data);
data_33 = [];
data_33 = data_3;
Lx = data_3(:,6);
m = length(time_3);
for i = 1:m
    if Lx(i) >= Lxt
      nn = i;
      break;
    end
end
 time_jslz = time_3(nn);
 vR = data_3(:,5) - data_3(:,4);
 ang_vehicle = zeros(nn,1);
 angw_vehicle = zeros(nn,1);
 ang_vehicle(1) = data_stage_2(1,5);                                     %空降车俯仰角度；
 angw_vehicle(1) = data_stage_2(1,6);                                    %空降车俯仰角速度；
 gama = atan(cmpull_y/cmpull_L);
 Lat = cmpull_H/sin(gama);
 Vmx = data_3(:,5).*cos(data_3(:,3));
 Dmi = zeros(m,1);
 FL = zeros(m,1);
 Msy = jiansusan_mass - jiasurope_mass;    %%伞衣总质量；
 acc_angle = zeros(m-1,1);
%  aa = -8 * mass_chuteclothes/diam_chutechothes^2;
%  bb = 8 * mass_chuteclothes*Lxt/diam_chutechothes^2;
 Do = sqrt(jiansusan_aera/pi)*2;      %%伞衣名义直径；
 B = Msy1/Msh1;
 for i = 1:m
      if data_3(i,6) <=Lsh
         Dmi(i) = Msh1;
     elseif data_3(i,6) <=Lsh +b
         Dmi(i) = Msy1;
     elseif data_3(i,6) <= Lxt
%         Dmi(i) = 4*Msy/Do - 8*Msy/Do*(data3(i,6)-Lsh-b)/Do;
          Dmi(i) = 8*(Msy-Msy1*b)*(Lxt - data_3(i,6))/Do^2; 
      end
     FL(i) = Dmi(i) * vR(i)^2 + Fsh;
 end
 for i = 2:m
     Sx(i-1) = (hull_H*cos(ang_vehicle(i-1)) +hull_L*sin(ang_vehicle(i-1) ))*hull_W;
     Shb(i-1) = (b1*cos(ang_vehicle(i-1))+a1*sin(ang_vehicle(i-1)))*hull_W;      %重心以下的迎风面积
     Sht(i-1) =  Sx(i-1) - Shb(i-1);
     Qwb(i-1) = 0.5*density_air*CAw*Shb(i-1)^2*Vmx(i-1)^2/(4*hull_W);              %空降车质心以下气动阻力矩
     Qwt(i-1) = 0.5*density_air*CAw*Sht(i-1)^2*Vmx(i-1)^2/(4*hull_W);              %空降车质心以上气动阻力矩
     QFL(i-1) = FL(i-1)*Lat*sin(ang_vehicle(i-1)+data_3(i-1,3)-pi/2);
   if ang_vehicle(i-1) + data_3(i-1,3) == pi/2
       Tw(i-1) = Qwt(i-1) - Qwb(i-1);
   else 
       Tw(i-1) = Qwt(i-1) - Qwb(i-1) - QFL(i-1);
   end
   acc_angle(i-1) = Tw(i-1)/hull_Izz; 
   angw_vehicle(i) = angw_vehicle(i-1) + acc_angle(i-1) *(time_3(i)-time_3(i-1));
   ang_vehicle(i) = ang_vehicle(i-1) + angw_vehicle(i-1)*(time_3(i)-time_3(i-1));
 end
data_jslz = zeros(9,1);
data_jslz(1,1) = data_3(nn,1);     %y(1)表示空投车辆X向位移Xd；
data_jslz(2,1) = data_3(nn,2);     %y(2)表示空投车辆Y向位移Yd；
data_jslz(3,1) = data_3(nn,3);     %y(3)表示姿态角；
data_jslz(4,1) = data_3(nn,4);     %y(4)表示降落伞速度Vys； 
data_jslz(5,1) = data_3(nn,5);     %y(5)表示空投车辆速度Vw；
data_jslz(6,1) = data_3(nn,6);     %y(6)表示拉出伞绳长度
data_jslz(7,1) = ang_vehicle(nn,1);   %y(7)表示空投车辆俯仰角度；
data_jslz(8,1) = angw_vehicle(nn,1);  %y(8)表示空投车辆俯仰角速度；
data_jslz(9,1) = FL(nn);         %y(9)不考虑弹性的伞绳拉直力。
data_out = zeros(nn,9);
data_out(:,1:6) = data_3(1:nn,1:6);
data_out(:,7) = ang_vehicle(1:nn,1);
data_out(:,8) = angw_vehicle(1:nn,1);
data_out(:,9) = FL(1:nn,1);
data_out_2 = zeros(nn,2);
data_out_2(:,1) =data_33(1:nn,5);
data_out_2(:,2) =data_33(1:nn,3);
% [time3,data3,FLT] = strighten (time,initial_data);
% bit = initial_data(3,1);
% vwx = - initial_data(2,1);
% vwy = initial_data(4,1);
% initial_data(4,1) = vwx*cos(bit) - vwy*sin(bit);               %y(4)表示Vys；
% initial_data(5,1) = vwx*cos(bit) - vwy*sin(bit);               %y(5)表示Vw；
% initial_data(6,1) = 0;                                         %y(6)表示伞系统微元质量拉出长度；
end

function [time_jscq,data_jscq,time_4,data_out,data_out_2] = jiansu_chongqi(data_stage_3,js_chuqi_para)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 hull_cmx= js_chuqi_para(1);
 resist_hull= js_chuqi_para(2);
 hull_cmy= js_chuqi_para(3);
 ini_loady= js_chuqi_para(4);
  hull_H = js_chuqi_para(5);
 hull_L = js_chuqi_para(6);
 hull_W = js_chuqi_para(7);
 hull_Izz = js_chuqi_para(8);
 cmpull_y = js_chuqi_para(9);
 cmpull_L= js_chuqi_para(10);
 cmpull_H = js_chuqi_para(11);
 jiansusan_num = js_chuqi_para(12);
 jiansusan_mass = js_chuqi_para(13);
 jiansusan_aera = js_chuqi_para(14);
 drop_hight = js_chuqi_para(15);
 jiansusan_c= js_chuqi_para(16);
 jiansu_kf = js_chuqi_para(17);
 density_air= js_chuqi_para(18);
 grav = js_chuqi_para(19);
 k_ks= js_chuqi_para(20);
 sim_time= js_chuqi_para(21);
 step= js_chuqi_para(22);

D = 6.5*1E-3;    %大气温度梯度系数；
h = drop_hight - data_stage_3(2);
C=2000;
simtime = [0:step:sim_time]';
initial_data = zeros(4,1);
initial_data(1,1) = data_stage_3(1,1);  %X向位移
initial_data(2,1) = data_stage_3(2,1);  %Y向位移；
initial_data(3,1) = data_stage_3(3,1);  %轨迹角
initial_data(4,1) = data_stage_3(5,1);  %空降车速度；
% initial_data(5,1) = data_stage_3(7,1);  %车辆俯仰角；
% initial_data(6,1) = data_stage_3(8,1);  %车辆俯仰角速度；
fun = @ode45_stage4;
[time_4,data_4] = ode45(fun,simtime,initial_data);
data_44 = [];
data_44 = data_4;
yita = 1.05 - 0.05 * jiansusan_num ; %伞群效率；
T0= 273;      %气温273k；
rho = density_air*(1-D*h/T0)^4.26; %%空气密度(随高度变化)；
Mjs = jiansusan_mass; %减速伞质量；
g = grav;             %%重力加速度；
m =length(time_4);
t = time_4;
CAjs = jiansusan_c * yita * jiansusan_aera * jiansusan_num;  %%减速伞阻力系数；
Msf =  rho * jiansu_kf * CAjs^(3/2);        %减速伞附加质量；
Ms_all = Mjs + Msf;    %空投系统总质量；
VL1 = data_stage_3(5,1);
jiansu_d0 = sqrt(jiansusan_aera/pi)*2;
% t1 =jiansu_d0 *jiansu_ks/VL1^jiansu_ksn;
CAjst = jiansusan_c * jiansusan_aera ;           %%主伞阻力系数；
t1 = k_ks*sqrt(CAjst)/VL1;
for i = 1:m
    if t(i) >= t1
        n = i;
        break;
    end
end
dv = zeros(n,1);
Sm = zeros(n,1);
CAms = zeros(n,1);
Fk = data_stage_3(9,1);
acc_angle = zeros(n-1,1);
% time_stage4 = t(n);
time_jscq = t1;
for i=2:n
    Sm(i) = jiansusan_aera * t(i)/t1;
    Dsm = jiansusan_aera /t1;
    CAms(i) = jiansusan_c * yita * Sm(i) * jiansusan_num;   %%减速伞阻力系数；
    dv(i) = (data_4(i-1,4)-data_4(i,4))/(t(i,1)-t(i-1,1));
    Fk(i) = Ms_all*dv(i) + 0.5*rho*data_4(i,4)^2*CAms(i)+data_4(i,4)*Dsm + Mjs*g*sin(data_4(i,3));
end
 ang_vehicle = zeros(n,1);
 angw_vehicle = zeros(n,1);
 ang_vehicle(1) = data_stage_3(7);                                     %空降车俯仰角度；
 angw_vehicle(1) = data_stage_3(8);                                    %空降车俯仰角速度；
 gama = atan(cmpull_y/cmpull_L);
 Lat = cmpull_H/sin(gama);
 Vmx = data_4(:,4).*cos(data_4(:,3));
 CAw = resist_hull;     %%空投物阻力系数；
 b1 = hull_cmy-ini_loady;
 a1 = hull_cmx;
 for i = 2:n
     Sx(i-1) = (hull_H*cos(ang_vehicle(i-1)) +hull_L*sin(ang_vehicle(i-1) ))*hull_W;
     Shb(i-1) = (b1*cos(ang_vehicle(i-1))+a1*sin(ang_vehicle(i-1)))*hull_W;      %重心以下的迎风面积
     Sht(i-1) =  Sx(i-1) - Shb(i-1);
     Qwb(i-1) = 0.5*density_air*CAw*Shb(i-1)^2*Vmx(i-1)^2/(4*hull_W);              %空降车质心以下气动阻力矩
     Qwt(i-1) = 0.5*density_air*CAw*Sht(i-1)^2*Vmx(i-1)^2/(4*hull_W);              %空降车质心以上气动阻力矩
     QFL(i-1) = Fk(i-1)*Lat*(sin(ang_vehicle(i-1)+data_4(i-1,3)-gama-pi/2));
   if ang_vehicle(i-1) + data_4(i-1,3) == pi/2
       Tw(i-1) = Qwt(i-1) - Qwb(i-1);
   else
       Tw(i-1) = Qwt(i-1) - Qwb(i-1)- QFL(i-1);
   end
   acc_angle(i-1) = 1*Tw(i-1)/hull_Izz; 
   angw_vehicle(i) = angw_vehicle(i-1) + acc_angle(i-1) *(time_4(i)-time_4(i-1));
   ang_vehicle(i) = ang_vehicle(i-1) + angw_vehicle(i-1)*(time_4(i)-time_4(i-1));
 end
data_jscq = zeros(7,1);
data_jscq(1,1) = data_4(n,1);        %y(1)表示空投车辆X向位移Xd；
data_jscq(2,1) = data_4(n,2);        %y(2)表示空投车辆Y向位移Yd；
data_jscq(3,1) = data_4(n,3);        %y(3)表示姿态角；
data_jscq(4,1) = data_4(n,4);        %y(4)表示空投车辆速度Vw； 
data_jscq(5,1) = ang_vehicle(n,1);   %y(5)表示空投车辆俯仰角度；
data_jscq(6,1) = angw_vehicle(n,1);  %y(6)表示空投车辆俯仰角速度；
data_jscq(7,1) = Fk(n);              %y(7)不考虑弹性的伞绳拉直力。
Fk = Fk';
data_out = zeros(n,7);
data_out(1:n,1:4) = data_4(1:n,:);        %y(1)表示空投车辆X向位移Xd；
data_out(1:n,5) = ang_vehicle(1:n,1);   %y(5)表示空投车辆俯仰角度；
data_out(1:n,6) = angw_vehicle(1:n,1);  %y(6)表示空投车辆俯仰角速度；
data_out(1:n,7) = Fk(1:n,1);  %y(7)不考虑弹性的伞绳拉直力。
data_out_2 = zeros(n,2);
data_out_2(:,1) =data_44(1:n,4);
data_out_2(:,2) =data_44(1:n,3);
 end

% --- Executes on button press in pushbutton15.
function [time_mslz,data_mslz,time_5,data_out,data_out_2] = zhusan_straight(data_stage_2,ms_strait_para,data_jslz)
global drop_envi Vehicle pullchute decechute mainchute sim 
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hull_cmx = ms_strait_para(1);
hull_cmy  = ms_strait_para(2);
ini_loady  = ms_strait_para(3);
track_fdy = ms_strait_para(4);
cmpull_y  = ms_strait_para(5);
cmpull_L  = ms_strait_para(6);
cmpull_H = ms_strait_para(7);
hull_Izz = ms_strait_para(8);
hull_H = ms_strait_para(9);
hull_L = ms_strait_para(10);
hull_W  = ms_strait_para(11);
resist_hull = ms_strait_para(12);
jiansusan_mass = ms_strait_para(13);
diam_chutechothes = ms_strait_para(14);
mass_chuteclothes = ms_strait_para(15);
pullchute_L = ms_strait_para(16);
velo_plane = ms_strait_para(17);
density_air = ms_strait_para(18);
force_pullout = ms_strait_para(19);
length_rope = ms_strait_para(21);
jiansusan_length = ms_strait_para(22);
jiansusan_aera = ms_strait_para(23);
jiansusan_num = ms_strait_para(24);
length_chuteall = ms_strait_para(25);
jiasu_width = ms_strait_para(26);
jiasurope_mass  = ms_strait_para(27);
jiansu_density_side = ms_strait_para(28);
sim_time = ms_strait_para(29); 
step = ms_strait_para(30);

Djs = sqrt(jiansusan_aera/pi)*2;
%参数转换
% nume = pull_y-ini_loady;
nume = data_stage_2(1,2) - (track_fdy - hull_cmy) ;

deno = cmpull_H + pullchute_L ;
initial_data = zeros(6,1);
initial_data(1,1) = data_stage_2(1,1);                           %y(1)示空降车X向位移；
initial_data(2,1) = data_stage_2(1,2);                      %y(2)表示空降车Y向位移；
initial_data(3,1) = atan(nume/deno);                        %y(3)表示空降车轨迹角；
lvx = data_stage_2(1,4)*sind(data_stage_2(1,3));
lvy = data_stage_2(1,4)*cosd(data_stage_2(1,3));
initial_data(4,1) = lvx*cos(initial_data(3,1))+lvy*sin(initial_data(3,1)); %y(4)表示降落伞速度；
initial_data(5,1) = initial_data(4,1);                                      %y(5)表示空降车速度；
initial_data(6,1) = data_jslz(6,1);                                                   %伞绳拉出长度。
% initial_data(7,1) = 0.1;                                                   %伞绳拉出速度
% initial_data(8,1) = 0.1;                                                   %回收物与伞袋的距离。
% initial_data(9,1) = data_stage_2(1,5);                                     %空降车俯仰角度；
% initial_data(10,1) = data_stage_2(1,6);                                    %空降车俯仰角速度；
Mjs = jiansusan_mass;            %减速伞总质量；
Mjsh = jiasurope_mass;           %减速伞伞绳质量；
Msy = mass_chuteclothes;     %%伞衣总质量
Lsh = length_rope;  %%减速伞伞绳长；
Ljs = jiansusan_length + Djs/2 + jiasu_width; %%减速伞系统全长；
Lxt = length_chuteall + Ljs; %%伞系统全长；
Fsh = force_pullout;    %%拉出阻力；
Ljsh = jiansusan_length;         %减速伞伞绳长度；
Djs = sqrt(jiansusan_aera/pi)*2; %减速伞有效直径；
Do = diam_chutechothes;      %%伞衣名义直径
Mjsy1 =  jiansu_density_side    ;%减速伞伞边质量密度；
Ljsb = jiasu_width ;             %减速伞伞边宽度；
Mjsy = Mjs - Mjsh - Ljsb*Mjsy1;     %减速伞伞衣质量；
n = jiansusan_num;      %%伞绳数量；
b = jiasu_width;     %%伞边宽度；
b1 = hull_cmy-ini_loady;
a1 = hull_cmx;
Msh1 = jiasurope_mass/jiansusan_length;  %%伞绳的质量密度；
Msy1 = jiansu_density_side;    %%伞边的质量密度；
CAw = resist_hull;     %%空投物阻力系数；
%%%%%%参数定义；
simtime = [0:step:sim_time]';
fun = @ode45_stage5;
[time_5,data_5] = ode45(fun,simtime,initial_data);
data_55 = [];
data_55 = data_5;
Lx = data_5(:,6);
m = length(time_5);
for i = 1:m
    if Lx(i) >= Lxt
      n = i;
      break;
    end
end
 time_mslz = time_5(n);
 vR = data_5(1:n,5) - data_5(1:n,4);
 B = Msy1/Msh1;
 ang_vehicle = zeros(n,1);
 angw_vehicle = zeros(n,1);
 ang_vehicle(1) = data_stage_2(1,5) ;                                    %空降车俯仰角度；
 angw_vehicle(1) = data_stage_2(1,6);                                    %空降车俯仰角速度；
 gama = atan(cmpull_y/cmpull_L);
 Lat = cmpull_H/sin(gama);
 Vmx = data_5(:,5).*cos(data_5(:,3));
%  Dmi = zeros(n,1);
 FL = zeros(n,1);
%  aa = -8 * mass_chuteclothes/diam_chutechothes^2;
%  bb = 8 * mass_chuteclothes*Lxt/diam_chutechothes^2;
 Lu = data_5(1:n,6);
 for i = 1:n
     if  Lu(i) <=Ljsh
         Dmi = Mjsh/Ljsh;
     elseif  Lu(i) <=Ljsh + Ljsb
         Dmi = Mjsy1;
     elseif  Lu(i) <=Ljs
         Dmi = 8*Mjsy * (Ljs-Lu(i))/Djs^2;
     elseif  Lu(i) <=Ljs + Lsh
         Dmi = Msh1;
     elseif  Lu(i) <= Ljs + Lsh + b
         Dmi = Msy1;
     elseif  Lu(i) <= Lxt
         Dmi =8*(Msy-Msy1*b)*(Lxt-Lu(i))/Do^2;
     end
     FL(i) = Dmi * vR(i)^2 + Fsh;
 end
 for i = 2:n
     Sx(i-1) = (hull_H*cos(ang_vehicle(i-1)) +hull_L*sin(ang_vehicle(i-1) ))*hull_W;
     Shb(i-1) = (b1*cos(ang_vehicle(i-1))+a1*sin(ang_vehicle(i-1)))*hull_W;      %重心以下的迎风面积
     Sht(i-1) =  Sx(i-1) - Shb(i-1);
     Qwb(i-1) = 0.5*density_air*CAw*Shb(i-1)^2*Vmx(i-1)^2/(4*hull_W);              %空降车质心以下气动阻力矩
     Qwt(i-1) = 0.5*density_air*CAw*Sht(i-1)^2*Vmx(i-1)^2/(4*hull_W);              %空降车质心以上气动阻力矩
     QFL(i-1) = FL(i-1)*Lat*sin(ang_vehicle(i-1)+data_5(i-1,3)-pi/2);
   if ang_vehicle(i-1) + data_5(i-1,3) -gama == pi/2
       Tw(i-1) = Qwt(i-1) - Qwb(i-1);
   else 
       Tw(i-1) = Qwt(i-1) - Qwb(i-1) - QFL(i-1);
   end
   acc_angle(i-1) = Tw(i-1)/hull_Izz;
   angw_vehicle(i) = angw_vehicle(i-1) + acc_angle(i-1) *(time_5(i)-time_5(i-1));
   ang_vehicle(i) = ang_vehicle(i-1) + angw_vehicle(i-1)*(time_5(i)-time_5(i-1));
 end
data_mslz = zeros(9,1);
data_mslz(1,1) = data_5(n,1);        %y(1)表示空投车辆X向位移Xd；
data_mslz(2,1) = data_5(n,2);        %y(2)表示空投车辆Y向位移Yd；
data_mslz(3,1) = data_5(n,3);        %y(3)表示姿态角；
data_mslz(4,1) = data_5(n,4);        %y(4)表示降落伞速度Vys； 
data_mslz(5,1) = data_5(n,5);        %y(5)表示空投车辆速度Vw；
data_mslz(6,1) = data_5(n,6);        %y(6)表示拉出伞绳长度
data_mslz(7,1) = ang_vehicle(n,1);   %y(7)表示空投车辆俯仰角度；
data_mslz(8,1) = angw_vehicle(n,1);  %y(8)表示空投车辆俯仰角速度；
data_mslz(9,1) = FL(n);              %y(9)不考虑弹性的伞绳拉直力
data_out = zeros(n,9);
data_out(1:n,1:6) = data_5(1:n,:);    %y(1)表示空投车辆X向位移Xd；
data_out(1:n,7) = ang_vehicle(1:n,1);   %y(5)表示空投车辆俯仰角度；
data_out(1:n,8) = angw_vehicle(1:n,1);  %y(6)表示空投车辆俯仰角速度；
data_out(1:n,9) = FL(n,1);            %y(6)表示空投车辆俯仰角速度；
data_out_2 = zeros(n,3);
data_out_2(:,1) =data_55(1:n,5);
data_out_2(:,2) =data_55(1:n,3);
data_out_2(:,3) =ang_vehicle(1:n,1);
global data_stage_5
data_stage_5 = data_mslz ;


 end
  
function dy = ode45_stage1(t,y)
global drop_envi Vehicle pullchute
%%参数赋初值：
grav = drop_envi.Grav;
plane_angle = drop_envi.Cargo_angle;
drop_hight = drop_envi.drophight;
velo_plane = drop_envi.AirplaneVelo;
R = drop_envi.Earth_R;
density_air = drop_envi.AirDensity;

drop_mass = Vehicle.dropmass;
mu = Vehicle.mu;

resist_chute = pullchute.resist;
chute_area = pullchute.area;

%%参数赋初值结束。

chute_k = resist_chute*chute_area;       %降落伞阻力系数
velocity = velo_plane/3.6;       %空投时运输机速度转换（km/h）；
alfa0 = plane_angle*pi/180;       %运输机货舱地板与水平面夹角转换；
mass = drop_mass*1000;       %空投车辆质量转换（t）
grav1 = grav * (1-drop_hight / R);
G = drop_mass * grav1;
D = 6.5*1E-3;    %大气温度梯度系数；
T0= 273; %气温273k；
h = drop_hight;
rho = density_air*(1-D*h/T0)^4.26; %%空气密度(随高度变化)；
%%%%%常数赋初值
dy = zeros(4,1);
%%%y(1)为x向位移；y(2)为x向速度；y(3)为y向位移；y(4)为y向速度；
dy(1) = y(2);
dy(3) = y(4);
%%%%定义微分方程组
Q = 0.5 * chute_k * rho * (velocity - y(2))^2;
N = G*cos(alfa0) - Q * sin(alfa0);
dy(2) = (-Q + mu * N * cos(alfa0) - N * sin(alfa0))/mass;
dy(4) =( G - N * cos(alfa0) - mu * N * sin(alfa0))/mass;
end

function dy = ode45_stage2(t,y)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Vehicle pullchute drop_envi
%%%%%%%%%%%%%%%%%%%%%   伞降相关参数设置  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
drop_mass = Vehicle.dropmass;
grav = drop_envi.Grav;
velo_plane = drop_envi.AirplaneVelo;
h = drop_envi.drophight;
R = drop_envi.Earth_R;
density_air = drop_envi.AirDensity;
Jzz = Vehicle.Izz;  %空投车沿车体横向轴的转动惯量
hm = Vehicle.hull_cmy -Vehicle.track_fdy ;        %车辆重心至下平面的距离(mm)
hp = Vehicle.pull_y - Vehicle.track_fdy  ;        %牵引伞牵引点至车辆下平面的距离（mm）
lp = - Vehicle.pull_x + Vehicle.hull_cmx;           %牵引伞牵引点至车辆重心的水平距离（mm）
mu = Vehicle.mu;
%%%%%%%%%%%%%%%%%%%%%%%   常数参数计算  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
chute_k = pullchute.resist * pullchute.area;       %牵引伞阻力系数                     %
% plane_angle=plane_angle*pi/180;       %运输机货舱地板与水平面夹角转换；   %
mass = drop_mass*1000;               %空投车辆质量单位转换（t->kg）      %
grav1 = grav * (1-h / R);   %海拔高度重力加速度；
L0 = sqrt((hm-hp)^2 + lp^2);
gama = atan((hm-hp)/lp);
G = grav1 * mass;
velos0 = velo_plane/3.6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%定义微分方程%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dy = zeros(6,1);
dy(1) = y(2);  %X向位移
dy(3) = y(4);  %Y向位移
dy(5) = y(6);  %俯仰角；
%%%%%%%%%%%%%%%%%%定义微分方程表达式%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
D = 6.5*1E-3;    %大气温度梯度系数；
T0= 273; %气温273k；

rho = density_air*(1-D*h/T0)^4.26; %%空气密度(随高度变化)；
Q = 0.5 * chute_k * rho * (velos0 - y(2))^2;%改动处
N = G*cos(y(5)) - Q * sin(y(5));
Lop = (abs(y(1))-hm * tan(y(5)))/cos(y(5));
dy(2) = (-Q - N * sin(y(5)) + mu * N * cos(y(5)) )/mass;  
y(3) = y(1) * tan(y(5)) - hm * (1/cos(y(5))-1);
dy(4) = ( G - N * cos(y(5))- mu * N * sin(y(5)) )/mass;   
dy(6) = (mu * N * hm + N * Lop - Q * L0 * sin(y(5)+gama))/Jzz;
end

function dy = ode45_stage3(t,y)
%%%%%参数赋初值
global Vehicle pullchute drop_envi mainchute decechute
density_air = drop_envi.AirDensity;
grav = drop_envi.Grav;
resist_chute_cover = mainchute.ChuteCoverResist;
resist_hull = Vehicle.resist_hull;
resist_pullout = mainchute.PulloutResist;
drop_mass = Vehicle.dropmass;
mass_parachuter = pullchute.mass;
jiansu_density_side = decechute.ChuteSideMass/decechute.ChuteWidth;
jiasu_width = decechute.ChuteWidth;
jiansusan_aera = decechute.aera;
force_pullout = mainchute.PulloutForce;
jiasurope_mass = decechute.RopeMass;
jiansusan_length = decechute.RopeLength;
hull_L = Vehicle.hull_L;
hull_W = Vehicle.hull_W;
resist_chute = pullchute.resist;
chute_area = pullchute.area;
jiansusan_mass = decechute.mass;
%%%%定义参数%%%%
dy = zeros(6,1);
rho = density_air; %%空气密度；
g = grav;     %%重力加速度；
CAys = resist_chute * chute_area;  %%牵引伞阻力系数；
CAd = resist_chute_cover;   %%伞衣套阻力系数；
CAw = resist_hull;     %%空投物阻力系数；
CAe = resist_pullout;  %%已拉出物体阻力系数
Mw = drop_mass*1000;    %%空投车辆质量；
Mys = mass_parachuter;    %%牵引伞及伞衣套质量；
Msh1 = jiasurope_mass/jiansusan_length;  %%伞绳的质量密度；
Msy1 = jiansu_density_side;    %%伞边的质量密度；
b = jiasu_width;     %%伞边宽度；
Msh = jiasurope_mass;     %%伞绳总质量；
Msy = jiansusan_mass - jiasurope_mass;    %%伞衣总质量；
Lsh = jiansusan_length;    %%伞绳全长；
Do = sqrt(jiansusan_aera/pi)*2;      %%伞衣名义直径；
Lxt = jiansusan_length + jiasu_width + Do/2; %%伞系统全长；
Fsh = force_pullout;    %%拉出阻力；
Qys = 0.5 * rho * CAys * y(4)^2;  %%牵引伞气动阻力；
Qd = 0.5 * rho * CAd * y(4)^2;    %%伞衣套气动阻力；
Qw = 0.5 * rho * CAw * hull_L*hull_W * y(5)^2;  %%空投车辆q向气动阻力；
% Qwy = 0.5 * rho * CAw * Sy * y(4)^2;    %%空投车辆Y向气动阻力；
Qe = 0.5 * rho * CAe * y(4)^2;            %%已拉出伞系统气动阻力；
if y(6) <= Lsh
    Me = Msh1*y(6);
    Dmi = Msh1;    
elseif y(6) <= Lsh +b
    Me = Msh1*Lsh + Msy1*y(6);
    Dmi = Msy1;       
elseif y(6) <= Lxt
    Me = Msh1*Lsh + Msy1*b +(Msy-Msy1*b) * (1-4*((Lxt-y(6))/Do)^2);
    Dmi = 8*(Msy-Msy1*b)*(Lxt - y(6))/Do^2; 
else 
     Me = 0;
     Dmi= 0.01;
end
    Mv = Msh + Msy -Me;
u = y(5)-y(4);
% vR = u;
FL = Dmi * u^2 + Fsh; 
% c = sqrt(E/Msh1);
% B = Msy1/Msh1;
% k = B * (vR/c - Fsh/(number_rope*E));
% FLT = number_rope*E*(vR/c - (sqrt(1+4*k)-1)/(2*B));
%  Vmx = y(5)*cos(y(3));
%  Sx = (hull_H*cos(y(9))+hull_L*sin(y(9)))*hull_W;
%  Sq = (hull_H*abs(sin(y(7)-pi/2+y(3)))+hull_L*cos(y(7)-pi/2+y(3)))*hull_W;
% FL = number_rope*E*(vR/c - (sqrt(1+4*k)-1)/(2*B));
dy(1) = 1*y(5)*cos(y(3));                                             %y(1)表示空投车辆X向位移Xd；
dy(2) = 1*y(5)*sin(y(3));                                             %y(2)表示空投车辆Y向位移Yd；
dy(3) = 1*g*cos(y(3))/y(5);                                           %y(3)表示姿态角；
dy(4) = 1*g*sin(y(3)) - (Qys+Qd-Fsh)/(Mys+Mv);                        %y(4)表示降落伞速度Vys；
dy(5) = 1*g*sin(y(3)) - (Qw+Qe+FL)/(Mw+Me);                           %y(5)表示空投车辆速度Vw；
dy(6) = ((FL-Fsh)/Dmi)^0.5;                                           %y(6)伞绳拉出长度；      
 end

function dy = ode45_stage4(t,y)
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

function dy = ode45_stage5(t,y)
%%%接收全局参数；
global Vehicle drop_envi mainchute decechute data_stage_3 pullchute
density_air = drop_envi.AirDensity;
grav = drop_envi.Grav;
resist_hull = Vehicle.resist_hull;
drop_mass = Vehicle.dropmass;
jiansusan_aera = decechute.aera;
hull_L = Vehicle.hull_L;
hull_W = Vehicle.hull_W; 
jiansusan_mass = decechute.mass;
jiansusan_num = mainchute.ChuteNum;
jiansusan_c = decechute.resist;
resist_chute = pullchute.resist;
resist_chute_cover  = mainchute.ChuteCoverResist;
resist_pullout = mainchute.PulloutResist;
mass_parachuter = pullchute.mass;
density_rope = mainchute.RopeMass/mainchute.RopeLength;
density_chuteside = mainchute.SideMass / mainchute.SideWidth;
chuteside_width = mainchute.SideWidth;
mass_rope = mainchute.RopeMass;
mass_chuteclothes = mainchute.ChuteMass;
length_rope = mainchute.RopeLength;
aera_mainchute = mainchute.ChuteAera;
diam_chutechothes =2*sqrt(aera_mainchute/pi);
length_chuteall =  chuteside_width + length_rope + diam_chutechothes/2;
force_pullout = mainchute.PulloutForce;
chute_area = decechute.aera;
jiasurope_mass = decechute.RopeMass ; 
jiansusan_length = decechute.RopeLength;
jiansu_density_side = decechute.ChuteSideMass / decechute.ChuteWidth;
jiasu_width = decechute.ChuteWidth;

%%%%定义参数%%%%
dy = zeros(6,1);
rho = density_air; %%空气密度；
g = grav;     %%重力加速度；
yita = 1.05 - 0.05 * jiansusan_num; %伞群效率；
CAys = resist_chute*chute_area;  %%牵引伞阻力系数；
CAjs = jiansusan_c * jiansusan_aera * yita * jiansusan_num;
CAd = resist_chute_cover;        %%伞衣套阻力系数；
CAw = resist_hull;               %%空投物阻力系数；
CAe = resist_pullout;            %%已拉出物体阻力系数
Mw = drop_mass*1000;             %%空投车辆质量；
Mjs = jiansusan_mass;            %减速伞总质量；
Mjsh = jiasurope_mass;           %减速伞伞绳质量；
Ljsh = jiansusan_length;         %减速伞伞绳长度；
Djs = sqrt(jiansusan_aera/pi)*2; %减速伞有效直径；
Mjsy1 =  jiansu_density_side    ;%减速伞伞边质量密度；
Ljsb = jiasu_width ;             %减速伞伞边宽度；
Mjsy = Mjs - Mjsh - Ljsb*Mjsy1;     %减速伞伞衣质量； 
Mys = mass_parachuter;       %%牵引伞及伞衣套质量；
Msh1 = density_rope;         %%伞绳的质量密度；
Msy1 = density_chuteside;    %%伞边的质量密度；
b = chuteside_width;         %%伞边宽度；
Msh = mass_rope;             %%伞绳总质量；
Msy = mass_chuteclothes;     %%伞衣总质量；
Lsh = length_rope;           %%伞绳全长；
Ljs = Ljsh + Ljsb + Djs/2;   %%减速伞全长
Lxt = length_chuteall + Ljs; %%伞系统全长；
Do = diam_chutechothes;      %%伞衣名义直径；
Fsh = force_pullout;         %%拉出阻力；
Qys = 0.5 * rho * CAys * y(4)^2;  %%牵引伞气动阻力；
Qd = 0.5 * rho * CAd * y(4)^2;    %%伞衣套气动阻力；
Qjs = 0.5 * rho * CAjs * y(5)^2;   %%减速伞气动阻力；
Qw = 0.5 * rho * CAw * hull_L*hull_W * y(5)^2;  %%空投车辆q向气动阻力；
% Qwy = 0.5 * rho * CAw * Sy * y(4)^2;    %%空投车辆Y向气动阻力；
Qe = 0.5 * rho * CAe * y(4)^2;            %%已拉出伞系统气动阻力；
m = length(t);
Me = zeros(m,1);
Dmi = zeros(m,1);
if y(6) <=Ljsh
    Me = Mjsh/Ljsh*y(6);
    Dmi = Mjsh/Ljsh;
elseif y(6) <=Ljsh + Ljsb
    Me = Mjsh + Mjsy1 *(y(6)-Ljsh);
    Dmi = Mjsy1;
elseif y(6) <=Ljs
    Me = Mjsh + Ljsb*Mjsy1 + Mjsy * (1-4*((Ljs-y(6))/Djs)^2);
    Dmi = 8*Mjsy * (Ljs-y(6))/Djs^2;
elseif y(6) <=Ljs + Lsh
    Me = Mjs + Msh1*(y(6)-Ljs);
    Dmi = Msh1;    
elseif y(6) <= Ljs + Lsh +b
    Me = Mjs + Msh1*Lsh + Msy1*(y(6)-Ljs - Lsh);
    Dmi = Msy1;
elseif y(6) <= Lxt
    Me = Mjs + Msh1*Lsh + Msy1*b +(Msy-Msy1*b) * (1-4*((Lxt-y(6))/Do)^2);
    Dmi =8*(Msy-Msy1*b)*(Lxt-y(6))/Do^2;
else
    Me = 0;
    Dmi = 0.001;
end
    Mv = Mjs + Msh + Msy -Me;
u = y(5)-y(4);
FL = Dmi * u^2 + Fsh; 
dy(1) = 1*y(5)*cos(y(3));                                          %y(1)表示空投车辆X向位移Xd；
dy(2) = 1*y(5)*sin(y(3));                                          %y(2)表示空投车辆Y向位移Yd；
dy(3) = 1*g*cos(y(3))/y(5);                                        %y(3)表示姿态角；
dy(4) = 1*g*sin(y(3)) - (Qys + Qd - Fsh)/(Mys + Mv);                %y(4)表示降落伞速度Vys；
if y(6) < Ljs
    dy(5) = 1*g*sin(y(3)) - (Qw + Qe + FL)/(Mw + Me);                        %y(5)表示空投车辆速度Vw；
else
    dy(5) = 1*g*sin(y(3)) - (Qw + Qe+ Qjs + FL)/(Mw+Me);                  %y(5)表示空投车辆速度Vw；
end
dy(6) = ((FL-Fsh)/Dmi)^0.5;                                             %y(6)伞绳拉出长度；    
end

function dy = ode45_stage6(t,y)
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

function [time_wj,data_wj,Xx,Yy,Zz,Vx,Vy,Vz] = sim_wj(data_mscq,wj_para,data_66)
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
function [dYdt,ax_body_1] = dynsys(t,Y,params)
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
