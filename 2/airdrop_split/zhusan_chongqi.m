function [time_mscq,data_mscq,time,data,data_2] = zhusan_chongqi(data_stage_4,ms_chuqi_para)
% ZHUSAN_CHONGQI 主伞充气阶段。
%
% 物理含义：主伞完成拉直后，主伞按“两阶段充气”模型增大有效面积，车辆受到
% 主伞阻力、减速伞阻力、车体阻力、附加质量变化和重力共同作用。
%
% 输入：
%   data_stage_4: 主伞拉直阶段终止状态。
%   ms_chuqi_para: 主伞充气参数向量。
%
% 输出：
%   time_mscq, data_mscq: 主伞充气稳定结束点。
%   time, data: ode45 全时程。
%   data_2: 后处理后的阶段曲线，含速度/轨迹角形式。
%
% 注意：本函数会计算全局变量 t21，供 ode45_stage6 计算二次充气面积增长使用。
% 稳定结束点不是 ode event，而是对开伞力 Fk 的差分 abs(diff(Fk)) <= 1 后处理获得。

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

