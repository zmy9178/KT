function [time_jscq,data_jscq,time_4,data_out,data_out_2] = jiansu_chongqi(data_stage_3,js_chuqi_para)
% JIANSU_CHONGQI 减速伞充气阶段。
%
% 物理含义：减速伞完成拉直后，有效阻力面积按充气模型增大，产生开伞力 Fk，
% 同时车辆继续沿轨迹角运动。
%
% 输入：
%   data_stage_3: 减速伞拉直阶段终止状态。
%   js_chuqi_para: 减速伞充气参数向量。
%
% 状态 y = [Xd, Yd, theta, Vw]：车辆位移、轨迹角、车辆速度。
% 终止条件：积分时间达到估算充气时间 t1。
% 迁移注意：ode45_stage4 内部读取 mainchute.k_ks，而主流程传入的是 decechute.ks，
% 这是原始代码里的参数来源不一致点，迁移时建议先保持行为再单独核查。

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
