function [time_jslz,data_jslz,time_3,data_out,data_out_2] = jiansu_straight(data_stage_2,js_strait_para)
% JIANSU_STRAIGHT 减速伞系统拉直阶段。
%
% 物理含义：牵引伞将减速伞系统逐步拉出，释放质量随拉出长度分段变化；
% 车辆和伞系统存在相对速度，由拉直力 FL 连接。
%
% 输入：
%   data_stage_2: 出舱翻转阶段终止状态。
%   js_strait_para: 减速伞拉直参数向量。
%
% 状态 y = [Xd, Yd, theta, Vys, Vw, Lx]：车辆位移、轨迹角、伞速、车速、拉出长度。
% 终止条件：拉出长度 Lx 达到减速伞系统总长度 Lxt。

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

