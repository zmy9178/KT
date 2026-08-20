function [time_mslz,data_mslz,time_5,data_out,data_out_2] = zhusan_straight(data_stage_2,ms_strait_para,data_jslz)
% ZHUSAN_STRAIGHT 主伞系统拉直阶段。
%
% 物理含义：减速伞已工作后继续拉出主伞系统；释放质量模型覆盖减速伞绳、
% 减速伞伞边、减速伞伞衣、主伞绳、主伞伞边、主伞伞衣多个区间。
%
% 输入：
%   data_stage_2: 减速伞充气阶段终止状态。
%   ms_strait_para: 主伞拉直参数向量。
%   data_jslz: 减速伞拉直阶段数据，用于继承上一阶段拉出长度/速度信息。
%
% 状态 y = [Xd, Yd, theta, Vys, Vw, Lx]。
% 终止条件：拉出长度 Lx 达到减速伞系统 + 主伞系统总长度 Lxt。

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
  
