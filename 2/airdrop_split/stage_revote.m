function [time_fz data_fz time_2 data_2] = stage_revote(data_stage_1,para_fanzh)
% STAGE_REVOTE 出舱翻转阶段。
%
% 物理含义：滑动阶段结束后，车辆绕履带/支点继续运动并发生俯仰翻转，直到
% 后端质心相关坐标越过舱口判定位置。
%
% 输入：
%   data_stage_1: stage_moving 的终止状态。
%   para_fanzh: 旧版参数向量；当前主要通过 global 读取参数。
%
% 状态 y = [x, vx, y, vy, theta, omega]：位移、速度、俯仰角、俯仰角速度。
% 终止条件：x_cm_back <= 0。


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
