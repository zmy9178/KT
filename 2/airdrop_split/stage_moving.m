function [time_pd,data_pd,time_1,data_1] = stage_moving(para_pdong)
% STAGE_MOVING 出舱滑动阶段。
%
% 物理含义：车辆在货舱内受到牵引伞阻力、重力分量、地板法向力和摩擦力作用，
% 沿货舱方向滑动，直到车辆前端/装载基准到达舱口判定位置。
%
% 输入：
%   para_pdong: 旧版参数向量；当前实现基本不使用该向量，而是读取 global。
%
% 输出：
%   time_pd, data_pd: 滑动阶段终止时间和终止状态。
%   time_1, data_1: ode45 返回的完整时间和状态历程。
%
% 状态 y = [x, vx, y, vy]：水平位移、水平速度、垂向位移、垂向速度。
% 终止条件：data_1(i,1) + drop_envi.LoadPosi_x <= 0。

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
