function dy = ode45_stage2(t,y)
% ODE45_STAGE2 出舱翻转阶段的微分方程右端。
%
% 输入 y = [x, vx, y, vy, theta, omega]。
% 相比 stage1，增加绕支点的转动方程，力矩 Tw 由牵引力、法向力、摩擦力等贡献组成。
% 车辆俯仰角 theta 使用弧度制，输出拼接时再转为角度。

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

