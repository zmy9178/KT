function dy = ode45_stage1(t,y)
% ODE45_STAGE1 出舱滑动阶段的微分方程右端。
%
% 输入 y = [x, vx, y, vy]。函数返回 dy，对应位移导数和加速度。
% 主要力：牵引伞阻力 Q、地板法向力 N、摩擦力 f、重力分量。
% 角度 plane_angle 使用弧度三角函数 sin/cos。

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

