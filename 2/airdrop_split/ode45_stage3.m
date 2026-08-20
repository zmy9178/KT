function dy = ode45_stage3(t,y)
% ODE45_STAGE3 减速伞拉直阶段的微分方程右端。
%
% 输入 y = [Xd, Yd, theta, Vys, Vw, Lx]。
% 根据拉出长度 Lx 计算已释放质量 Me 和质量释放率 Dmi，再由相对速度计算拉直力 FL。
% 该函数同时估计车辆俯仰角加速度，供 jiansu_straight 后处理输出。

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

