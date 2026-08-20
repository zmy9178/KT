function dy = ode45_stage5(t,y)
% ODE45_STAGE5 主伞拉直阶段的微分方程右端。
%
% 输入 y = [Xd, Yd, theta, Vys, Vw, Lx]。
% 分段释放模型覆盖减速伞系统与主伞系统，Lx 小于减速伞全长时不计减速伞阻力 Qjs。
% 拉直力 FL = Dmi * 相对速度平方 + Fsh。

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

