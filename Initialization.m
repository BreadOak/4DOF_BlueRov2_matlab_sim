
global M C D g J

time = 100;
length = 100;

K = zeros(64);
Bar_A = zeros(64);
Bar_G = zeros(64);

% Control frequency is defined as follows %
s_freq = 100;      % Hz
s_time = 1/s_freq; % sec

% Model paramter
m = 10;      % kg
W = 98.1;    % N
B = 100.6;   % N
Ix = 0.16;   % kg*m^2
Iy = 0.16;   % kg*m^2
Iz = 0.16;   % kg*m^2

rg = [0, 0, 0];    % m
rb = [0, 0, 0.02]; % m
BG = rg - rb;      % m
xg = BG(1);        % m
yg = BG(2);        % m
zg = BG(3);        % m

Xud = -5.5;   % kg
Yvd = -12.7;  % kg
Zwd = -14.57; % kg
Kpd = -0.12;  % kg*m^2/rad
Mqd = -0.12;  % kg*m^2/rad
Nrd = -0.12;  % kg*m^2/rad

Xu = -4.03;   % N*s/m
Yv = -6.22;   % N*s/m
Zw = -5.18;   % N*s/m
Kp = -0.07;   % N*s/rad
Mq = -0.07;   % N*s/rad
Nr = -0.07;   % N*s/rad

Xuu = -18.18; % N*s^2/m^2
Yvv = -21.66; % N*s^2/m^2
Zww = -36.99; % N*s^2/m^2
Kpp = -1.55;  % N*s^2/rad^2
Mqq = -1.55;  % N*s^2/rad^2
Nrr = -1.55;  % N*s^2/rad^2

% Position & Orientation
x  = zeros(1,length);
y  = zeros(1,length);
z  = zeros(1,length);
ps = zeros(1,length);

% Velocity
u = zeros(1,length);
v = zeros(1,length);
w = zeros(1,length);
r = zeros(1,length);

tau1 = zeros(1,time);
tau2 = zeros(1,time);
tau3 = zeros(1,time);
tau4 = zeros(1,time);

for i = 2 : time

    tau1(i) = randn;
    tau2(i) = randn;
    tau3(i) = randn;
    tau4(i) = randn;

    X_init   = zeros(8,1);
    Tau_init = [tau1(i); tau2(i); tau3(i); tau4(i)];

    for j = 2 : length

        % Transformation Matrix of linear velocity
        R = [cos(ps(j)), -sin(ps(j)),        0;
             sin(ps(j)),  cos(ps(j)),        0;
                   0,           0,           1];
        
        % Transformation of angular velocity
        T = 1;
    
        % Transformation Matrix (Body -> World)
        J = [         R,  zeros(3,1);
               zeros(1,3),         T];
    
        % -- [Mass matrix] --
        % Rigid-Body System Inertia Matrix
        Mrb = [m,     0,     0,     0;
               0,     m,     0,     0;
               0,     0,     m,     0;
               0,     0,     0,    Iz];  
    
        % Hydrodynamic System Inertia Matrix(Added term)
        Ma  = - [Xud,   0,   0,   0;
                   0, Yvd,   0,   0;
                   0,   0, Zwd,   0;
                   0,   0,   0, Nrd];
    
        M = Mrb + Ma;
    
        % -- [Coriolis force matrix] --
        % Rigid-Body Coriolis and Centripetal Matrix
    
        % 1). Largrangian parameterizations
        Crb_L = [ 0,      0,     0,  -m*v(j);
                  0,      0,     0,   m*u(j);
                  0,      0,     0,     0   ;
               m*v(j), -m*u(j),  0,     0   ];
    
        % 2). Velocity-independent parameterizations
        Crb_V = [ 0,  -m*r(j),  0,     0;
                m*r(j),  0,     0,     0;
                  0,     0,     0,     0;
                  0,     0,     0,     0];
    
        % Hydrodynamic Coriolis-Centripetal Matrix(Added term)
        Ca = [  0,         0,         0,  Yvd*v(j);
                0,         0,         0, -Xud*u(j);
                0,         0,         0,      0   ;
           -Yvd*v(j),  Xud*u(j),      0,      0   ];
    
        C = Crb_L + Ca;
    
        % -- [Damping matrix] --
        % Linear Part
        Dl = - [Xu,   0,   0,   0;
                 0,  Yv,   0,   0;
                 0,   0,  Zw,   0;
                 0,   0,   0,  Nr];
    
        % Nonlinear Part
        Dnl = - [Xuu*abs(u(j)),           0,           0,              0;
                          0,  Yvv*abs(v(j)),           0,              0;
                          0,              0,  Zww*abs(w(j)),           0;
                          0,              0,           0,  Nrr*abs(r(j))];
    
        D = Dl + Dnl;
    
        % -- [Resorting Force] --
        g = [       0;
                    0; 
             -(W - B);
                    0];

        % Dynamics
        [t,S_list] = ode45(@(t,State) Dynamics(t, X_init, Tau_init), [0  s_time], X_init);
        Current_X = S_list(end,(1:8)).';
        
        % Position & Orientation
        x(j)  = Current_X(1);
        y(j)  = Current_X(2);
        z(j)  = Current_X(3);
        ps(j) = Current_X(4);

        % Velocity
        u(j) = Current_X(5);
        v(j) = Current_X(6);
        w(j) = Current_X(7);
        r(j) = Current_X(8);

        % Collect data
        s0 = [x(j-1); y(j-1); z(j-1); ps(j-1); u(j-1); v(j-1); w(j-1); r(j-1)];
        sn = [x(j);     y(j);   z(j);   ps(j);   u(j);   v(j);   w(j);   r(j)];
        u0 = Tau_init;
        un = u0;

        % Closed form solution
        Bar_A = Bar_A + RobotArm_Basis(sn,un)*RobotArm_Basis(s0,u0)';
        Bar_G = Bar_G + RobotArm_Basis(s0,u0)*RobotArm_Basis(s0,u0)';
            
        % Update
        X_init   = Current_X;
        Tau_init = un;

    end

    K_test = Bar_A*pinv(Bar_G);
    K = (K + K_test);

end

K = K/time;

disp(K)


