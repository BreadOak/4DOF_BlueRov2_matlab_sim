function  [Input_trajectory, Output_trajectory] = Controller_PD(Initial_Eta, Desired_trajectory, s_time)
    
    global Tf M C D g J
    
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

    Kp = diag([1 1 1 1]);
    Kd = diag([1 1 1 1]);

    Output_trajectory = {};
    Input_trajectory = {};
    Current_X = [Initial_Eta(1:3); Initial_Eta(6); 0; 0; 0; 0];

    t = 0;
    i = 0;

    for t_proc = 0 : s_time : Tf - s_time

        i = i + 1;

        % Trajectory(matrix -> list)
        T = cell2mat( Desired_trajectory(i) );
        [R , p] = TransToRp(T);
        eul = rotm2eul(R).';
        Desired_X = [p; eul(3); 0; 0; 0; 0];

        % Position & Orientation
        x  = Current_X(1);
        y  = Current_X(2);
        z  = Current_X(3);
        ps = Current_X(4);

        % Velocity
        u = Current_X(5);
        v = Current_X(6);
        w = Current_X(7);
        r = Current_X(8);

        % Set Error
        Err = Desired_X - Current_X;

        % Transformation Matrix of linear velocity
        R = [cos(ps), -sin(ps),        0;
             sin(ps),  cos(ps),        0;
                   0,        0,        1];
        
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
        Crb_L = [ 0,     0,     0,  -m*v;
                  0,     0,     0,   m*u;
                  0,     0,     0,     0;
                m*v,  -m*u,     0,     0];
    
        % 2). Velocity-independent parameterizations
        Crb_V = [ 0,  -m*r,     0,     0;
                m*r,     0,     0,     0;
                  0,     0,     0,     0;
                  0,     0,     0,     0];
    
        % Hydrodynamic Coriolis-Centripetal Matrix(Added term)
        Ca = [  0,      0,      0,  Yvd*v;
                0,      0,      0, -Xud*u;
                0,      0,      0,      0;
           -Yvd*v,  Xud*u,      0,      0];
    
        C = Crb_L + Ca;
    
        % -- [Damping matrix] --
        % Linear Part
        Dl = - [Xu,   0,   0,   0;
                 0,  Yv,   0,   0;
                 0,   0,  Zw,   0;
                 0,   0,   0,  Nr];
    
        % Nonlinear Part
        Dnl = - [Xuu*abs(u),           0,           0,           0;
                          0,  Yvv*abs(v),           0,           0;
                          0,           0,  Zww*abs(w),           0;
                          0,           0,           0,  Nrr*abs(r)];
    
        D = Dl + Dnl;
    
        % -- [Resorting Force] --
        g = [       0;
                    0; 
             -(W - B);
                    0];

        Nu = Current_X(5:8);
        Tau = M*(Kp*Err(1:4) + Kd*J*Err(5:8)) + (C+D)*Nu + g;

        % Dynamics
        [t,S_list] = ode45(@(t,State) Dynamics(t, Current_X, Tau), [0  s_time], Current_X);
        Current_X = S_list(end,(1:8)).';
        Output_trajectory{end+1} = {Current_X};
        Input_trajectory{end+1}  = {Desired_X};

    end
    
end