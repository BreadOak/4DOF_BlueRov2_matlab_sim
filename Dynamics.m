function  dxdt = Dynamics(t, state, input)  

    global M C D g J
    
    Nu  = state(5:8);
    Tau = input;

    dxdt1 = J*Nu;
    dxdt2 = inv(M)*(Tau - (C+D)*Nu - g);
    dxdt  = [dxdt1; dxdt2];
    
end