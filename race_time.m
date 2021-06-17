function [tc, ta, td, x0] = race_time(parameters)
    % race_time  calculates race time: total, aceleration stage and
    % deceleration stage (if exists).

    %% Parameters
    g   = parameters.g;
    mu  = parameters.mu;
    d   = parameters.d;
    b   = parameters.b;
    L   = parameters.L;
    h   = parameters.h;
    R   = parameters.R;
    r   = parameters.r;
    mc  = parameters.mc;
    me  = parameters.me;
    I   = parameters.I;
    k   = parameters.k;
    DeltaMax = parameters.DeltaMax;
    Fd  = parameters.Fd;

    %% Additional parameters
    M   = mc + 2*me + 2*I/R^2;  % Equivalent mass           [kg]
    K   = k*(r/R)^2;            % Equivalent stiffness      [N/m]
    wn  = sqrt(K/M);            % Natural frequency         [rad/s]

    %% Initial condition
    % Initial position that generates maximum accleration
    % at the start [m]
    x0  = -M/K*(mu*(mc + 2*me)*g*b/L)/(mc + 2*me + I/R^2 - mu*(mc + 2*me)*h/L);

    % This automatic evaluation of the initial position may generate
    % a huge deformation.
    % Let's limit the deformation:
    Delta = -r/R*x0;            % Spring deformation        [m]
    if Delta > DeltaMax
        x0 = -R/r*DeltaMax;
    end

    %% Two stages
    % Time of the two stages
    % Acceleration stage [s]
    ta = (pi/2)/wn;              
    % Deceleration stage [s]
    td = (-x0*wn - sqrt((x0*wn)^2 - 2*Fd/M*(d + x0)))/(Fd/M); 

    %% One stage
    %  If the race has only the acceleration stage, the time of the
    %  deceleration stage is zero.
    % This occurs when the time of deceleration stage is negative.
    if td < 0
        ta = acos((x0+d)/x0)/wn;
        td = 0;
    end

    %% Total time
    % Total race time [s]
    tc = ta + td;

end