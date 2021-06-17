function parameters = load_parameters()
    % Parameters
    % Constants
    g   = 9.81;             % Gravity                           [m/s2]
    mu  = 0.25;             % Friction coefficient              [-]
    d   = 5;                % Race length                       [m]
    % Distances
    b   = 0.18;             % Distance CG to front axle D       [m]
    c   = 0.14;             % Distance CG to rear axle T        [m] 
    L   = b+c;              % Wheelbase T-D                     [m]
    h   = 0.088;            % Height CG                         [m]
    R   = 0.08;             % Wheel radius                      [m]
    r   = 0.006;            % Axle radius                       [m]
    % Inertial
    mc  = 1.05;             % Mass of the chassis               [kg]
    me  = 0.391;            % Mass of the axle                  [kg]
    I   = 0.0007;             % Moment of inertia               [kgm2]
    % Spring
    k   = 100;              % Spring constant                   [N/m]
    DeltaMax = 0.2;         % Spring max. deformation           [m]
    % Resistive force
    Fd  = 0.3;              % Resistive force. Dry friction     [N]

    % Output
    % Struct
    parameters.g    = g;
    parameters.mu   = mu;
    parameters.d    = d;
    parameters.b    = b;
    parameters.L    = L;
    parameters.h    = h;
    parameters.R    = R;
    parameters.r    = r;
    parameters.mc    = mc;
    parameters.me   = me;
    parameters.I    = I;
    parameters.k    = k;
    parameters.DeltaMax = DeltaMax;
    parameters.Fd   = Fd;

end