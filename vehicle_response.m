function solucao = vehicle_response(parametros)
    % vehicle_response  return response (solution) of the vehicle
    % (parameters) in a race.
    
    %% Parameters
    R   = parametros.R;
    r   = parametros.r;
    mc  = parametros.mc;
    me  = parametros.me;
    I   = parametros.I;
    k   = parametros.k;
    Fd  = parametros.Fd;

    %% Time
    [~, ta, td, x0] = race_time(parametros);

    %% Additional parameters
    M   = mc + 2*me + 2*I/R^2;  % Equivalent mass                   [kg]
    K   = k*(r/R)^2;            % Equivalent stiffness              [N/m]
    wn  = sqrt(K/M);            % Natural frequency                 [rad/s]

    %% Acceleration stage
    taVec = 0:0.01:ta;          % Time vector of the acc. stage     [s]  

    xa = x0*cos(wn*taVec);      % Pos. vector of the  acc. stage    [m]
    va = -x0*wn*sin(wn*taVec);  % Speed vector of the acc. stage    [m/s]
    aa = -K/M*xa;               % Accel. vector of the acc. stage   [m/s2]

    Delta = -r/R*xa;            % Spring deformation                [m]

    %% Deceleration stage
    vi = -x0*wn;                % Speed at the begining [m/s]

    tdVec = 0:0.01:td;          % Time vector of the dec. stage     [s] 

    xd = vi*tdVec + (-Fd/M/2)*tdVec.^2; % Position vector           [m]
    vd = vi + (-Fd/M)*tdVec;            % Speed vector              [m/s]
    ad = ones(1,length(tdVec))*-Fd/M;   % Accel. vector             [m/s2]

    %% Updating variables
    % As the initial condition is negative, we subtract the initial
    % condition from the position vectors in order to have an initial
    % position value at zero.

    % Updating - Position vector during the acceleration stage [m]
    xaC = xa - x0;              
    % Updating - Position vector during the deceleration stage [m]
    xdC = xd - x0;              

    % Updating - Time vector of the deceleration stage
    tdVec = tdVec+ta;
    
    %% Output
    solucao.taVec = taVec;
    solucao.xaC = xaC;
    solucao.va = va;
    solucao.aa = aa;
    solucao.tdVec = tdVec;
    solucao.xdC = xdC;
    solucao.vd = vd;
    solucao.ad = ad;
    solucao.Delta = Delta;

end