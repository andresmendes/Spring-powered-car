%% Spring powered car - Analytical and numerical
% Response and race time of the spring powered car according to the
% analytical and numerical solutions.
%
%%

clear ; close all ; clc

%% Parameters

parameters = load_parameters();

%% Race time
% Total race time and time during each stage (acceleration and
% deceleration).

[tc, ta, td, x0] = race_time(parameters);

disp('Total time (Analytical) [s]:')
disp(tc)
disp('Acceleration stage time (Analytical) [s]:')
disp(ta)
disp('Deceleration stage time (Analytical) [s]:')
disp(td)

%% Simulation

time = 0:0.1:10;            % Time vector [s] 

% Initial speed. Vehicle at rest.
dx0 = 0;

options = odeset('Events',@(t,z) acceleration_stage_end(t,z,parameters));
[TOUTa,YOUTa] = ode45(@(t,z) vehicle_acceleration(t,z,parameters),time,[x0 dx0],options);

xSima = YOUTa(:,1)-x0;
vSima = YOUTa(:,2);
aSima = zeros(length(TOUTa),1);
for i=1:length(TOUTa)
     dz = vehicle_acceleration(TOUTa(i),YOUTa(i,:),parameters);
     aSima(i) = dz(2);
end

Delta = -parameters.r/parameters.R*(xSima+x0); % Spring deformation                [m]

if parameters.d-xSima(end)>0
    options = odeset('Events',@(t,z) deceleration_stage_end(t,z,parameters));
    [TOUTd,YOUTd] = ode45(@(t,z) vehicle_deceleration(t,z,parameters),time+TOUTa(end),[xSima(end) vSima(end)],options);

    xSimd = YOUTd(:,1);
    vSimd = YOUTd(:,2);
    aSimd = zeros(length(TOUTd),1);
    for i=1:length(TOUTd)
         dz = vehicle_deceleration(TOUTd(i),YOUTd(i,:),parameters);
         aSimd(i) = dz(2);
    end
    disp('Total time (Simulation) [s]:')
    disp(TOUTd(end))
    disp('Acceleration stage time (Simulation) [s]:')
    disp(TOUTa(end))
    disp('Deceleration stage time (Simulation) [s]:')
    disp(TOUTd(end)-TOUTa(end))
else
    disp('Total time (Simulation) [s]:')
    disp(TOUTa(end))
    disp('Acceleration stage time (Simulation) [s]:')
    disp(TOUTa(end))
    disp('Deceleration stage time (Simulation) [s]:')
    disp(0)
end

%% Results

% Analytical response
plot_response(parameters)

% Adding numerical response
subplot(4,1,1)
    hold on ; grid on ; box on
    plot(TOUTa,xSima,'r--')
    if parameters.d-xSima(end)>0
        plot(TOUTd,xSimd,'m--')
        legend('Acc. Analytical','Dec. Analytical','Acc. Simulation','Dec. Simulation','Location','EastOutside')
        set(gca,'Xlim',[0 TOUTd(end)],'Ylim',[0 1.1*max(xSimd)])
    else
        legend('Acc. Analytical','Acc. Simulation','Location','EastOutside')
        set(gca,'Xlim',[0 TOUTa(end)],'Ylim',[0 1.1*max(xSima)])
    end
subplot(4,1,2)
    hold on ; grid on ; box on
    plot(TOUTa,vSima,'r--')
    if parameters.d-xSima(end)>0
        plot(TOUTd,vSimd,'m--')
        legend('Acc. Analytical','Dec. Analytical','Acc. Simulation','Dec. Simulation','Location','EastOutside')
        set(gca,'Xlim',[0 TOUTd(end)])
    else
        legend('Acc. Analytical','Acc. Simulation','Location','EastOutside')
        set(gca,'Xlim',[0 TOUTa(end)])
    end
subplot(4,1,3)
    hold on ; grid on ; box on
    plot(TOUTa,aSima,'r--')
    if parameters.d-xSima(end)>0
        plot(TOUTd,aSimd,'m--')
        legend('Acc. Analytical','Dec. Analytical','Acc. Simulation','Dec. Simulation','Location','EastOutside')
        set(gca,'Xlim',[0 TOUTd(end)])
    else
        legend('Acc. Analytical','Acc. Simulation','Location','EastOutside')
        set(gca,'Xlim',[0 TOUTa(end)])
    end
subplot(4,1,4)
    hold on ; grid on ; box on
    plot(TOUTa,Delta,'r--')
    if parameters.d-xSima(end)>0
        plot([TOUTd(1) TOUTd(end)],[0 0],'m--')
        legend('Acc. Analytical','Dec. Analytical','Acc. Simulation','Dec. Simulation','Location','EastOutside')
        set(gca,'Xlim',[0 TOUTd(end)])
    else
        legend('Acc. Analytical','Acc. Simulation','Location','EastOutside')
        set(gca,'Xlim',[0 TOUTa(end)])
    end

%% Auxiliary functions

function dz = vehicle_acceleration(~,z,parameters)

    %% Parameters
    R   = parameters.R;
    r   = parameters.r;
    mc  = parameters.mc;
    me  = parameters.me;
    I   = parameters.I;
    k   = parameters.k;
    Fd  = parameters.Fd;

    %% Additional parameters
    M   = mc + 2*me + 2*I/R^2;      % Equivalent mass           [kg]
    K   = k*(r/R)^2;                % Equivalent stiffness      [N/m]
    
    %% Dynamics
    dz(1,1) = z(2);
%     dz(2,1) = -K/M*z(1);            % Without friction!
    dz(2,1) = -K/M*z(1) -Fd/M;      % With friction!
    
end
function dz = vehicle_deceleration(~,z,parameters)

    %% Parameters
    R   = parameters.R;
    mc  = parameters.mc;
    me  = parameters.me;
    I   = parameters.I;
    Fd  = parameters.Fd;

    %% Additional parameter
    M = mc + 2*me + 2*I/R^2;
    
    %% Dynamics
    dz(1,1) = z(2);
    dz(2,1) = -Fd/M;
    
end

function [position,isterminal,direction] = acceleration_stage_end(~,y,parameters)
    % Retrieving initial condition
    [~,~,~,x0] = race_time(parameters);
    % Stops when position equal to zero or race finish line is reached.
    position = (parameters.d-(y(1)-x0))*y(1); % The value that we want to be zero
    isterminal = 1;  % Halt integration 
    direction = 0;   % The zero can be approached from either direction
end

function [position,isterminal,direction] = deceleration_stage_end(~,y,parameters)
    % Stops when speed equal to zero or race finish line is reached.
    position = (parameters.d-y(1)); % The value that we want to be zero
    isterminal = 1;  % Halt integration 
    direction = 0;   % The zero can be approached from either direction
end
