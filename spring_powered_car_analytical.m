%% Spring powered car - Analytical
% Response and race time of the spring powered car according to the
% analytical solution.
%
%%

clear ; close all ; clc

%% Parameters

parameters = load_parameters();

%% Race time
% Total race time and time during each stage (acceleration and
% deceleration).

[tc, ta, td, x0] = race_time(parameters);

disp('Total time [s]:')
disp(tc)
disp('Acceleration stage time [s]:')
disp(ta)
disp('Deceleration stage time [s]:')
disp(td)

%% Plots

plot_response(parameters)
