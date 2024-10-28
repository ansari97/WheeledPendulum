close all
clear
clc

%% define wheel parameters
L = 0.30; % total length of the rod
l = L/2; % length from pivot to center of mass
m = 1;
I_r = 1/3*m*L^2;
r = 0.05;
M = 0.25;
I_w = 1/2*M*r^2;

wheel_param = [l, m, I_r, r, M, I_w];


%% define initial conditions
init_x = 0;
init_ang = 0.5;
init_vel = 0; 
init_ang_vel = 0;

init_con = [init_x; init_ang; init_vel; init_ang_vel];

%% Input
T = 100;

%% solver settings

time_interval = [0 5];
% create ode object
E = odeEvent(EventFcn=@(t, q)collisionEvent(t, q, wheel_param), ...
    Direction="ascending", ...
    Response="stop");

F = ode(ODEFcn = @(t, q) diffFunc(t, q, wheel_param, T), InitialValue = init_con, EventDefinition = E,  Solver = 'ode45');
% set solver options
F.SolverOptions.MaxStep = 0.01;

% Solve ODE
q_sol = solve(F, time_interval(1), time_interval(2), Refine=8);

%% solver results
state = q_sol.Solution;
% matrix of solution values (time as row 1, angle as 2, ang_vel as 3)
t = q_sol.Time;
sol = [t; state(1, :); state(2, :); state(3, :); state(4, :)];

plot(t, state(1, :))
figure;
plot(t, state(2, :))
figure;
plot(t, state(3, :))
figure;
plot(t, state(4, :))

drawFigure(wheel_param, state)
