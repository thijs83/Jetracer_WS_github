close all
clear 
clc
ax.TickLabelInterpreter='latex';
%% USEFUL QUANTITIES

% motor coefficient identified by master students
% a_mot = 60/1.63 ;
% b_mot = 1.54/1.63 ;
% c_mot = a_mot*0.1 ;

dt = .1 ;       %[s]
m = 1.63 ;      %[kg]
L = 0.175 ;     %[m]
L_lat = 0.12 ;  %[m]
lr = 0.45*L ;   %[m]  
lf = L - lr ;   %[m]
Iz = 0.0061 ;   %[Kg*m^2]

%% LOAD DATA
%%% raw data not filtered
% load('matlab_data1_LOW_RAW.mat')
load('matlab_data2_LUNGO_RAW.mat')

%%% old data collected with joystick 
% load('matlab_data1_old.mat')
% load('matlab_data2_old.mat')

%%% Filtered data
% load('matlab_data1_LUNGO.mat')

%%% In case you want combine two batches
% a = load('matlab_data1_LOW.mat') ;
% b = load('matlab_data2_HIGH.mat') ;
% data1 = [ a.data1 b.data1 ] ;

%% extract the necessary qunatities from the matrix data
data1 = data1' ;
% data1 = data1(1:1000,:) ; % to restrict the number of data used for the fittnig
tau = data1(:,1) ;
msk = (tau < 0.1) ;
tau(msk) = 0.10 ; % when the safety is disengaged we impose that tau = 0.1 instead of zero so the model can learn that
% no movement is produced for tau < 0.1
steer = data1(:,2) ; % [rad] (the minus sign is already included)
vx_local = data1(:,3) ;
vy_local = data1(:,4) ; % should I change the sign of vy???
w_IMU = data1(:,5) ;
theta = data1(:,6) ;
w_opti = data1(:,7) ;
v = sqrt(vx_local.^2 + vy_local.^2) ;
delta = steer ; % just a matter of notation

% useful values to normalize the cost function
max_vx = max(abs(vx_local)) ;
max_vy = max(abs(vy_local)) ;
max_w = max(abs(w_IMU)) ;

N = size(delta, 1) ;
time = 0:dt:(N-1)*dt ;
time = time' ;

%% PLOT DATA
figure()
subplot(2,1,1)
plot(time, rad2deg(delta), 'LineWidth',2)
grid on
tit = title('$\delta [deg]$', 'FontSize',20) ;
set(tit,'Interpreter','latex');
subplot(2,1,2)
plot(time, tau, 'LineWidth',2)
grid on
tit = title('$\tau$', 'FontSize',20) ;
set(tit,'Interpreter','latex');
% 
figure()
plot(time, vx_local, 'LineWidth', 2)
tit = title('Local speeds', 'FontSize', 20) ;
grid on
hold on
plot(time, vy_local, 'LineWidth', 2)
leg = legend('$v_x$', '$v_y$', 'FontSize', 16) ;
set(tit,'Interpreter','latex');
set(leg,'Interpreter','latex');
% 
figure()
plot(time, theta, 'LineWidth', 1)
title('\theta')
% 
figure()
plot(time, w_IMU, 'LineWidth', 2)
hold on
% plot(time, w_opti, 'LineWidth', 1)
title('\omega')
% useful plot to understand the sign of slip angles
figure()
plot(time, w_IMU.*lr, 'LineWidth', 2)
hold on
plot(time, vy_local, 'LineWidth', 2)
hold on
plot(time, delta, 'g', 'LineWidth', 2)
hold on
leg = legend('$\omega$', '$v_y$', 'steer', 'FontSize', 16) ;
set(leg,'Interpreter','latex');

%% compute the slip angles
alpha_f = (delta - atan2((vy_local + lf*w_IMU), vx_local)) ;
alpha_r = atan2( (-vy_local + lr*w_IMU), vx_local ) ;

figure()
ax1 = subplot(2,1,1) ;
plot(time, rad2deg(alpha_f), 'LineWidth',2)
grid on
tit = title('$\alpha_f [deg]$', 'FontSize',20) ;
set(tit,'Interpreter','latex');
ax2 = subplot(2,1,2) ;
plot(time, rad2deg(alpha_r), 'LineWidth',2)
grid on
tit = title('$\alpha_r [deg]$', 'FontSize',20) ;
set(tit,'Interpreter','latex');

linkaxes([ax1 ax2], 'y')

%% FITTING with fmincon

N_par = 8 ; % number of parameters to be identified

x0 = rand(1,N_par) ; % randomize the initial condition
% --> could be wise to impose the motor parameters identified by bachelor
% as initial condition

lb = zeros(1,N_par) ; % lower bound
ub = 100.*ones(1,N_par) ; % upper bound
% --> should the coefficient be all positive?

N_pred = 3000 ; % number of step ahead prediction for the fitting

% Define the objective function for optimization
J = @(x) objective_4_DYN(x, tau, delta, vx_local, vy_local, w_IMU, N_pred, a_mot, b_mot, c_mot) ;

% Call the solver to minimize the objective function given constraints
% [ x_opt, fval ] = fmincon(J, x0, [], [], [], [], lb, ub, [], []) ;
[ x_opt, fval ] = fmincon(J, x0, [], [], [], [], [], [], [], []) ;

%% Test the parameters obtained from the optimization
% initialize to zero the velocity vectors
vx_local_anal = zeros(length(tau),1) ;
vy_local_anal = zeros(length(tau),1) ;
w_IMU_anal = zeros(length(tau),1) ;

alpha_f_anal = zeros(length(tau),1) ;
alpha_r_anal = zeros(length(tau),1) ;

% impose the initial conditions
vx_local_anal(1) = vx_local(1) ;
vy_local_anal(1) = vy_local(1) ;
w_IMU_anal(1) = w_IMU(1) ;

alpha_f_anal(1) = (delta(1) - atan2((vy_local_anal(1) + lf*w_IMU_anal(1)), vx_local_anal(1))) ;
alpha_r_anal(1) = atan2( (-vy_local_anal(1) + lr*w_IMU_anal(1)), vx_local_anal(1) ) ;
%
N = length(tau) ;
N_pred = 3000 ; % how many step-ahead we would our model to predict
msk = [] ;
for ii=1:N
    if mod(ii,N_pred) == 0
        msk = [ msk , ii] ;
    end
end

h = 0.1 ; % time discretization

for ii=2:length(tau)
    % if ii is a multiple of N_pred we do not predict the value but we
    % impose it equal to the value taken from the data
    if sum(msk==ii) > 0

        vx_local_anal(ii) = vx_local(ii) ;
        vy_local_anal(ii) = vy_local(ii) ;
        w_IMU_anal(ii) = w_IMU(ii) ;

    else
%%% LINEAR TYRE MODEL --> only 4 parameters to be identified instead of 8
%         Fx = ( (x_opt(3).*tau(ii-1) - x_opt(4).*vx_local_anal(ii-1)) - 0.1.*x_opt(3) ).*m ;  % Fx = a*tau - b*vx - c
%         Ff = x_opt(1).*alpha_f_anal(ii-1) ; % Ff = Cf*alpha_f_anal
%         Fr = x_opt(2).*alpha_r_anal(ii-1) ; % Fr = Cr*alpha_r_anal

%%% NON LINEAR TYRE MODEL
        Fx = ( (x_opt(7) - x_opt(8).*vx_local_anal(ii-1)).*tau(ii-1) - x_opt(7).*0.1 ).*m ;  % Fx = (a - b*vx)*tau - c
%         Fx = ( (x_opt(7) - x_opt(8).*vx_local_anal(ii-1)).*tau(ii-1) ).*m ;                % Fx = a*tau - b*vx
        Ff = x_opt(1).*sin(x_opt(3)*atan2( x_opt(5).*alpha_f_anal(ii-1),1) ) ;
        Fr = x_opt(2).*sin(x_opt(4)*atan2(x_opt(6).*alpha_r_anal(ii-1), 1) ) ; 

%%% MODEL WITH REAR DRIVEN WHEELs
%             vx_local_anal(ii) = vx_local_anal(ii-1) + h*( 1/m .* ( Fx - Ff.*sin(delta(ii-1)) + m.*vy_local_anal(ii-1).*w_IMU_anal(ii-1) ) ) ;
%             vy_local_anal(ii) = vy_local_anal(ii-1) + h*( 1/m .* ( Ff.*cos(delta(ii-1)) + Fr - m.*vx_local_anal(ii-1).*w_IMU_anal(ii-1) ) ) ;
%             w_IMU_anal(ii) = w_IMU_anal(ii-1) + h*( 1/Iz .* (  lf.*( Ff.*cos(delta(ii-1)) ) - Fr.*lr ) ) ;

% MODEL WITH 4 DRIVEN WHEELS
        vx_local_anal(ii) = vx_local_anal(ii-1) + h*( 1/m .* ( Fx/2 + Fx/2.*cos(delta(ii-1)) - Ff.*sin(delta(ii-1)) + m.*vy_local_anal(ii-1).*w_IMU_anal(ii-1) ) ) ;
        vy_local_anal(ii) = vy_local_anal(ii-1) + h*( 1/m .* ( Fx/2.*sin(delta(ii-1)) + Ff.*cos(delta(ii-1)) + Fr - m.*vx_local_anal(ii-1).*w_IMU_anal(ii-1) ) ) ;
        w_IMU_anal(ii) = w_IMU_anal(ii-1) + h*( 1/Iz .* (  lf.*( Ff.*cos(delta(ii-1)) + Fx/2.*sin(delta(ii-1)) ) - Fr.*lr ) ) ;

    end

    % predicted slip angles
    alpha_f_anal(ii) = (delta(ii) - atan2((vy_local_anal(ii) + lf*w_IMU_anal(ii)), vx_local_anal(ii))) ;
    alpha_r_anal(ii) = atan2( (-vy_local_anal(ii) + lr*w_IMU_anal(ii)), vx_local_anal(ii) ) ;

end

cost_vx = (vx_local_anal - vx_local)'*(vx_local_anal - vx_local)
cost_vy = (vy_local_anal - vy_local)'*(vy_local_anal - vy_local)
cost_w = (w_IMU_anal - w_IMU)'*(w_IMU_anal - w_IMU)

%% PLOT THE PREDICTION RESULTS
figure()
subplot(3,1,1)
plot(time, vx_local_anal, 'LineWidth',2)
hold on
plot(time, vx_local,'LineWidth',2)
legend('analytic', 'data')
tit = title('$v_x$', FontSize=20) ;
set(tit,'Interpreter','latex');

subplot(3,1,2)
plot(time, vy_local_anal, 'LineWidth',2)
hold on
plot(time, vy_local,'LineWidth',2)
legend('analytic', 'data')
tit = title('$v_y$', FontSize=20) ;
set(tit,'Interpreter','latex');

subplot(3,1,3)
plot(time, w_IMU_anal, 'LineWidth',2)
hold on
plot(time, w_IMU,'LineWidth',2)

legend('analytic', 'data')
tit = title('$\omega$', FontSize=20) ;
set(tit,'Interpreter','latex') ;
% slip angles
figure()
subplot(2,1,1)
hold on
plot(time, rad2deg(alpha_f_anal), 'LineWidth',2, color='r')
grid on
tit = title('$\alpha_f [deg]$', 'FontSize',20) ;
set(tit,'Interpreter','latex');
subplot(2,1,2)
hold on
plot(time, rad2deg(alpha_r_anal), 'LineWidth',2, color='r')
grid on
tit = title('$\alpha_r [deg]$', 'FontSize',20) ;
set(tit,'Interpreter','latex');


%% PLOT TYRE FORCES

Df = x_opt(1) ;
Dr = x_opt(2) ;
Cf = x_opt(3) ;
Cr = x_opt(4) ;
Bf = x_opt(5) ;
Br = x_opt(6) ;
% 
% Cf = x_opt(1) ;
% Cr = x_opt(2) ;

slip_ang = linspace(-2*pi, 2*pi, 1000) ;
Ff = @(alpha) Df*sin(Cf*atan2(Bf*alpha,1)) ;
Fr =  @(alpha) Dr*sin(Cr*atan2(Br*alpha,1)) ;
% Ff = @(alpha) Cf*alpha ;
% Fr =  @(alpha) Cr*alpha ;

% Ff
figure()
subplot(1,2,1)
plot(rad2deg(slip_ang), Ff(slip_ang), 'LineWidth',2) ;
xlabel('$\alpha [deg]$', 'Interpreter','latex','FontSize',15) ;
ylabel('$ Force [N]$', 'Interpreter','latex','FontSize',15) ;
tit = title('$F_f$','FontSize',20) ;
set(tit,'Interpreter','latex')
grid on
xlim([-1,1])
xlim([-90,90])
subplot(1,2,2)
plot(rad2deg(slip_ang), Fr(slip_ang), 'LineWidth',2)
xlabel('$\alpha [deg]$', 'Interpreter','latex','FontSize',15) ;
ylabel('$ Force [N]$', 'Interpreter','latex','FontSize',15) ;
tit = title('$F_r$','FontSize',20) ;
set(tit,'Interpreter','latex')
grid on
xlim([-1,1])
xlim([-90,90])








