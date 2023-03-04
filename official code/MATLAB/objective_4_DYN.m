function obj = objective_4_DYN(x, tau, delta, vx_local, vy_local, w_IMU, N_pred, a_mot, b_mot, c_mot)


h = 0.1 ;
m = 1.63 ;
L = 0.175 ;
lr = 0.45*L ; %[m]
lf = L - lr ; %[m]
N = length(tau) ;
Iz = 0.0061  ; %[Kg*m^2]

% useful values to normalize the cost function
max_vx = max(abs(vx_local))^2 ;
max_vy = max(abs(vy_local))^2 ;
max_w = max(abs(w_IMU))^2 ;

% initialize to zero the velocity vectors
vx_local_anal = zeros(length(tau),1) ;
vy_local_anal = zeros(length(tau),1) ;
w_IMU_anal = zeros(length(tau),1) ;

alpha_f = zeros(length(tau),1) ;
alpha_r = zeros(length(tau),1) ;

% impose the initial conditions
vx_local_anal(1) = vx_local(1) ;
vy_local_anal(1) = vy_local(1) ;
w_IMU_anal(1) = w_IMU(1) ;
alpha_f(1) = (delta(1) - atan2((vy_local_anal(1) + lf*w_IMU_anal(1)), vx_local_anal(1))) ;
alpha_r(1) = atan2( (-vy_local_anal(1) + lr*w_IMU_anal(1)), vx_local_anal(1) ) ;

msk = [] ;
for ii=1:N
    if mod(ii,N_pred) == 0
        msk = [ msk , ii] ;
    end
end

% Start the forward integration:
for ii=2:length(tau)
    % if ii is a multiple of N_pred we do not predict the value but we
    % impose it equal to the value taken from the data
    if sum(msk==ii) > 0

        vx_local_anal(ii) = vx_local(ii) ;
        vy_local_anal(ii) = vy_local(ii) ;
        w_IMU_anal(ii) = w_IMU(ii) ;

    else
        %%% LINEAR TYRE MODEL --> only 4 parameters to be identified instead of 8
        %         Fx = @(x)  ( (x(3).*tau(ii-1) - x(4).*vx_local_anal(ii-1)) - 0.1.*x(3) ).*m ;  % Fx = a*tau - b*vx - c
        %         Ff = @(x)  x(1).*alpha_f(ii-1) ; % Ff = Cf*alpha_f
        %         Fr = @(x)  x(2).*alpha_r(ii-1) ; % Fr = Cr*alpha_r

        %%% NON LINEAR TYRE MODEL
        Fx = @(x) ( (x(7) - x(8).*vx_local_anal(ii-1)).*tau(ii-1) - x(7).*0.1  ).*m ;  % Fx = a*tau - b*vx - c
        %         Fx = @(x) ( (x(7) - x(8).*vx_local_anal(ii-1)).*tau(ii-1) ).*m ;     % Fx = (a - b*vx)*tau
        Ff = @(x) x(1).*sin(x(3)*atan2( x(5).*alpha_f(ii-1),1) ) ;
        Fr = @(x) x(2).*sin(x(4)*atan2(x(6).*alpha_r(ii-1), 1) ) ; 

        %%% MODEL WITH REAR DRIVEN WHEELs
        vx_local_anal(ii) = vx_local_anal(ii-1) + h*( 1/m .* ( Fx(x)/2 + Fx(x)/2.*cos(delta(ii-1)) - Ff(x).*sin(delta(ii-1)) + m.*vy_local_anal(ii-1).*w_IMU_anal(ii-1) ) ) ;
        vy_local_anal(ii) = vy_local_anal(ii-1) + h*( 1/m .* ( Fx(x)/2.*sin(delta(ii-1)) + Ff(x).*cos(delta(ii-1)) + Fr(x) - m.*vx_local_anal(ii-1).*w_IMU_anal(ii-1) ) ) ;
        w_IMU_anal(ii) = w_IMU_anal(ii-1) + h*( 1/Iz .* (  lf.*( Ff(x).*cos(delta(ii-1)) + Fx(x)/2.*sin(delta(ii-1)) ) - Fr(x).*lr) ) ;

        % MODEL WITH 4 DRIVEN WHEELS
        %         vx_local_anal(ii) = vx_local_anal(ii-1) + h*( 1/m .* ( Fx(x) - Ff(x).*sin(delta(ii-1)) + m.*vy_local_anal(ii-1).*w_IMU_anal(ii-1) ) ) ;
        %         vy_local_anal(ii) = vy_local_anal(ii-1) + h*( 1/m .* ( Ff(x).*cos(delta(ii-1)) + Fr(x) - m.*vx_local_anal(ii-1).*w_IMU_anal(ii-1) ) ) ;
        %         w_IMU_anal(ii) = w_IMU_anal(ii-1) + h*( 1/Iz .* (  lf.*( Ff(x).*cos(delta(ii-1)) ) - Fr(x).*lr) ) ;

    end

    % predicted slip angle
    alpha_f(ii) = (delta(ii) - atan2((vy_local_anal(ii) + lf*w_IMU_anal(ii)), vx_local_anal(ii))) ;
    alpha_r(ii) = atan2( (-vy_local_anal(ii) + lr*w_IMU_anal(ii)), vx_local_anal(ii) ) ;

end

% Write down the cost function
obj = 1/max_vx*(vx_local_anal - vx_local)'*(vx_local_anal - vx_local) + 1/max_vy*(vy_local_anal - vy_local)'*(vy_local_anal - vy_local) + 1/max_w*(w_IMU_anal - w_IMU)'*(w_IMU_anal - w_IMU) ;

end





