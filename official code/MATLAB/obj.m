function obj = obj(x, tau_mat, delta_mat, vx_mat, vy_mat, w_mat, N_pred, N, max_vx, max_vy, max_w, alpha_f_mat, alpha_r_mat, a_mot, b_mot, c_mot) 

h = 0.1 ;
m = 1.63 ;
L = 0.175 ;
lr = 0.45*L ; %[m]
lf = L - lr ; %[m]
Iz = 0.0061  ; %[Kg*m^2]

% initialize to zero the velocity matrices
vx_fit = zeros(N_pred,N-N_pred) ;
vy_fit = zeros(N_pred,N-N_pred) ;
w_fit= zeros(N_pred,N-N_pred) ;

% Start the forward integration:
for ii=1:(N-N_pred)
   % write the first row (equal to data)
    vx_fit(1,ii) = vx_mat(1,ii) ;
    vy_fit(1,ii) = vy_mat(1,ii) ;
    w_fit(1,ii) = w_mat(1,ii) ;
    alpha_f_fit = alpha_f_mat(1,ii) ;
    alpha_r_fit = alpha_r_mat(1,ii) ;

    for jj=2:N_pred
        %%% LINEAR TYRE MODEL --> only 4 parameters to be identified instead of 8
        %         Fx = @(x) ( ( x(3) .* tau_mat(jj-1) - x(4) .* vx_fit(jj-1,ii) ) - 0.1.*x(3) ) .* m ;  % Fx = a*tau - b*vx - c
        %         Ff = @(x)  x(1).*alpha_f_fit ; % Ff = Cf*alpha_f
        %         Fr = @(x)  x(2).*alpha_r_fit ; % Fr = Cr*alpha_r

        %%% NON LINEAR TYRE MODEL
        Fx = @(x) ( ( x(7) - x(8) .* vx_fit(jj-1,ii) ).* tau_mat(jj-1) - 0.1.*x(7) ) .* m ;  % Fx = (a - b*vx)*tau - c
        Ff = @(x) x(1) .* sin( x(3) .* atan2( x(5).*alpha_f_fit, 1 ) ) ;
        Fr = @(x) x(2) .* sin( x(4) .* atan2( x(6).*alpha_r_fit, 1 ) ) ;

        %%% MODEL WITH REAR DRIVEN WHEELs
        vx_fit(jj,ii) = vx_fit(jj-1,ii) + h*( 1/m .* ( Fx(x)/2 + Fx(x)/2.*cos(delta_mat(jj-1,ii)) - Ff(x).*sin(delta_mat(jj-1,ii)) + m.*vy_fit(jj-1,ii).*w_fit(jj-1,ii) ) ) ;
        vy_fit(jj,ii) = vy_fit(jj-1,ii) + h*( 1/m .* ( Fx(x)/2.*sin(delta_mat(jj-1,ii)) + Ff(x).*cos(delta_mat(jj-1,ii)) + Fr(x) - m.*vx_fit(jj-1,ii).*w_fit(jj-1,ii) ) ) ;
        w_fit(jj,ii) = w_fit(jj-1,ii) + h*( 1/Iz .* (  lf.*( Ff(x).*cos(delta_mat(jj-1,ii)) + Fx(x)/2.*sin(delta_mat(jj-1,ii)) ) - Fr(x).*lr) ) ;
        
        % MODEL WITH 4 DRIVEN WHEELS
        %         vx_fit(jj,ii) = vx_fit(jj-1,ii) + h*( 1/m .* ( Fx(x) - Ff(x).*sin(delta_mat(jj-1,ii)) + m.*vy_fit(jj-1,ii).*w_fit(jj-1,ii) ) ) ;
        %         vy_fit(jj,ii) = vy_fit(jj-1,ii) + h*( 1/m .* ( Ff(x).*cos(delta_mat(jj-1,ii)) + Fr(x) - m.*vx_fit(jj-1,ii).*w_fit(jj-1,ii) ) ) ;
        %         w_fit(jj,ii) = w_fit(jj-1,ii) + h*( 1/Iz .* (  lf.*( Ff(x).*cos(delta_mat(jj-1,ii)) ) - Fr(x).*lr) ) ;

        alpha_f_fit = ( delta_mat(jj,ii) - atan2( ( vy_fit(jj,ii) + lf*w_fit(jj,ii) ), vx_fit(jj,ii) ) ) ;
        alpha_r_fit = atan2( ( -vy_fit(jj,ii) + lr*w_fit(jj,ii) ), vx_fit(jj,ii) ) ;

    end

end

array_obj = zeros(N-N_pred,1) ;

% weight each column of the matrices
for kk=1:N-N_pred

    array_obj(kk) = 5*1/max_vx*(vx_fit(:,kk) - vx_mat(:,kk))'*(vx_fit(:,kk) - vx_mat(:,kk)) + 1/max_vy*(vy_fit(:,kk) - vy_mat(:,kk))'*(vy_fit(:,kk) - vy_mat(:,kk)) + 1/max_w*(w_fit(:,kk) - w_mat(:,kk))'*(w_fit(:,kk) - w_mat(:,kk)) ;

end

% sum the cost of each column
obj = sum(array_obj) ;

end





