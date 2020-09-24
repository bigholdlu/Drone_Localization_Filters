function result = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step camvel & linvel
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    %PART2
    v_from_proj2 = z_t(1:3);
    omega_from_proj2 = z_t(4:6);
    k = 2;
    alpha = 0.001;
    beta = 2;
    n = 15;
    lambda = (alpha^2)*(n+k)-n;
    R_t =0.6* eye(3);

    squared_covarEst = chol(covarEst)';     %chol decomposition
    
    %Find Sigma Point
    X_t = zeros(15,2*n+1);
    X_t(:,1) = uEst;
    for i = 1:n
        X_t(:,2*i) = uEst + sqrt(n + lambda) * squared_covarEst(:,i);
        X_t(:,2*i+1) = uEst - sqrt(n + lambda) * squared_covarEst(:,i);
    end


    %Propagate Sigma Point
    Z_t = zeros(3,2*n+1);
    for i = 1:2*n+1
        Z_t(:,i) = g(X_t(4:6,i),X_t(7:9,i),omega_from_proj2);
    end
    
    %Find predicted Mean z_miu_t
    W_0_m = lambda / (n + lambda);
    W_i_m = 1/(2*(n+lambda));
    
    z_ut = W_0_m * Z_t(:,1);
    for i = 2: 2*n+1
        z_ut = z_ut + W_i_m * Z_t(:,i);
    end
    
    %Find C_t
    W_0_c = (lambda / (lambda + n)) + (1 - alpha^2 + beta);
    W_i_c = 1 / (2 * (n + lambda));
    
    C_t = W_0_c * (X_t(:,1) - uEst) * (Z_t(:,1) - z_ut)';
    for i = 2: 2*n+1
        C_t = C_t + W_i_c * (X_t(:,i) - uEst) * (Z_t(:,i) - z_ut)';
    end
    
    %Find S_t
    S_t = W_0_c * (Z_t(:,1) - z_ut) * (Z_t(:,1) - z_ut)';
    for i = 2: 2*n+1
        S_t = S_t + W_i_c * (Z_t(:,i) - z_ut) * (Z_t(:,i) - z_ut)';
    end
    S_t = S_t + R_t;
    
    %Filter Gain and Update
    K = C_t * inv(S_t);
    uCurr = uEst + K * (v_from_proj2 - z_ut);
    covar_curr = covarEst - K * S_t * K';
    result = [uCurr,covar_curr];
end

function result = g(ori,pdot_bww,omega_cwc)
    R_b2w = eul2rotm([ori(3), ori(2), ori(1)], 'ZYX');

     R_c2b=[1/sqrt(2),-1/sqrt(2),0;
            -1/sqrt(2),-1/sqrt(2),0;
            0,0,-1]; 
    
    R_b2c = R_c2b';
    R_w2b = R_b2w';
    
    r=[0.04*1/sqrt(2);-0.04*1/sqrt(2);-0.03];  
   
    S_r_c2b_b = [   0, -r(3),  r(2);
                 r(3),     0, -r(1);
                -r(2),  r(1),     0
                ];
    S_r_b2c_b = S_r_c2b_b';
    omega_bwb = R_c2b * omega_cwc;
    v_cwc = R_b2c * R_w2b * pdot_bww  -  R_b2c * S_r_b2c_b * omega_bwb;
    result = v_cwc;
end

