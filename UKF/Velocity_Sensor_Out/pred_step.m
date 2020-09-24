function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time 
    % PART 1
    k = 2;
    alpha = 0.001;
    beta = 2;
    n = 27;
    lambda = alpha^2*(n+k)-n; 
    Q_t = 0.02*eye(12);
    uPrev_aug = [uPrev;zeros(12,1)];
    P_aug = [covarPrev,zeros(15,12);zeros(12,15),Q_t];

    X = zeros(27,2*n+1);
    P_aug_sq = chol(P_aug)';
    X(:,1) = uPrev_aug;
    for i= 1:n
        X(:,2*i) = uPrev_aug + sqrt(n+lambda) * P_aug_sq(:,i);
        X(:,2*i+1) = uPrev_aug - sqrt(n+lambda) * P_aug_sq(:,i);
    end
        
    X_dot = zeros(15,2*n+1);
    for i = 1:2*n + 1
        X_dot(:,i) = f(X(1:15,i),angVel,acc,X(16:27,i));
    end
    
    X_t = zeros(15,2*n+1);    
    for i = 1:2*n + 1
        X_t(:,i) = X(1:15,i) + X_dot(:,i) * dt;
    end
    W_0_m = lambda / (n + lambda);
    uEst = W_0_m * X_t(:,1);
    W_i_m = 1/(2*(n+lambda));
    for i = 2: 2*n+1
        uEst = uEst + W_i_m * X_t(:,i);
    end
    
    W_0_c = (lambda / (lambda + n)) + (1 - alpha^2 + beta);
    W_i_c = 1 / (2 * (n + lambda));
    covarEst = W_0_c * (X_t(:,1) - uEst) * (X_t(:,1) - uEst)';
    for i = 2: 2*n+1
        covarEst = covarEst + W_i_c * (X_t(:,i) - uEst) * (X_t(:,i) - uEst)';
    end
    
end

function X_t = f(X_i_x,angVel,acc,n)
    x = X_i_x(4);
    y = X_i_x(5);
    z = X_i_x(6);
    x2 = X_i_x(4:6);

    G = [1, sin(x2(2)), -1 * sin(x2(2));
        0, cos(x2(1)), cos(x2(2)) * sin(x2(1));
        0, -1 * sin(x2(1)), cos(x2(2))*cos(x2(1))];

    R = eul2rotm([z,y,x]);
    
    X_t = [X_i_x(7:9);
        G\(angVel - X_i_x(10:12) - n(1:3));
        [0;0;-9.8] + R * (acc - X_i_x(13:15)) - n(4:6);
        n(7:9);
        n(10:12);
        ];
end

