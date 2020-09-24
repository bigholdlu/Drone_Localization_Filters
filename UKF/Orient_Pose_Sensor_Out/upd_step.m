function result = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    %PART1
    C = [eye(6),zeros(6,9)];
    R = 0.01 * eye(6);
    K = covarEst * C' / (C * covarEst * C'+ R);
    uCurr = uEst + K * (z_t - C * uEst);
    covar_curr = covarEst - K * C * covarEst;
    result = [uCurr,covar_curr];
end

