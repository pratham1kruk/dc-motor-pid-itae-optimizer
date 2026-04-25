%% DC motor plant
R = 1; L = 0.5; J = 0.01; B = 0.1; Kt = 0.01; Ke = 0.01;
num = Kt;
den = [L*J, L*B + R*J, R*B + Ke*Kt];
G = tf(num, den);

%% Ziegler-Nichols baseline
Ku = 70; Pu = 0.8;
Kp_ZN = 0.6*Ku;
Ki_ZN = 2*Kp_ZN/Pu;
Kd_ZN = Kp_ZN*Pu/8;
C_ZN = pid(Kp_ZN, Ki_ZN, Kd_ZN);
CL_ZN = feedback(C_ZN*G, 1);

% baseline step info
info_ZN = stepinfo(CL_ZN);
fprintf('Z-N baseline: Tr=%.4f, Ts=%.4f, OS=%.2f%%\n', info_ZN.RiseTime, info_ZN.SettlingTime, info_ZN.Overshoot);

%% Optimization setup
Tfinal = 3.0;
t = linspace(0, Tfinal, 1000);

% Use fmincon for constraint handling
x0 = [Kp_ZN, Ki_ZN, Kd_ZN];    % initial (Z-N)
lb = [0.1, 0.1, 0.01];         % reasonable lower bounds
ub = [200, 300, 50];           % reasonable upper bounds

% Objective function
obj_fun = @(x) pid_itae_obj(x, G, t);

% Use fmincon with bounds
options = optimoptions('fmincon', 'Display', 'iter', 'MaxIterations', 100, ...
    'TolFun', 1e-6, 'TolX', 1e-6, 'Algorithm', 'interior-point');

% Optimize
[x_opt, fval] = fmincon(obj_fun, x0, [], [], [], [], lb, ub, [], options);

Kp_opt = x_opt(1); 
Ki_opt = x_opt(2); 
Kd_opt = x_opt(3);
C_opt = pid(Kp_opt, Ki_opt, Kd_opt);
CL_opt = feedback(C_opt*G, 1);

% compute step infos
info_opt = stepinfo(CL_opt);
fprintf('\nOptimized PID: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp_opt, Ki_opt, Kd_opt);
fprintf('Optimized: Tr=%.4f, Ts=%.4f, OS=%.2f%%\n', info_opt.RiseTime, info_opt.SettlingTime, info_opt.Overshoot);

%% Plot comparisons
figure;
subplot(2,1,1);
[y_z, tz] = step(CL_ZN, t);
[y_opt, to] = step(CL_opt, t);
plot(tz, y_z, 'b', 'LineWidth', 1.6); hold on;
plot(to, y_opt, 'r--', 'LineWidth', 1.6);
yline(1, 'k:');
legend('Z-N PID', 'Optimized PID', 'Setpoint');
xlabel('Time (s)'); 
ylabel('Output (speed)');
title('Closed-loop Step Response: Z-N vs Optimized PID');
grid on;

% Error plot
subplot(2,1,2);
e_z = 1 - y_z; 
e_opt = 1 - y_opt;
plot(tz, abs(e_z), 'b', 'LineWidth', 1.6); hold on;
plot(to, abs(e_opt), 'r--', 'LineWidth', 1.6);
legend('Z-N Error', 'Optimized Error');
xlabel('Time (s)'); 
ylabel('|Error|');
title('Absolute Error Comparison');
grid on;

% Calculate ITAE correctly
ITAE_ZN = trapz(tz, tz' .* abs(e_z));
ITAE_OPT = trapz(to, to' .* abs(e_opt));
fprintf('\nITAE ZN = %.6f, ITAE OPT = %.6f\n', ITAE_ZN, ITAE_OPT);
fprintf('ITAE Improvement: %.2f%%\n', (ITAE_ZN - ITAE_OPT)/ITAE_ZN * 100);

%% Optional: use pidtune for comparison
[C_pidtune, ~] = pidtune(G, 'PID');
CL_pidtune = feedback(C_pidtune*G, 1);
info_pidtune_step = stepinfo(CL_pidtune);
fprintf('\npidtune PID: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', C_pidtune.Kp, C_pidtune.Ki, C_pidtune.Kd);
fprintf('pidtune: Tr=%.4f, Ts=%.4f, OS=%.2f%%\n', info_pidtune_step.RiseTime, info_pidtune_step.SettlingTime, info_pidtune_step.Overshoot);

% Compare all three
figure;
step(CL_ZN, 'b', CL_opt, 'r--', CL_pidtune, 'g-.', t);
legend('Z-N PID', 'Optimized PID', 'pidtune PID');
title('Comparison of All Three Controllers');
grid on;

%% Objective function
function J = pid_itae_obj(x, G, t)
    % x = [Kp, Ki, Kd]
    Kp = x(1); Ki = x(2); Kd = x(3);
    
    % Check for invalid gains
    if any(isnan(x)) || any(isinf(x)) || any(x <= 0)
        J = 1e10;
        return;
    end
    
    try
        C = pid(Kp, Ki, Kd);
        sys = feedback(C*G, 1);
        
        % Check stability
        if ~isstable(sys)
            J = 1e10;
            return;
        end
        
        % Simulate step response
        y = step(sys, t);
        e = 1 - y;  % error
        
        % Calculate ITAE
        ITAE = trapz(t, t' .* abs(e));
        
        % Get step info for penalties
        info = stepinfo(sys);
        
        % Penalty for excessive overshoot (>15%)
        overshoot_pen = 0;
        if ~isempty(info.Overshoot) && ~isnan(info.Overshoot)
            if info.Overshoot > 15
                overshoot_pen = 100 * (info.Overshoot - 15)^2;
            end
        end
        
        % Penalty for long settling time (>2s)
        settling_pen = 0;
        if ~isempty(info.SettlingTime) && ~isnan(info.SettlingTime)
            if info.SettlingTime > 2.0
                settling_pen = 50 * (info.SettlingTime - 2.0)^2;
            end
        end
        
        % Small penalty on control effort (derivative of output)
        dy = [0; diff(y)];
        control_pen = 0.0001 * sum(dy.^2);
        
        % Total cost
        J = ITAE + overshoot_pen + settling_pen + control_pen;
        
    catch ME
        % Penalize configurations that cause errors
        J = 1e10;
        fprintf('Error in simulation: %s\n', ME.message);
    end
end