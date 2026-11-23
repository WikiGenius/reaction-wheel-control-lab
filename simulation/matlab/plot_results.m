function plot_results(S, P)
%PLOT_RESULTS Clean study plots for the reaction-wheel system.

t = S.t(:);
x = S.x;
sig = S.sig;

theta_b = x(:,1);
omega_b = x(:,2);
theta_w = x(:,3);
omega_w = x(:,4);
i       = x(:,5);

theta_rel = theta_w - theta_b;
omega_rel = omega_w - omega_b;

theta_ref = getsig(sig,'theta_ref',zeros(size(t)));
omega_ref = getsig(sig,'omega_ref',zeros(size(t)));

figure('Name','Angles');
subplot(3,1,1);
plot(t, theta_b, 'LineWidth', 1.2); hold on;
plot(t, theta_ref, '--', 'LineWidth', 1.0);
grid on; ylabel('\theta_b [rad]');
legend('\theta_b','\theta_{ref}','Location','best');
title('Base angle');

subplot(3,1,2);
plot(t, theta_w, 'LineWidth', 1.2);
grid on; ylabel('\theta_w [rad]');
title('Wheel absolute angle');

subplot(3,1,3);
plot(t, theta_rel, 'LineWidth', 1.2);
grid on; ylabel('\theta_{rel} [rad]'); xlabel('Time [s]');
title('Wheel relative angle');

figure('Name','Rates');
subplot(3,1,1);
plot(t, omega_b, 'LineWidth', 1.2);
grid on; ylabel('\omega_b [rad/s]');
title('Base rate');

subplot(3,1,2);
plot(t, omega_w, 'LineWidth', 1.2);
grid on; ylabel('\omega_w [rad/s]');
title('Wheel absolute rate');

subplot(3,1,3);
plot(t, omega_rel, 'LineWidth', 1.2); hold on;
plot(t, omega_ref, '--', 'LineWidth', 1.0);
grid on; ylabel('\omega_{rel} [rad/s]'); xlabel('Time [s]');
legend('\omega_{rel}','\omega_{ref}','Location','best');
title('Wheel relative rate');

figure('Name','Current and command');
subplot(2,1,1);
plot(t, i, 'LineWidth', 1.2); hold on;
plot(t, getsig(sig,'i_ref',zeros(size(t))), '--', 'LineWidth', 1.0);
grid on; ylabel('i [A]');
legend('i','i_{ref}','Location','best');
title('Motor current');

subplot(2,1,2);
plot(t, getsig(sig,'u_cmd_raw',zeros(size(t))), ':', 'LineWidth', 1.0); hold on;
plot(t, S.u, 'LineWidth', 1.2);
grid on; ylabel('u_{norm}'); xlabel('Time [s]');
legend('raw','safe/final','Location','best');
title('Normalized command');

figure('Name','Battery / torque');
subplot(3,1,1);
plot(t, getsig(sig,'Vbatt',zeros(size(t))), 'LineWidth', 1.2); hold on;
plot(t, getsig(sig,'Voc',zeros(size(t))), '--', 'LineWidth', 1.0);
grid on; ylabel('V [V]');
legend('V_{batt}','V_{oc}','Location','best');
title('Battery voltage');

subplot(3,1,2);
plot(t, getsig(sig,'Ibatt',zeros(size(t))), 'LineWidth', 1.2);
grid on; ylabel('I_{batt} [A]');
title('Battery current');

subplot(3,1,3);
plot(t, getsig(sig,'tau_w',zeros(size(t))), 'LineWidth', 1.2); hold on;
plot(t, getsig(sig,'tau_m',zeros(size(t))), '--', 'LineWidth', 1.0);
grid on; ylabel('\tau [N*m]'); xlabel('Time [s]');
legend('\tau_w','\tau_m','Location','best');
title('Wheel-side and motor-shaft torque');

figure('Name','Power');
plot(t, getsig(sig,'P_batt',zeros(size(t))), 'LineWidth', 1.2); hold on;
plot(t, getsig(sig,'P_elec',zeros(size(t))), '--', 'LineWidth', 1.0);
plot(t, getsig(sig,'P_mech_exchange',zeros(size(t))), ':', 'LineWidth', 1.1);
grid on; ylabel('Power [W]'); xlabel('Time [s]');
legend('P_{batt}','P_{elec}','P_{mech,exchange}','Location','best');
title('Power flow');

if isfield(sig,'safety_flag')
    figure('Name','Safety');
    subplot(3,1,1);
    stairs(t, safety_flag_to_level(sig.safety_flag), 'LineWidth', 1.5);
    grid on;
    yticks([0 1 2 3]);
    yticklabels({'GOOD','WARN','LIMIT','FAULT'});
    ylabel('Flag');
    title('Safety state');

    subplot(3,1,2);
    plot(t, 100*getsig(sig,'util_I_cont',zeros(size(t))), 'LineWidth', 1.1); hold on;
    plot(t, 100*getsig(sig,'util_omega',zeros(size(t))), '--', 'LineWidth', 1.0);
    grid on; ylabel('Util [%]');
    legend('I_{cont}','\omega_{rel}','Location','best');

    subplot(3,1,3);
    plot(t, getsig(sig,'u_derate',ones(size(t))), 'LineWidth', 1.1); hold on;
    stairs(t, double(getsig(sig,'fault_latched',false(size(t)))), '--', 'LineWidth', 1.0);
    grid on; ylabel('derate / latch'); xlabel('Time [s]');
    legend('u derate','fault latched','Location','best');
end

% Energy audit
try
    E = energy_audit(S, P);
    figure('Name','Energy residuals');
    subplot(3,1,1);
    plot(E.t, E.P_drv_loss, 'LineWidth', 1.1); grid on; ylabel('P_{drv} [W]');
    title('Driver / bus-side dissipation');
    yyaxis right;
    plot(E.t, E.P_gear_loss, '--', 'LineWidth', 1.0);
    ylabel('P_{gear} [W]');
    yyaxis left;
    subplot(3,1,2);
    plot(E.t, E.res_elec, 'LineWidth', 1.1); hold on;
    plot(E.t, E.res_mech, '--', 'LineWidth', 1.0);
    grid on; ylabel('res [W]'); legend('res_{elec}','res_{mech}','Location','best');
    subplot(3,1,3);
    plot(E.t, E.res_total, 'LineWidth', 1.1); hold on;
    plot(E.t, E.E_drift_total, '--', 'LineWidth', 1.0);
    grid on; ylabel('res / drift'); legend('res_{total}','E drift total','Location','best'); xlabel('Time [s]');
catch ME
    warning('plot_results:energyAuditFailed','Energy audit failed: %s', ME.message);
end

end

function y = getsig(sig, field, defaultVal)
if isstruct(sig) && isfield(sig, field) && ~isempty(sig.(field))
    tmp = sig.(field);
    n = numel(defaultVal);
    if isscalar(tmp), y = repmat(tmp, n, 1); return; end
    if numel(tmp)==n, y = tmp(:); return; end
    y = defaultVal;
else
    y = defaultVal;
end
end

function y = safety_flag_to_level(flag)
flag = string(flag);
y = zeros(size(flag));
y(flag=="GOOD")  = 0;
y(flag=="WARN")  = 1;
y(flag=="LIMIT") = 2;
y(flag=="FAULT") = 3;
end
