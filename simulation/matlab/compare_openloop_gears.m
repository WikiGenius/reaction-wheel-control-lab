function results = compare_openloop_gears()
%COMPARE_OPENLOOP_GEARS Compare the same open-loop pulse for 6, 10, 19.
%
% This uses one common test:
%   caseName  = "open_loop_stepV"
%   u_step    = 0.20 (normalized)
%   pulse on  = 0.20 s
%   pulse off = 0.80 s
%
% It prints a summary table and overlays the key responses.

gearList = [6 10 19];
results = repmat(struct(), numel(gearList), 1);
colors = lines(numel(gearList));

fig1 = figure('Name','Open-loop comparison: angle / rate');
ax11 = subplot(2,1,1); hold(ax11,'on'); grid(ax11,'on');
ax12 = subplot(2,1,2); hold(ax12,'on'); grid(ax12,'on');

fig2 = figure('Name','Open-loop comparison: current / command');
ax21 = subplot(2,1,1); hold(ax21,'on'); grid(ax21,'on');
ax22 = subplot(2,1,2); hold(ax22,'on'); grid(ax22,'on');

for k = 1:numel(gearList)
    P = params_default();
    P = set_motor_variant(P, gearList(k));

    P.sim.caseName = "open_loop_stepV";
    P.sim.doPlot = false;
    P.sim.doAuditPrint = false;

    P.ctrl.t_step = 0.0;
    P.ctrl.openloop.u_step = 0.20;
    P.ctrl.openloop.enable_pulse = true;
    P.ctrl.openloop.t_on  = 0.20;
    P.ctrl.openloop.t_off = 0.80;

    S = simulate_case(P);
    E = energy_audit(S,P);

    t = S.t(:);
    x = S.x;
    theta_b = x(:,1);
    omega_rel = x(:,4) - x(:,2);
    i_m = x(:,5);

    peakBaseDeg = rad2deg(max(abs(theta_b)));
    finalBaseDeg = rad2deg(theta_b(end));
    maxOmegaRel_rpm = max(abs(omega_rel)) * 60/(2*pi);
    maxI = max(abs(i_m));
    fault = false;
    if isfield(S.sig,'fault_latched')
        fault = any(logical(S.sig.fault_latched));
    end

    results(k).gearChoice = gearList(k); %#ok<*AGROW>
    results(k).label = string(P.gear.label);
    results(k).ratio_exact = P.gear.N;
    results(k).rpm_noload = P.gear.outputSpeedNoLoad_rpm;
    results(k).tau_stall_out_Nm = P.gear.tau_stall_out;
    results(k).torque_per_amp_out_Nm_A = P.gear.torque_per_amp_out;
    results(k).peak_base_deg = peakBaseDeg;
    results(k).final_base_deg = finalBaseDeg;
    results(k).max_omega_rel_rpm = maxOmegaRel_rpm;
    results(k).max_current_A = maxI;
    results(k).fault = fault;
    results(k).end_drift_total_J = E.E_drift_total(end);

    plot(ax11, t, rad2deg(theta_b), 'LineWidth', 1.3, 'Color', colors(k,:));
    plot(ax12, t, omega_rel * 60/(2*pi), 'LineWidth', 1.3, 'Color', colors(k,:));
    plot(ax21, t, i_m, 'LineWidth', 1.2, 'Color', colors(k,:));
    plot(ax22, t, S.u, 'LineWidth', 1.2, 'Color', colors(k,:));
end

title(ax11, 'Base angle comparison'); ylabel(ax11, '\theta_b [deg]');
legend(ax11, string({results.label}), 'Location','best');

title(ax12, 'Relative wheel speed comparison'); ylabel(ax12, '\omega_{rel} [rpm]'); xlabel(ax12, 'Time [s]');
legend(ax12, string({results.label}), 'Location','best');

title(ax21, 'Motor current comparison'); ylabel(ax21, 'i [A]');
legend(ax21, string({results.label}), 'Location','best');

title(ax22, 'Normalized command'); ylabel(ax22, 'u_{norm}'); xlabel(ax22, 'Time [s]');
legend(ax22, string({results.label}), 'Location','best');

T = struct2table(results);
disp('=== Open-loop gear comparison ===');
disp(T(:, {'label','ratio_exact','rpm_noload','tau_stall_out_Nm','torque_per_amp_out_Nm_A', ...
           'peak_base_deg','final_base_deg','max_omega_rel_rpm','max_current_A','fault','end_drift_total_J'}));

assignin('base','gear_compare_results',results);
assignin('base','gear_compare_table',T);
end
