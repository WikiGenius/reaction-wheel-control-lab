function cases = run_validation_cases()
%RUN_VALIDATION_CASES Practical sanity tests for the final 19:1 study model.
%
% The goal is not perfect certification. The goal is to catch the most
% important semantic/logical failures:
%   - immediate false faults
%   - runaway current
%   - wheel-speed limit violations
%   - large energy-drift inconsistencies

fprintf('\n=== Running validation cases ===\n');

cases = {};
k = 0;

%% 1) Open-loop pulse
k=k+1;
P = params_default();
P.sim.caseName = "open_loop_stepV";
P.ctrl.t_step = 0.0;
P.ctrl.openloop.u_step = 0.20;
P.ctrl.openloop.enable_pulse = true;
P.ctrl.openloop.t_on = 0.20;
P.ctrl.openloop.t_off = 0.80;
cases{k} = run_one("open_loop_pulse", P);

%% 2) Wheel-speed PI
k=k+1;
P = params_default();
P.sim.caseName = "wheel_speed_PI";
P.ctrl.t_step = 0.0;
P.ctrl.wheel.omega_ref = 20.0;
P.ctrl.wheel.Kp = 0.01;
P.ctrl.wheel.Ki = 0.2;
cases{k} = run_one("wheel_speed_PI", P);

%% 3) Base-angle PD
k=k+1;
P = params_default();
P.sim.caseName = "base_angle_PD";
P.ctrl.theta_ref = deg2rad(30);
cases{k} = run_one("base_angle_PD", P);

fprintf('\n=== Validation summary ===\n');
all_pass = true;
for j = 1:numel(cases)
    C = cases{j};
    fprintf('%-18s pass=%d  fault=%d  max|i|=%.3f A  max|omega_rel|=%.3f rad/s  drift=%.4g J\n', ...
        C.name, C.pass, C.fault, C.max_i, C.max_omega_rel, C.end_drift_total);
    all_pass = all_pass && C.pass;
end

if ~all_pass
    warning('run_validation_cases:SomeChecksFailed','One or more validation cases failed. Inspect the returned structures.');
end

assignin('base','validation_cases',cases);
end

function C = run_one(name, P)
P.sim.doPlot = false;
P.sim.doAuditPrint = false;
S = rwlab_main(P);
E = energy_audit(S,P);

omega_rel = S.x(:,4) - S.x(:,2);
fault = false;
if isfield(S.sig,'fault_latched')
    fault = any(logical(S.sig.fault_latched));
end

max_i = max(abs(S.x(:,5)));
max_omega_rel = max(abs(omega_rel));
end_drift_total = E.E_drift_total(end);
end_drift_mech  = E.E_drift_mech(end);
end_drift_elec  = E.E_drift_elec(end);

% Practical pass criteria.
pass = true;
pass = pass && ~fault;
pass = pass && isfinite(max_i) && max_i <= 3.5;
pass = pass && isfinite(max_omega_rel) && max_omega_rel <= 1.05*P.gear.outputSpeedNoLoad_rad_s;
pass = pass && isfinite(end_drift_total) && abs(end_drift_total) <= 0.10;
pass = pass && isfinite(end_drift_mech)  && abs(end_drift_mech)  <= 0.02;
pass = pass && isfinite(end_drift_elec)  && abs(end_drift_elec)  <= 0.02;

C = struct('name',name,'pass',pass,'fault',fault,'max_i',max_i,...
    'max_omega_rel',max_omega_rel,'end_drift_total',end_drift_total,...
    'end_drift_mech',end_drift_mech,'end_drift_elec',end_drift_elec);
end
