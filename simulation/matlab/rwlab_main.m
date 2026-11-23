function S = rwlab_main(Pin)
%RWLAB_MAIN Final simulation entry point for the reaction-wheel lab model.
%
% State:
%   x = [theta_b; omega_b; theta_w; omega_w; i]
%
% Angles/rates:
%   theta_b, omega_b : base absolute angle/rate
%   theta_w, omega_w : wheel absolute angle/rate
%   theta_rel        : theta_w - theta_b
%   omega_rel        : omega_w - omega_b
%
% Core principle:
%   tau_reaction = I_w * domega_w/dt
%
% Internal actuator torque acts equal/opposite between base and wheel.
%
% This file is intentionally thin. All physics and configuration live in:
%   params_default.m
%   rw_controller.m
%   rw_dynamics.m
%   battery_step.m
%   simulate_case.m

if nargin < 1 || isempty(Pin)
    P = params_default();
else
    P = params_default(Pin);
end

validate_params_local(P);

S = simulate_case(P);
if logical(safe_get_local(P, {'sim','doPlot'}, true))
    plot_results(S, P);
end

end

function validate_params_local(P)
assert(isfield(P,'sim')   && isfield(P.sim,'dt')    && P.sim.dt > 0, 'P.sim.dt must be > 0.');
assert(isfield(P,'sim')   && isfield(P.sim,'T_end') && P.sim.T_end > 0, 'P.sim.T_end must be > 0.');
assert(isfield(P,'sim')   && isfield(P.sim,'x0')    && numel(P.sim.x0)==5, 'P.sim.x0 must be 5x1.');
assert(isfield(P,'base')  && isfield(P.base,'Jb')   && P.base.Jb > 0, 'P.base.Jb missing/invalid.');
assert(isfield(P,'wheel') && isfield(P.wheel,'Jw')  && P.wheel.Jw > 0, 'P.wheel.Jw missing/invalid.');
assert(isfield(P,'motor') && isfield(P.motor,'R')   && P.motor.R > 0, 'P.motor.R missing/invalid.');
assert(isfield(P,'motor') && isfield(P.motor,'Kt')  && P.motor.Kt > 0, 'P.motor.Kt missing/invalid.');
assert(isfield(P,'motor') && isfield(P.motor,'Ke')  && P.motor.Ke > 0, 'P.motor.Ke missing/invalid.');
assert(isfield(P,'gear')  && isfield(P.gear,'N')    && P.gear.N > 0, 'P.gear.N missing/invalid.');
assert(isfield(P,'batt')  && isfield(P.batt,'V_nom')&& P.batt.V_nom > 0, 'P.batt.V_nom missing/invalid.');
assert(isfield(P,'driver')&& isfield(P.driver,'V_max_cmd') && P.driver.V_max_cmd > 0, 'P.driver.V_max_cmd missing/invalid.');

Jsum = P.base.Jb + P.wheel.Jw;
Jtot = P.stack.J_total;
assert(abs(Jsum - Jtot) <= max(1e-12, 1e-6*max(Jtot,1)), ...
    'Inertia split inconsistent: P.base.Jb + P.wheel.Jw must equal P.stack.J_total.');

if isfield(P.motor,'Jeq_out')
    assert(P.motor.Jeq_out >= 0, 'P.motor.Jeq_out must be >= 0.');
end
end

function v = safe_get_local(S, path, defaultVal)
v = defaultVal;
try
    tmp = S;
    for i = 1:numel(path)
        f = path{i};
        if isstruct(tmp) && isfield(tmp, f)
            tmp = tmp.(f);
        else
            v = defaultVal; return;
        end
    end
    if isempty(tmp), v = defaultVal; else, v = tmp; end
catch
    v = defaultVal;
end
end
