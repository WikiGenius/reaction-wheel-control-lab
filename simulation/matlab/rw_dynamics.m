function xdot_mech = rw_dynamics(t, x, P, u) %#ok<INUSD>
%RW_DYNAMICS Mechanical dynamics only.
%
% State:
%   x = [theta_b; omega_b; theta_w; omega_w; i]
%
% Coordinates:
%   theta_b, theta_w are absolute angles.
%   Relative wheel motion:
%       theta_rel = theta_w - theta_b
%       omega_rel = omega_w - omega_b
%
% Internal actuator torque:
%   tau_w acts + on wheel and - on base.
%
% Internal joint dissipation:
%   tau_joint opposes omega_rel and acts equal/opposite on wheel/base.
%
% External support friction:
%   acts only on the base against the environment.
%
% Optional relative actuator inertia:
%   Jeq_out is output-side equivalent inertia tied to the RELATIVE motion.
%   Kinetic energy term:
%       0.5*Jeq_out*(omega_w - omega_b)^2
%   This yields the coupled 2x2 inertia matrix solved below.

theta_b = x(1);
omega_b = x(2);
theta_w = x(3);
omega_w = x(4);
i       = x(5);

Jb   = safe_get(P, {'base','Jb'}, 1e-3);
Jw   = safe_get(P, {'wheel','Jw'}, 1e-3);
Jeq  = safe_get(P, {'motor','Jeq_out'}, 0.0);
N    = safe_get(P, {'gear','N'}, 1.0);
eta  = safe_get(P, {'gear','eta'}, 1.0);
Kt   = safe_get(P, {'motor','Kt'}, 0.0);

tau_m = Kt * i;
tau_w = eta * N * tau_m;

theta_rel = theta_w - theta_b;
omega_rel = omega_w - omega_b;
epsw = max(safe_get(P, {'fric','omega_eps'}, 8e-3), 1e-9);

% Base external/support dissipation
Bb   = safe_get(P, {'base','Bb'}, 0.0);
Tc_b = safe_get(P, {'base','Tc_b'}, 0.0);
tau_base_env = Bb*omega_b + Tc_b*tanh(omega_b/epsw);

% Internal relative dissipation
Brel   = safe_get(P, {'wheel','Brel'}, safe_get(P, {'wheel','Bw'}, 0.0));
Tc_rel = safe_get(P, {'wheel','Tc_rel'}, safe_get(P, {'wheel','Tc_w'}, 0.0));
tau_joint = Brel*omega_rel + Tc_rel*tanh(omega_rel/epsw);

% Optional external brakes
if isfield(P,'dist') && isfield(P.dist,'base_brake') && getfield_def(P.dist.base_brake,'enable',false)
    B = getfield_def(P.dist.base_brake,'Bload',0.0);
    T = getfield_def(P.dist.base_brake,'Tload',0.0);
    tau_base_env = tau_base_env + B*omega_b + T*tanh(omega_b/epsw);
end
if isfield(P,'dist') && isfield(P.dist,'wheel_brake') && getfield_def(P.dist.wheel_brake,'enable',false)
    B = getfield_def(P.dist.wheel_brake,'Bload',0.0);
    T = getfield_def(P.dist.wheel_brake,'Tload',0.0);
    tau_joint = tau_joint + B*omega_rel + T*tanh(omega_rel/epsw);
end

% External disturbances / loads
[tau_ext_b, tau_ext_w] = disturbance_torque(t, x, P);

% Optional preload Coulomb boost on base support friction
if isfield(P,'dist') && isfield(P.dist,'preload') && getfield_def(P.dist.preload,'enable',false)
    pr = P.dist.preload;
    Tc_boost    = getfield_def(pr,'Tc_boost',1.0);
    omega_stick = max(getfield_def(pr,'omega_stick',1.0), 1e-9);
    boost = 1 + (Tc_boost - 1) * exp(-(abs(omega_b)/omega_stick)^2);
    tau_base_env = Bb*omega_b + (Tc_b*boost) * tanh(omega_b/epsw);
end

% Base stiction against support/environment
base_stuck = false;
if isfield(P,'base') && isfield(P.base,'stiction') && getfield_def(P.base.stiction,'enable',false)
    omega_lock = getfield_def(P.base.stiction,'omega_lock',0.0);
    Ts_b       = getfield_def(P.base.stiction,'Ts_b',0.0);
    if abs(omega_b) < omega_lock
        tau_base_without_env_fric = (-tau_w) + tau_joint + tau_ext_b;
        if abs(tau_base_without_env_fric) < Ts_b
            base_stuck = true;
        end
    end
end

theta_b_dot = omega_b;
theta_w_dot = omega_w;

if base_stuck
    theta_b_dot = 0.0;
    omega_b_dot = 0.0;
    % Wheel still moves relative to stuck base
    omega_w_dot = ((+tau_w) - tau_joint + tau_ext_w) / max(Jw + Jeq, 1e-12);
else
    % Coupled inertia matrix from:
    % T = 0.5*Jb*wb^2 + 0.5*Jw*ww^2 + 0.5*Jeq*(ww-wb)^2
    %
    % [Jb+Jeq  -Jeq ] [ab] = [ -tau_w + tau_joint + tau_ext_b - tau_base_env ]
    % [ -Jeq  Jw+Jeq] [aw]   [ +tau_w - tau_joint + tau_ext_w             ]
    A11 = Jb + Jeq;
    A12 = -Jeq;
    A21 = -Jeq;
    A22 = Jw + Jeq;

    rhs_b = (-tau_w) + tau_joint + tau_ext_b - tau_base_env;
    rhs_w = (+tau_w) - tau_joint + tau_ext_w;

    detA = A11*A22 - A12*A21;
    if abs(detA) < 1e-15
        error('rw_dynamics: inertia matrix is singular.');
    end

    omega_b_dot = ( rhs_b*A22 - A12*rhs_w ) / detA;
    omega_w_dot = ( A11*rhs_w - rhs_b*A21 ) / detA;
end

xdot_mech = [theta_b_dot; omega_b_dot; theta_w_dot; omega_w_dot];

end

function v = safe_get(S, path, defaultVal)
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

function v = getfield_def(S, field, defaultVal)
v = defaultVal;
if isstruct(S) && isfield(S, field)
    tmp = S.(field);
    if ~isempty(tmp), v = tmp; end
end
end
