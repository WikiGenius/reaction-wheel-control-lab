function [tau_ext_b, tau_ext_w, dbg] = disturbance_torque(t, x, P)
%DISTURBANCE_TORQUE Centralized disturbance/load torques.
%
% State:
%   x = [theta_b; omega_b; theta_w; omega_w; i]
%
% Relative wheel coordinates used for wheel-dependent ripple:
%   theta_rel = theta_w - theta_b
%   omega_rel = omega_w - omega_b

theta_b = safe_x(x,1,0);
omega_b = safe_x(x,2,0);
theta_w = safe_x(x,3,0);
omega_w = safe_x(x,4,0);

theta_rel = theta_w - theta_b;
omega_rel = omega_w - omega_b;

tau_ext_b = 0;
tau_ext_w = 0;

dbg = struct();

% Base components
dbg.tau_const_b   = 0;
dbg.tau_step_b    = 0;
dbg.tau_sin_b     = 0;
dbg.tau_wire_b    = 0;
dbg.tau_vib_b     = 0;
dbg.tau_imb_b     = 0;
dbg.tau_preload_b = 0;

% Wheel components
dbg.tau_const_w = 0;
dbg.tau_step_w  = 0;
dbg.tau_sin_w   = 0;

dbg.tau_ripple_b    = 0;
dbg.tau_ext_b_total = 0;
dbg.tau_ext_w_total = 0;

if ~isfield(P,'dist') || ~isstruct(P.dist)
    return;
end

D = P.dist;

% Constant torques
if isfield(D,'tau_ext_b') && getfield_def(D.tau_ext_b,'enable',false)
    dbg.tau_const_b = getfield_def(D.tau_ext_b,'value',0.0);
    tau_ext_b = tau_ext_b + dbg.tau_const_b;
end
if isfield(D,'tau_ext_w') && getfield_def(D.tau_ext_w,'enable',false)
    dbg.tau_const_w = getfield_def(D.tau_ext_w,'value',0.0);
    tau_ext_w = tau_ext_w + dbg.tau_const_w;
end

% Step torques
if isfield(D,'tau_step_b') && getfield_def(D.tau_step_b,'enable',false) && t >= getfield_def(D.tau_step_b,'t_on',0)
    dbg.tau_step_b = getfield_def(D.tau_step_b,'value',0.0);
    tau_ext_b = tau_ext_b + dbg.tau_step_b;
end
if isfield(D,'tau_step_w') && getfield_def(D.tau_step_w,'enable',false) && t >= getfield_def(D.tau_step_w,'t_on',0)
    dbg.tau_step_w = getfield_def(D.tau_step_w,'value',0.0);
    tau_ext_w = tau_ext_w + dbg.tau_step_w;
end

% Sine torques
if isfield(D,'tau_sin_b') && getfield_def(D.tau_sin_b,'enable',false)
    A  = getfield_def(D.tau_sin_b,'amp',0.0);
    f  = getfield_def(D.tau_sin_b,'freqHz',0.0);
    ph = getfield_def(D.tau_sin_b,'phase0',0.0);
    dbg.tau_sin_b = A*sin(2*pi*f*t + ph);
    tau_ext_b = tau_ext_b + dbg.tau_sin_b;
end
if isfield(D,'tau_sin_w') && getfield_def(D.tau_sin_w,'enable',false)
    A  = getfield_def(D.tau_sin_w,'amp',0.0);
    f  = getfield_def(D.tau_sin_w,'freqHz',0.0);
    ph = getfield_def(D.tau_sin_w,'phase0',0.0);
    dbg.tau_sin_w = A*sin(2*pi*f*t + ph);
    tau_ext_w = tau_ext_w + dbg.tau_sin_w;
end

% Wire torsion on base
if isfield(D,'wire') && getfield_def(D.wire,'enable',false)
    K  = getfield_def(D.wire,'K',0.0);
    Dd = getfield_def(D.wire,'D',0.0);
    dbg.tau_wire_b = -K*theta_b - Dd*omega_b;
    tau_ext_b = tau_ext_b + dbg.tau_wire_b;
end

% Vibration / imbalance
if isfield(D,'vib') && isstruct(D.vib)
    vib = D.vib;

    if getfield_def(vib,'enable',false)
        A   = getfield_def(vib,'tau_b_amp',0.0);
        fHz = getfield_def(vib,'freqHz',0.0);
        ph  = getfield_def(vib,'phase0',0.0);
        dbg.tau_vib_b = A * sin(2*pi*fHz*t + ph);
        tau_ext_b = tau_ext_b + dbg.tau_vib_b;
    end

    if getfield_def(vib,'imbalance_enable',false)
        k  = getfield_def(vib,'k_imb',0.0);
        ph = getfield_def(vib,'phase0',0.0);
        dbg.tau_imb_b = k * (omega_rel^2) * sin(theta_rel + ph);
        tau_ext_b = tau_ext_b + dbg.tau_imb_b;
    end
end

% Preload / ripple
if isfield(D,'preload') && isstruct(D.preload) && getfield_def(D.preload,'enable',false)
    pr = D.preload;
    harm = getfield_def(pr,'ripple_harm',1);
    ph   = getfield_def(pr,'phase0',0.0);

    A0  = getfield_def(pr,'tau_ripple0',0.0);
    kw2 = getfield_def(pr,'k_ripple_w2',0.0);
    A   = A0 + kw2*(omega_rel^2);

    dbg.tau_preload_b = A * sin(harm*theta_rel + ph);
    tau_ext_b = tau_ext_b + dbg.tau_preload_b;
end

dbg.tau_ripple_b    = dbg.tau_vib_b + dbg.tau_imb_b + dbg.tau_preload_b;
dbg.tau_ext_b_total = tau_ext_b;
dbg.tau_ext_w_total = tau_ext_w;

end

function v = getfield_def(S, field, defaultVal)
v = defaultVal;
if isstruct(S) && isfield(S, field)
    tmp = S.(field);
    if ~isempty(tmp), v = tmp; end
end
end

function xi = safe_x(x, idx, defaultVal)
xi = defaultVal;
if isnumeric(x) && numel(x) >= idx
    xi = x(idx);
end
end
