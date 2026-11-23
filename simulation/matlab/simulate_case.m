function S = simulate_case(P)
%SIMULATE_CASE Fixed-step simulation.
%
% Per sample:
%   1) controller -> raw normalized command
%   2) battery preview (non-mutating) for safety context
%   3) safety supervisor -> final normalized command
%   4) battery authoritative update (single state mutation)
%   5) RK4 on mechanical states only
%   6) exact-ZOH current update once per sample

dt = safe_get(P, {'sim','dt'}, 1e-3);
T  = safe_get(P, {'sim','T_end'}, 5.0);
if ~(isfinite(dt) && dt > 0), dt = 1e-3; end
if ~(isfinite(T) && T > 0),   T = 5.0; end

N = floor(T/dt) + 1;
tlog = (0:(N-1)).' * dt;

x = safe_get(P, {'sim','x0'}, zeros(5,1));
x = x(:);
if numel(x) ~= 5
    error('simulate_case: x0 must be 5x1.');
end

xlog = zeros(N,5);
ulog = zeros(N,1);

mem = struct();
mem.int_i = 0.0;
mem.int_w = 0.0;
mem.soc_Ah_used = 0.0;
mem.Wh_used = 0.0;
mem.Vp = 0.0;
mem.soc = safe_get(P, {'batt','soc0'}, 1.0);
mem.safety = struct();

Lim = compute_sys_limits(P);
sig = init_sig(N);
sig.safety_enable = logical(safe_get(P, {'safety','enable'}, false));

stopOnFault = logical(safe_get(P, {'safety','on_fault','stop_sim'}, false));

for k = 1:N
    t = tlog(k);

    xlog(k,:) = x(:).';

    theta_b = x(1);
    omega_b = x(2);
    theta_w = x(3);
    omega_w = x(4);
    i_m     = x(5);

    theta_rel = theta_w - theta_b;
    omega_rel = omega_w - omega_b;

    % Controller
    [u_cmd_raw, mem, outc] = rw_controller(t, x, P, mem);
    u_cmd_raw = clamp(u_cmd_raw, -1, 1);

    sig.theta_ref(k)  = getfield_def(outc,'theta_ref',0);
    sig.theta_rel(k)  = theta_rel;
    sig.omega_ref(k)  = getfield_def(outc,'omega_ref',0);
    sig.i_ref(k)      = getfield_def(outc,'i_ref',0);
    sig.u_cmd_raw(k)  = u_cmd_raw;

    % Preview battery only for safety context
    mem_preview = mem;
    [batt0, ~] = battery_step(P, mem_preview, u_cmd_raw, i_m, omega_rel);

    u_cmd_safe = u_cmd_raw;
    flag = "GOOD";
    derate = 1.0;
    util = default_util();

    if sig.safety_enable
        meas = struct();
        meas.i         = i_m;
        meas.omega_w   = omega_w;
        meas.omega_b   = omega_b;
        meas.omega_rel = omega_rel;
        meas.Vbatt     = batt0.Vbatt;
        meas.Voc       = batt0.Voc;
        meas.Ibatt     = batt0.Ibatt;
        [u_cmd_safe, flag, util, derate, mem.safety] = safety_supervisor(t, u_cmd_raw, meas, Lim, P, mem.safety);
        u_cmd_safe = clamp(u_cmd_safe, -1, 1);
    end

    % Authoritative battery update (single mutation)
    [batt, mem] = battery_step(P, mem, u_cmd_safe, i_m, omega_rel);

    % Log battery / command
    ulog(k)            = u_cmd_safe;
    sig.u_cmd_norm(k)  = u_cmd_safe;
    sig.u(k)           = getfield_def(batt,'u',NaN);
    sig.Voc(k)         = getfield_def(batt,'Voc',NaN);
    sig.Vbatt(k)       = getfield_def(batt,'Vbatt',NaN);
    sig.Ibatt(k)       = getfield_def(batt,'Ibatt',NaN);
    sig.P_elec(k)      = getfield_def(batt,'P_elec',NaN);
    sig.P_batt(k)      = getfield_def(batt,'P_batt',NaN);
    sig.soc(k)         = getfield_def(batt,'soc',NaN);
    sig.Vp(k)          = getfield_def(batt,'Vp',NaN);

    % Disturbance breakdown for logging
    [tau_ext_b, tau_ext_w, dbgDist] = disturbance_torque(t, x, P);
    sig.tau_ext_b(k)  = tau_ext_b;
    sig.tau_ext_w(k)  = tau_ext_w;
    sig.tau_dist_b(k) = getfield_def(dbgDist,'tau_ext_b_total',0);
    sig.tau_vib_b(k)  = getfield_def(dbgDist,'tau_vib_b',0);
    sig.tau_imb_b(k)  = getfield_def(dbgDist,'tau_imb_b',0);
    sig.tau_preload_b(k) = getfield_def(dbgDist,'tau_preload_b',0);
    sig.tau_wire_b(k) = getfield_def(dbgDist,'tau_wire_b',0);

    % Log torques and powers
    sig.tau_m(k) = getfield_def(outc,'tau_m',0);
    sig.tau_w(k) = getfield_def(outc,'tau_w',0);

    epsw = max(safe_get(P, {'fric','omega_eps'}, 8e-3), 1e-9);

    Bb   = safe_get(P, {'base','Bb'}, 0.0);
    Tc_b = safe_get(P, {'base','Tc_b'}, 0.0);
    tau_base_env = Bb*omega_b + Tc_b*tanh(omega_b/epsw);
    sig.P_fric_b(k) = max(tau_base_env*omega_b, 0.0);

    Brel   = safe_get(P, {'wheel','Brel'}, safe_get(P, {'wheel','Bw'}, 0.0));
    Tc_rel = safe_get(P, {'wheel','Tc_rel'}, safe_get(P, {'wheel','Tc_w'}, 0.0));
    tau_joint = Brel*omega_rel + Tc_rel*tanh(omega_rel/epsw);
    sig.P_fric_w(k) = max(tau_joint*omega_rel, 0.0);

    sig.P_brake_b(k) = 0.0;
    if isfield(P,'dist') && isfield(P.dist,'base_brake') && getfield_def(P.dist.base_brake,'enable',false)
        B = getfield_def(P.dist.base_brake,'Bload',0.0);
        Tload = getfield_def(P.dist.base_brake,'Tload',0.0);
        tau_brake_b = B*omega_b + Tload*tanh(omega_b/epsw);
        sig.P_brake_b(k) = max(tau_brake_b*omega_b, 0.0);
    end

    sig.P_brake_w(k) = 0.0;
    if isfield(P,'dist') && isfield(P.dist,'wheel_brake') && getfield_def(P.dist.wheel_brake,'enable',false)
        B = getfield_def(P.dist.wheel_brake,'Bload',0.0);
        Tload = getfield_def(P.dist.wheel_brake,'Tload',0.0);
        tau_brake_w = B*omega_rel + Tload*tanh(omega_rel/epsw);
        sig.P_brake_w(k) = max(tau_brake_w*omega_rel, 0.0);
    end

    sig.P_wire(k) = 0.0;
    if isfield(P,'dist') && isfield(P.dist,'wire') && getfield_def(P.dist.wire,'enable',false)
        Dw = getfield_def(P.dist.wire,'D',0.0);
        sig.P_wire(k) = Dw*(omega_b^2);
    end

    Jeq = safe_get(P, {'motor','Jeq_out'}, 0.0);
    sig.P_mech_exchange(k) = sig.tau_w(k) * omega_rel;
    sig.h_w(k)      = safe_get(P, {'wheel','Jw'}, 0.0) * omega_w;
    sig.h_rel_act(k)= Jeq * omega_rel;

    % Integrate dynamics unless at final sample
    if k < N
        x_mech = x(1:4);
        k1 = rw_dynamics(t,           x,                    P, batt.u);
        x2 = [x_mech + 0.5*dt*k1; x(5)];
        k2 = rw_dynamics(t + 0.5*dt,  x2,                   P, batt.u);
        x3 = [x_mech + 0.5*dt*k2; x(5)];
        k3 = rw_dynamics(t + 0.5*dt,  x3,                   P, batt.u);
        x4 = [x_mech + dt*k3; x(5)];
        k4 = rw_dynamics(t + dt,      x4,                   P, batt.u);
        x_mech_next = x_mech + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        i_next = current_exact_zoh(x, P, batt.u, dt);
        x = [x_mech_next; i_next];
    end

    if sig.safety_enable
        sig.safety_flag(k)     = flag;
        sig.util_I_cont(k)     = util.I_cont;
        sig.util_I_peak(k)     = util.I_peak;
        sig.util_Ibatt_cont(k) = util.Ibatt_cont;
        sig.util_Ibatt_peak(k) = util.Ibatt_peak;
        sig.util_Vbus(k)       = util.Vbus;
        sig.util_omega(k)      = util.omega;
        sig.u_derate(k)        = derate;
        sig.fault_latched(k)   = isfield(mem.safety,'fault_latched') && logical(mem.safety.fault_latched);
    else
        sig.safety_flag(k) = "GOOD";
        sig.u_derate(k) = 1.0;
        sig.fault_latched(k) = false;
    end

    if stopOnFault && (flag == "FAULT")
        tlog = tlog(1:k);
        xlog = xlog(1:k,:);
        ulog = ulog(1:k);
        sig = trim_sig(sig, k);
        break;
    end
end

S = struct();
S.t = tlog;
S.x = xlog;
S.u = ulog;
S.sig = sig;
S.mem_end = mem;
S.Lim = Lim;

end

function i_next = current_exact_zoh(x, P, u, dt)
omega_b = x(2);
omega_w = x(4);
i       = x(5);

N  = safe_get(P, {'gear','N'}, 1.0);
R  = max(safe_get(P, {'motor','R'}, 1e-6), 1e-9);
L  = max(safe_get(P, {'motor','L'}, 1e-6), 1e-12);
Ke = safe_get(P, {'motor','Ke'}, 0.0);

omega_m = N * (omega_w - omega_b);
u_eff = u - Ke*omega_m;
alpha = exp(-(R/L)*dt);
if ~isfinite(alpha), alpha = 0; end
i_next = alpha*i + (1 - alpha)*(u_eff/R);
end

function sig = init_sig(N)
fields_num = { ...
    'theta_ref','theta_rel','omega_ref','i_ref','u_cmd_raw','u_cmd_norm', ...
    'tau_w','tau_m','Voc','Vbatt','Ibatt','u','P_elec','P_batt','soc','Vp', ...
    'P_mech_exchange','h_w','h_rel_act','P_fric_b','P_fric_w','P_brake_b', ...
    'P_brake_w','P_wire','tau_ext_b','tau_ext_w','tau_dist_b','tau_vib_b', ...
    'tau_imb_b','tau_preload_b','tau_wire_b','util_I_cont','util_I_peak', ...
    'util_Ibatt_cont','util_Ibatt_peak','util_Vbus','util_omega','u_derate'};
sig = struct();
for k = 1:numel(fields_num)
    sig.(fields_num{k}) = zeros(N,1);
end
sig.safety_enable = false;
sig.safety_flag = strings(N,1);
sig.fault_latched = false(N,1);
end

function sig = trim_sig(sig, k)
fn = fieldnames(sig);
for i = 1:numel(fn)
    v = sig.(fn{i});
    if isvector(v) && numel(v) >= k
        sig.(fn{i}) = v(1:k,:);
    end
end
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

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end

function util = default_util()
util = struct('I_cont',0,'I_peak',0,'Ibatt_cont',0,'Ibatt_peak',0,'Vbus',0,'omega',0);
end
