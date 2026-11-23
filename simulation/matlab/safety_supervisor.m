function [u_out, flag, util, derate, mem] = safety_supervisor(t, u_in, meas, Lim, P, mem)
%SAFETY_SUPERVISOR Command derating + fault logic.
%
% This version is aligned with params_default.m:
%   - warning thresholds live in P.safety.warn.*
%   - limit/derate thresholds live in P.safety.limit.*
%   - timers live in P.safety.timers.*
%   - fault action lives in P.safety.on_fault.*

if nargin < 6 || isempty(mem), mem = struct(); end

if ~isfield(P,'safety') || ~logical(safe_get(P, {'safety','enable'}, false))
    u_out  = clamp(u_in, -1, 1);
    flag   = "GOOD";
    util   = default_util();
    derate = 1.0;
    mem    = init_mem(mem, t);
    return;
end

S = P.safety;
u_in = clamp(u_in, -1, 1);
mem = init_mem(mem, t);

dt = t - mem.t_prev;
if ~isfinite(dt) || dt < 0, dt = 0; end
mem.t_prev = t;

i_m      = safe_get(meas, {'i'}, 0.0);
Vbatt    = safe_get(meas, {'Vbatt'}, safe_get(P, {'batt','V_nom'}, 12.0));
Ibatt    = safe_get(meas, {'Ibatt'}, 0.0);
omegaRel = safe_get(meas, {'omega_rel'}, 0.0);

abs_i  = abs(i_m);
abs_Ib = abs(Ibatt);
abs_w  = abs(omegaRel);

Icont = safe_get(Lim, {'I_cont'}, Inf);
Ipeak = safe_get(Lim, {'I_peak'}, Inf);
Vmax  = safe_get(Lim, {'Vbus_max'}, Inf);
wmax  = safe_get(Lim, {'omega_w_max'}, Inf);
Ib_cont = safe_get(Lim, {'Ibatt_cont'}, NaN);
Ib_peak = safe_get(Lim, {'Ibatt_peak'}, NaN);

util = default_util();
util.I_cont     = ratio(abs_i,  Icont);
util.I_peak     = ratio(abs_i,  Ipeak);
util.Ibatt_cont = ratio(abs_Ib, Ib_cont);
util.Ibatt_peak = ratio(abs_Ib, Ib_peak);
util.Vbus       = ratio(Vbatt,  Vmax);
util.omega      = ratio(abs_w,  wmax);

% thresholds
limit_Ic = safe_get(S, {'limit','I_cont_frac'}, 1.0);
limit_Ip = safe_get(S, {'limit','I_peak_frac'}, 1.0);
limit_Ib = safe_get(S, {'limit','Ibatt_cont_frac'}, limit_Ic);
limit_Ibp = safe_get(S, {'limit','Ibatt_peak_frac'}, limit_Ip);
limit_V  = safe_get(S, {'limit','Vbus_frac'}, 1.0);
limit_w  = safe_get(S, {'limit','omega_frac'}, 1.0);

warn_Ic  = safe_get(S, {'warn','I_cont_frac'}, min(0.8, 0.8*limit_Ic));
warn_Ib  = safe_get(S, {'warn','Ibatt_cont_frac'}, min(0.8, 0.8*limit_Ib));
warn_V   = safe_get(S, {'warn','Vbus_frac'}, min(0.85, 0.9*limit_V));
warn_w   = safe_get(S, {'warn','omega_frac'}, min(0.85, 0.9*limit_w));

smooth_enable = logical(safe_get(S, {'limit','smooth_enable'}, true));
smooth_tau_s  = max(safe_get(S, {'limit','smooth_tau_s'}, 0.05), 1e-6);
deadband      = safe_get(S, {'deadband_u'}, 0.01);

% timers
t_Ic  = safe_get(S, {'timers','I_cont_s'}, 0.20);
t_Ip  = safe_get(S, {'timers','I_peak_s'}, 0.02);
t_Ib  = safe_get(S, {'timers','Ibatt_cont_s'}, 0.20);
t_Ibp = safe_get(S, {'timers','Ibatt_peak_s'}, 0.02);
t_V   = safe_get(S, {'timers','Vbus_s'}, 0.05);
t_w   = safe_get(S, {'timers','omega_s'}, 0.05);

latch   = logical(safe_get(S, {'on_fault','latch'}, true));
action  = string(safe_get(S, {'on_fault','action'}, "zero_cmd"));
holdTau = safe_get(S, {'on_fault','hold_tau_s'}, 0.0);
t_recover = safe_get(S, {'recovery','t_recover'}, 0.25);

hard_clip_enable = logical(safe_get(S, {'hard_current_clip','enable'}, false));
hard_clip_frac   = safe_get(S, {'hard_current_clip','frac'}, limit_Ip);

warn = false;
warn = warn || (isfinite(util.I_cont)     && util.I_cont     >= warn_Ic);
regen_active = (Ibatt < -1e-3);
warn = warn || (regen_active && isfinite(util.Vbus) && util.Vbus >= warn_V);
warn = warn || (isfinite(util.omega)      && util.omega      >= warn_w);
warn = warn || (isfinite(util.Ibatt_cont) && util.Ibatt_cont >= warn_Ib);

cond_i_peak  = isfinite(util.I_peak)      && (util.I_peak      >= limit_Ip);
cond_i_cont  = isfinite(util.I_cont)      && (util.I_cont      >= 1.0);
cond_vbus    = regen_active && isfinite(util.Vbus) && (util.Vbus >= 1.0);
cond_omega   = isfinite(util.omega)       && (util.omega       >= 1.0);
cond_ib_peak = isfinite(util.Ibatt_peak)  && (util.Ibatt_peak  >= limit_Ibp);
cond_ib_cont = isfinite(util.Ibatt_cont)  && (util.Ibatt_cont  >= 1.0);

mem.timers = timer_update(mem.timers,'i_peak',  cond_i_peak,  dt);
mem.timers = timer_update(mem.timers,'i_cont',  cond_i_cont,  dt);
mem.timers = timer_update(mem.timers,'vbus',    cond_vbus,    dt);
mem.timers = timer_update(mem.timers,'omega',   cond_omega,   dt);
mem.timers = timer_update(mem.timers,'ib_peak', cond_ib_peak, dt);
mem.timers = timer_update(mem.timers,'ib_cont', cond_ib_cont, dt);

fault_any = (mem.timers.i_peak  >= t_Ip)  || ...
            (mem.timers.i_cont  >= t_Ic)  || ...
            (mem.timers.vbus    >= t_V)   || ...
            (mem.timers.omega   >= t_w)   || ...
            (mem.timers.ib_peak >= t_Ibp) || ...
            (mem.timers.ib_cont >= t_Ib);

if fault_any
    if latch
        mem.fault_latched = true;
    end
    mem.t_fault_clear = 0.0;
else
    mem.t_fault_clear = mem.t_fault_clear + dt;
    if (~latch) && (mem.t_fault_clear >= t_recover)
        mem.fault_latched = false;
    end
end

% Derating (soft limit zone)
derate_raw = 1.0;
if abs(u_in) >= deadband
    if isfinite(util.I_cont) && util.I_cont >= limit_Ic && isfinite(Icont) && Icont > 0
        derate_raw = min(derate_raw, (limit_Ic*Icont) / max(abs_i, 1e-12));
    end
    if isfinite(util.I_peak) && util.I_peak >= limit_Ip && isfinite(Ipeak) && Ipeak > 0
        derate_raw = min(derate_raw, (limit_Ip*Ipeak) / max(abs_i, 1e-12));
    end
    if isfinite(util.Ibatt_cont) && util.Ibatt_cont >= limit_Ib && isfinite(Ib_cont) && Ib_cont > 0
        derate_raw = min(derate_raw, (limit_Ib*Ib_cont) / max(abs_Ib, 1e-12));
    end
    if isfinite(util.Ibatt_peak) && util.Ibatt_peak >= limit_Ibp && isfinite(Ib_peak) && Ib_peak > 0
        derate_raw = min(derate_raw, (limit_Ibp*Ib_peak) / max(abs_Ib, 1e-12));
    end
    if isfinite(util.omega) && util.omega >= limit_w && isfinite(wmax) && wmax > 0
        derate_raw = min(derate_raw, (limit_w*wmax) / max(abs_w, 1e-12));
    end
    if regen_active && isfinite(util.Vbus) && util.Vbus >= limit_V && isfinite(Vmax) && Vmax > 0
        derate_raw = min(derate_raw, (limit_V*Vmax) / max(Vbatt, 1e-12));
    end
end

derate_raw = clamp(derate_raw, 0.0, 1.0);
if smooth_enable
    alpha = 1 - exp(-dt / smooth_tau_s);
    mem.derate = mem.derate + alpha*(derate_raw - mem.derate);
    derate = clamp(mem.derate, 0.0, 1.0);
else
    derate = derate_raw;
    mem.derate = derate;
end

isLimit = (derate < 0.999);

if mem.fault_latched || fault_any
    flag = "FAULT";
    [u_out, mem] = fault_action(u_in, action, holdTau, dt, mem);
elseif isLimit
    flag  = "LIMIT";
    u_out = clamp(derate*u_in, -1, 1);
elseif warn
    flag  = "WARN";
    u_out = clamp(u_in, -1, 1);
else
    flag  = "GOOD";
    u_out = clamp(u_in, -1, 1);
end

if hard_clip_enable && isfinite(util.I_cont) && util.I_cont >= hard_clip_frac && abs(u_out) > 0 && isfinite(Icont) && Icont > 0
    scale = (hard_clip_frac*Icont) / max(abs_i, 1e-12);
    scale = clamp(scale, 0.0, 1.0);
    u_out = clamp(scale*u_out, -1, 1);
    if flag == "GOOD"
        flag = "LIMIT";
    end
end

if logical(safe_get(S, {'verbose'}, false))
    fprintf('[safety] t=%.3f flag=%s u_in=%.3f u_out=%.3f derate=%.3f | I=%.2fA utilIc=%.2f utilW=%.2f\n', ...
        t, flag, u_in, u_out, derate, i_m, util.I_cont, util.omega);
end

end

function mem = init_mem(mem, t)
if ~isfield(mem,'t_prev') || ~isfinite(mem.t_prev), mem.t_prev = t; end
if ~isfield(mem,'timers') || ~isstruct(mem.timers), mem.timers = struct(); end
if ~isfield(mem,'derate') || ~isfinite(mem.derate), mem.derate = 1.0; end
if ~isfield(mem,'fault_latched'), mem.fault_latched = false; end
if ~isfield(mem,'t_fault_clear') || ~isfinite(mem.t_fault_clear), mem.t_fault_clear = 0.0; end
if ~isfield(mem,'u_hold'), mem.u_hold = []; end
if ~isfield(mem,'t_hold') || ~isfinite(mem.t_hold), mem.t_hold = 0.0; end
end

function [u_out, mem] = fault_action(u_in, action, holdTau, dt, mem)
switch lower(char(action))
    case 'zero_cmd'
        u_out = 0.0;
    case 'hold_last'
        if isempty(mem.u_hold)
            mem.u_hold = u_in;
            mem.t_hold = 0.0;
        end
        if holdTau > 0
            mem.t_hold = mem.t_hold + dt;
            if mem.t_hold <= holdTau
                u_out = clamp(mem.u_hold, -1, 1);
            else
                u_out = 0.0;
            end
        else
            u_out = clamp(mem.u_hold, -1, 1);
        end
    otherwise
        u_out = 0.0;
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

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end

function r = ratio(x, lim)
if ~isfinite(lim) || lim <= 0
    r = NaN;
else
    r = x / lim;
end
end

function timers = timer_update(timers, name, cond, dt)
if ~isfield(timers, name), timers.(name) = 0.0; end
if cond
    timers.(name) = timers.(name) + dt;
else
    timers.(name) = 0.0;
end
end

function util = default_util()
util = struct('I_cont',0,'I_peak',0,'Ibatt_cont',0,'Ibatt_peak',0,'Vbus',0,'omega',0);
end
