function [u_cmd_norm, mem, out] = rw_controller(t, x, P, mem)
%RW_CONTROLLER Reaction-wheel control law -> normalized command in [-1,1].
%
% State:
%   x = [theta_b; omega_b; theta_w; omega_w; i]
%
% IMPORTANT:
% Motor and gearbox act on the RELATIVE wheel motion:
%   theta_rel = theta_w - theta_b
%   omega_rel = omega_w - omega_b
%
% Supported modes:
%   "open_loop_stepV" : normalized open-loop command
%   "wheel_speed_PI"  : relative wheel speed PI -> current PI
%   otherwise         : base angle PD -> desired wheel torque -> current PI

if nargin < 4 || isempty(mem), mem = struct(); end
assert(numel(x) == 5, 'rw_controller: state x must be 5x1.');

dt = safe_get(P, {'sim','dt'}, 1e-3);
if ~(isfinite(dt) && dt > 0), dt = 1e-3; end

VmaxCmd = clamp(safe_get(P, {'driver','V_max_cmd'}, 1.0), 0.0, 1.0);
caseName = string(safe_get(P, {'sim','caseName'}, "base_angle_PD"));

% Reset integrators on mode switch
if ~isfield(mem,'mode_prev') || ~strcmp(string(mem.mode_prev), caseName)
    mem.int_i = 0.0;
    mem.int_w = 0.0;
    mem.mode_prev = caseName;
end

theta_b = x(1);
omega_b = x(2);
theta_w = x(3);
omega_w = x(4);
i       = x(5);

theta_rel = theta_w - theta_b;
omega_rel = omega_w - omega_b;

N   = safe_get(P, {'gear','N'}, 1.0);
eta = safe_get(P, {'gear','eta'}, 1.0);
Kt  = safe_get(P, {'motor','Kt'}, 0.0);

tau_m = Kt * i;
tau_w = eta * N * tau_m;

t_step    = safe_get(P, {'ctrl','t_step'}, 0.0);
theta_ref = (t >= t_step) * safe_get(P, {'ctrl','theta_ref'}, 0.0);

i_ref      = 0.0;
u_cmd_norm = 0.0;
omega_ref  = 0.0;
tau_w_des  = 0.0;

switch caseName

    case "open_loop_stepV"
        u_step = safe_get(P, {'ctrl','openloop','u_step'}, 0.0);

        enable_pulse = logical(safe_get(P, {'ctrl','openloop','enable_pulse'}, false));
        if enable_pulse
            t_on  = safe_get(P, {'ctrl','openloop','t_on'}, 0.0);
            t_off = safe_get(P, {'ctrl','openloop','t_off'}, inf);
            on = (t >= t_on) && (t < t_off);
            u_cmd_norm = on * u_step;
        else
            u_cmd_norm = (t >= t_step) * u_step;
        end
        u_cmd_norm = clamp(u_cmd_norm, -VmaxCmd, +VmaxCmd);

    case "wheel_speed_PI"
        omega_ref = (t >= t_step) * safe_get(P, {'ctrl','wheel','omega_ref'}, 0.0);
        e_w = omega_ref - omega_rel;

        if ~isfield(mem,'int_w'), mem.int_w = 0.0; end
        int_w_prev = mem.int_w;
        int_w_prop = int_w_prev + e_w * dt;

        int_w_max = abs(safe_get(P, {'ctrl','wheel','int_w_max'}, inf));
        if isfinite(int_w_max)
            int_w_prop = clamp(int_w_prop, -int_w_max, +int_w_max);
        end

        Kp_w = safe_get(P, {'ctrl','wheel','Kp'}, 0.0);
        Ki_w = safe_get(P, {'ctrl','wheel','Ki'}, 0.0);

        i_prop = Kp_w*e_w + Ki_w*int_w_prop;
        i_max = abs(safe_get(P, {'ctrl','current','i_max'}, inf));
        if isfinite(i_max)
            i_ref = clamp(i_prop, -i_max, +i_max);
        else
            i_ref = i_prop;
        end

        % Simple anti-windup: only accept the wheel-speed integrator update
        % if the unsaturated request is inside bounds or would drive back toward bounds.
        if ~isfinite(i_max) || (i_prop == i_ref) || ((i_prop > i_max) && (e_w < 0)) || ((i_prop < -i_max) && (e_w > 0))
            mem.int_w = int_w_prop;
        else
            mem.int_w = int_w_prev;
        end

        [u_cmd_norm, mem] = current_PI_to_norm(i_ref, x, P, mem, dt, VmaxCmd);

    otherwise
        % Base angle control:
        % positive tau_w accelerates wheel + and base -.
        % Therefore to move base toward +theta_ref we need negative tau_w_des.
        e_th = theta_ref - theta_b;
        e_wb = 0.0 - omega_b;

        Kp_b = safe_get(P, {'ctrl','base','Kp'}, 0.0);
        Kd_b = safe_get(P, {'ctrl','base','Kd'}, 0.0);

        tau_w_des = -(Kp_b*e_th + Kd_b*e_wb);

        denom = max(eta * N * Kt, 1e-9);
        i_ref = tau_w_des / denom;

        i_max = abs(safe_get(P, {'ctrl','current','i_max'}, inf));
        if isfinite(i_max)
            i_ref = clamp(i_ref, -i_max, +i_max);
        end

        [u_cmd_norm, mem] = current_PI_to_norm(i_ref, x, P, mem, dt, VmaxCmd);
end

out = struct();
out.theta_ref  = theta_ref;
out.theta_rel  = theta_rel;
out.omega_ref  = omega_ref;
out.i_ref      = i_ref;
out.tau_w_des  = tau_w_des;
out.tau_m      = tau_m;
out.tau_w      = tau_w;
out.u_cmd_norm = u_cmd_norm;
out.caseName   = caseName;

end

function [u_cmd_norm, mem] = current_PI_to_norm(i_ref, x, P, mem, dt, VmaxCmd)
omega_b = x(2);
omega_w = x(4);
i       = x(5);

N  = safe_get(P, {'gear','N'}, 1.0);
Ke = safe_get(P, {'motor','Ke'}, 0.0);

omega_m = N * (omega_w - omega_b);   % relative motor speed
e_i = i_ref - i;

if ~isfield(mem,'int_i'), mem.int_i = 0.0; end
int_i_prev = mem.int_i;
int_i_prop = int_i_prev + e_i * dt;

int_i_max = abs(safe_get(P, {'ctrl','current','int_i_max'}, inf));
if isfinite(int_i_max)
    int_i_prop = clamp(int_i_prop, -int_i_max, +int_i_max);
end

Vff = Ke * omega_m;
Kp_i = safe_get(P, {'ctrl','current','Kp'}, 0.0);
Ki_i = safe_get(P, {'ctrl','current','Ki'}, 0.0);
Vcmd_unsat  = Vff + Kp_i*e_i + Ki_i*int_i_prop;

Vnom = safe_get(P, {'batt','V_nom'}, 12.0);
u_unsat = Vcmd_unsat / max(Vnom, 1e-6);
u_cmd_norm = clamp(u_unsat, -VmaxCmd, +VmaxCmd);

% Simple anti-windup: only accept integration if the controller is either
% not saturated or the error would drive the command back inside the bounds.
if (u_unsat == u_cmd_norm) || ((u_unsat > VmaxCmd) && (e_i < 0)) || ((u_unsat < -VmaxCmd) && (e_i > 0))
    mem.int_i = int_i_prop;
else
    mem.int_i = int_i_prev;
end
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
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
            v = defaultVal;
            return;
        end
    end
    if isempty(tmp) || (isnumeric(tmp) && any(~isfinite(tmp(:))))
        v = defaultVal;
    else
        v = tmp;
    end
catch
    v = defaultVal;
end
end
