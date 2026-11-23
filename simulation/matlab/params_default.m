function P = params_default(Pin)
%PARAMS_DEFAULT Final study parameters for the reaction-wheel lab model.
%
% This parameter set is aligned with the stored project assumptions:
% - reaction wheel experiment uses tau = Iw * domega_w/dt
% - active wheel is the one-piece webbed wheel with radial tuning
% - motor family is the Pololu 37D 12 V 19:1 geared family
% - user current adopted inertias are used directly
%
% IMPORTANT INERTIA SPLIT
% ----------------------
% J_total(rotating stack with wheel) = J_body_excluding_wheel + J_wheel
%
% Adopted current values:
%   J_wheel = 0.66e-3 kg*m^2
%   J_total = 2.00e-3 kg*m^2
% therefore
%   J_body  = 1.34e-3 kg*m^2

if nargin < 1 || isempty(Pin)
    P = struct();
else
    P = Pin;
end

%% ================= Simulation =================
P.sim.T_end = 12.0;
P.sim.dt    = 1e-3;
P.sim.caseName = "base_angle_PD";
P.sim.x0 = [0; 0; 0; 0; 0];
P.sim.doPlot = true;
P.sim.doAuditPrint = true;

%% ================= Metadata =================
P.meta.project = "Reaction_Wheel_Lab_Project";
P.meta.release = "simulation_final_validated_19";
P.meta.motor_family = "Pololu_37D_12V_19_encoder";
P.meta.wheel_type   = "one_piece_webbed_wheel_with_radial_tuning";

%% ================= Inertias =================
P.stack.J_total = 2.00e-3;        % [kg*m^2] rotating body including wheel
P.wheel.Jw_cad_baseline = 0.639e-3; % [kg*m^2] stored CAD baseline
P.wheel.Jw = 0.66e-3;               % [kg*m^2] current adopted wheel inertia
P.base.Jb  = P.stack.J_total - P.wheel.Jw; % [kg*m^2] body excluding wheel

% Gear-train reflected output inertia (UNMEASURED -> default zero)
% Do NOT use guessed rotor inertia reflected by N^2 unless identified.
P.motor.Jeq_out = 0.0;            % [kg*m^2] equivalent actuator relative inertia at output side

%% ================= Wheel geometry / CAD references =================
P.wheel.Do = 0.140;          % [m]
P.wheel.Di = 0.114;          % [m]
P.wheel.thk = 0.010;         % [m]
P.wheel.tuneStations = 8;
P.wheel.tuneTapDrill = 3.3e-3;    % [m]
P.wheel.tuneDepth    = 10e-3;     % [m]
P.wheel.tuneSetScrewNominal = 4e-3; % [m]
P.wheel.tuneSetScrewLength  = 8e-3; % [m]

%% ================= Base friction (support-to-world) =================
P.base.Bb   = 1.5e-3;    % [N*m/(rad/s)]
P.base.Tc_b = 6.0e-3;    % [N*m]
P.base.stiction.enable     = true;
P.base.stiction.Ts_b       = 1.2e-2;  % [N*m]
P.base.stiction.omega_lock = 0.03;    % [rad/s]

%% ================= Internal wheel-to-base joint dissipation =================
% This is INTERNAL dissipation in relative motion omega_rel = omega_w - omega_b.
P.wheel.Brel   = 8.0e-4;   % [N*m/(rad/s)]
P.wheel.Tc_rel = 3.0e-3;   % [N*m]
% Backward-compatible aliases
P.wheel.Bw   = P.wheel.Brel;
P.wheel.Tc_w = P.wheel.Tc_rel;
P.wheel.stiction.enable = false;

%% ================= Friction smoothing =================
P.fric.omega_eps = 8e-3;

%% ================= Gear family =================
% Current hardware: 19:1
P.gear.family = 19;
P.gear.N = 19.0;
P.gear.eta = 0.73;
P.gear.outputSpeedNoLoad_rpm = 530;
P.gear.outputCPR = 1200;
P.gear.outputSpeedNoLoad_rad_s = P.gear.outputSpeedNoLoad_rpm * 2*pi/60;

%% ================= Motor electrical / torque model =================
% Consistent with 12 V, 19:1 family-scale specs.
P.motor.R  = 12.0 / 5.5;    % [Ohm] from stall current scale
P.motor.L  = 0.6e-3;        % [H] study placeholder
P.motor.Kt = 0.0110;        % [N*m/A] shaft-side torque constant
P.motor.Ke = 0.0110;        % [V/(rad/s)] shaft-side back-EMF constant
P.motor.I0 = 0.20;          % [A] no-load current family value
P.motor.shaft_d = 6e-3;
P.motor.shaft_L = 16e-3;

%% ================= Driver =================
P.driver.model     = "MD15B_class";
P.driver.V_max_cmd = 1.0;
P.driver.eta       = 0.95;
P.driver.iface.mode   = "sign_pwm";
P.driver.iface.pwm_hz = 20000;
P.driver.cs.enable       = true;
P.driver.cs.vref         = 5.0;
P.driver.cs.adc_counts   = 1023;
P.driver.cs.gain_A_per_V = NaN;
P.driver.cs.offset_V     = 0.0;
P.driver.cs.lpf_tau_s    = 0.005;

%% ================= Battery =================
P.batt.model = "Rint";      % "Rint" or "RC"
P.batt.V_nom = 12.0;
P.batt.R_int = 0.15;
P.batt.Ah    = 2.0;
P.batt.V_min = 0.5;
P.batt.soc0  = 1.0;
P.batt.V_full  = 1.08 * P.batt.V_nom;
P.batt.V_empty = 0.88 * P.batt.V_nom;
P.batt.R_p = 0.10;
P.batt.C_p = 200.0;
P.batt.regen.mode = "clamp";   % "clamp" | "allow" | "none"
P.batt.regen.eta  = 0.9;

%% ================= Controller =================
P.ctrl.theta_ref = deg2rad(30);
P.ctrl.t_step    = 0.3;

% Base angle PD -> desired wheel torque
P.ctrl.base.Kp = 0.35;
P.ctrl.base.Kd = 0.06;

% Wheel speed PI -> current PI (used in wheel_speed_PI mode)
P.ctrl.wheel.omega_ref = 20;   % [rad/s] conservative 19:1 study reference
P.ctrl.wheel.Kp        = 0.01;
P.ctrl.wheel.Ki        = 0.2;
P.ctrl.wheel.int_w_max = 20.0;

% Current loop
P.ctrl.current.Kp        = 2.0;
P.ctrl.current.Ki        = 40.0;
P.ctrl.current.i_max     = 3.0;   % software clamp
P.ctrl.current.int_i_max = 2.0;

% Open-loop
P.ctrl.openloop.u_step       = 0.35;
P.ctrl.openloop.enable_pulse = false;
P.ctrl.openloop.t_on         = 0.3;
P.ctrl.openloop.t_off        = 2.0;

%% ================= Disturbances / loads =================
P.dist.tau_ext_b.enable  = false; P.dist.tau_ext_b.value = 0.0;
P.dist.tau_step_b.enable = false; P.dist.tau_step_b.value = 0.01; P.dist.tau_step_b.t_on = 1.0;
P.dist.tau_sin_b.enable  = false; P.dist.tau_sin_b.amp = 0.005; P.dist.tau_sin_b.freqHz = 1.0; P.dist.tau_sin_b.phase0 = 0.0;

P.dist.tau_ext_w.enable  = false; P.dist.tau_ext_w.value = 0.0;
P.dist.tau_step_w.enable = false; P.dist.tau_step_w.value = 0.0; P.dist.tau_step_w.t_on = 1.0;
P.dist.tau_sin_w.enable  = false; P.dist.tau_sin_w.amp = 0.0; P.dist.tau_sin_w.freqHz = 1.0; P.dist.tau_sin_w.phase0 = 0.0;

P.dist.base_brake.enable  = false; P.dist.base_brake.Tload = 0.0; P.dist.base_brake.Bload = 0.0;
P.dist.wheel_brake.enable = false; P.dist.wheel_brake.Tload = 0.0; P.dist.wheel_brake.Bload = 0.0;

P.dist.wire.enable = false; P.dist.wire.K = 0.03; P.dist.wire.D = 0.004;

P.dist.vib.enable = false; P.dist.vib.tau_b_amp = 1e-3; P.dist.vib.freqHz = 18; P.dist.vib.phase0 = 0.0;
P.dist.vib.imbalance_enable = false; P.dist.vib.k_imb = 0.0;

P.dist.preload.enable = false;
P.dist.preload.tau_ripple0 = 0.0;
P.dist.preload.k_ripple_w2 = 0.0;
P.dist.preload.ripple_harm = 1;
P.dist.preload.phase0 = 0.0;
P.dist.preload.Tc_boost = 1.0;
P.dist.preload.omega_stick = 1.0;

%% ================= Safety =================
P.safety.enable = true;
P.safety.warn_missing_limits = false;
P.safety.limit.smooth_enable = true;
P.safety.limit.smooth_tau_s  = 0.05;
P.safety.on_fault.action = "zero_cmd";
P.safety.on_fault.hold_tau_s = 0.0;
P.safety.on_fault.latch = true;
P.safety.on_fault.stop_sim = false;
P.safety.recovery.t_recover = 0.25;
P.safety.deadband_u = 0.02;
P.safety.verbose = false;

% Derating thresholds as fractions of computed limits
P.safety.limit.I_cont_frac     = 0.90;
P.safety.limit.I_peak_frac     = 0.98;
P.safety.limit.Ibatt_cont_frac = 0.90;
P.safety.limit.Ibatt_peak_frac = 0.98;
P.safety.limit.Vbus_frac       = 0.98;
P.safety.limit.omega_frac      = 0.98;

% Warning thresholds (can be omitted; defaults use 80% of limit thresholds)
P.safety.warn.I_cont_frac      = 0.75;
P.safety.warn.Ibatt_cont_frac  = 0.75;
P.safety.warn.Vbus_frac        = 0.85;
P.safety.warn.omega_frac       = 0.85;

% Fault timers
P.safety.timers.I_cont_s       = 0.20;
P.safety.timers.I_peak_s       = 0.02;
P.safety.timers.Ibatt_cont_s   = 0.20;
P.safety.timers.Ibatt_peak_s   = 0.02;
P.safety.timers.Vbus_s         = 0.05;
P.safety.timers.omega_s        = 0.05;

P.safety.hard_current_clip.enable = false;
P.safety.hard_current_clip.frac   = 0.98;

%% ================= Limits =================
P.limits.motor.I_cont = 3.0;   % [A] conservative continuous study limit
P.limits.motor.I_peak = 5.5;   % [A] family stall-current-scale ceiling
P.limits.driver.Vbus_max = max(P.batt.V_full, 1.05*P.batt.V_nom);
P.limits.wheel.omega_max = 0.98 * P.gear.outputSpeedNoLoad_rad_s;

% Optional battery limits (fill if exact pack/BMS values are known)
P.limits.batt.I_cont = NaN;
P.limits.batt.I_peak = NaN;

%% ================= Plot options =================
P.plot.wrap_angle_deg = false;

% Merge caller overrides last if Pin was provided
if nargin >= 1 && ~isempty(Pin)
    P = merge_structs(P, Pin);
end

end

function S = merge_structs(S, U)
%MERGE_STRUCTS Recursive shallow-deep merge: user overrides default fields.
if ~isstruct(U), return; end
f = fieldnames(U);
for k = 1:numel(f)
    name = f{k};
    if isstruct(U.(name))
        if ~isfield(S,name) || ~isstruct(S.(name))
            S.(name) = U.(name);
        else
            S.(name) = merge_structs(S.(name), U.(name));
        end
    else
        S.(name) = U.(name);
    end
end
end
