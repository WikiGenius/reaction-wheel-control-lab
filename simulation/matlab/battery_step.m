function [batt, mem] = battery_step(P, mem, u_cmd_norm, i_motor, omega_rel)
%BATTERY_STEP  Battery + driver power model (discrete-time, quasi-static).
%
% Role in architecture
% --------------------
% This is the *only* place that converts normalized command to motor voltage:
%   u = u_cmd_norm * Vbatt
% It also estimates battery current and sag consistently:
%   Vbatt = Voc(SOC) - Ibatt*R_int - Vp   (Vp only in RC model)
%
% Sign conventions (CRITICAL)
% ---------------------------
% - i_motor is the motor electrical current state in rw_dynamics [A].
% - Motor electrical terminal power:
%       P_elec = u * i_motor
%   If P_elec > 0  => motoring (battery provides power)
%   If P_elec < 0  => generating/regen (motor returns power to rails)
%
% - Battery current sign:
%       Ibatt > 0  => battery DISCHARGING (supplying power)
%       Ibatt < 0  => battery CHARGING (absorbing power)  [only if regen allowed]
%
% Driver efficiency model (simple but consistent)
% -----------------------------------------------
% Let etaDrv in (0..1).
% - Motoring: battery power is higher than motor electrical power
%       P_batt ~= P_elec / etaDrv
% - Regen: battery absorbed power is lower magnitude than motor electrical power
%       P_batt ~= P_elec * etaRegen   (P_elec is negative)
%   where etaRegen in (0..1).
%
% Regen policy (P.batt.regen.mode)
% -------------------------------
%   "clamp" : disallow charging -> Ibatt >= 0 (regen is dumped/clamped)
%   "none"  : ignore regen -> Ibatt = 0 if computed charging
%   "allow" : allow Ibatt < 0 (charging), with efficiency etaRegen applied
%
% Battery models supported
% ------------------------
%   P.batt.model = "Rint" : Vbatt = Voc(soc) - Ibatt*R_int
%   P.batt.model = "RC"   : Vbatt = Voc(soc) - Ibatt*R_int - Vp
%                           dVp/dt = -(1/(RpCp))Vp + (1/Cp)Ibatt
%
% Notes
% -----
% - This is quasi-static (no inductors/capacitors on rails except optional RC polarization).
% - Uses a small fixed-point iteration because u depends on Vbatt, and Ibatt depends on u.
% - omega_rel is currently unused; kept for future extensions (speed-dependent losses).

%% ---------------- Defaults & memory init ----------------
dt = safe_get(P, {'sim','dt'}, 1e-3);
if ~(isfinite(dt) && dt > 0), dt = 1e-3; end

if nargin < 2 || isempty(mem), mem = struct(); end

if ~isfield(mem,'soc') || isempty(mem.soc)
    mem.soc = safe_get(P, {'batt','soc0'}, 1.0);
end
if ~isfield(mem,'Vp') || isempty(mem.Vp)
    mem.Vp = 0.0;
end
if ~isfield(mem,'soc_Ah_used') || isempty(mem.soc_Ah_used)
    mem.soc_Ah_used = 0.0;
end
if ~isfield(mem,'Wh_used') || isempty(mem.Wh_used)
    mem.Wh_used = 0.0;
end

%% ---------------- Parameters ----------------
Vmin    = safe_get(P, {'batt','V_min'}, 0.5);
Vmin    = max(Vmin, 0.0);

Ah_cap  = safe_get(P, {'batt','Ah'}, 1.0);
Ah_cap  = max(Ah_cap, 1e-9);

Rint    = safe_get(P, {'batt','R_int'}, 0.15);
Rint    = max(Rint, 0.0);

etaDrv  = safe_get(P, {'driver','eta'}, 0.95);
etaDrv  = clamp(etaDrv, 1e-3, 1.0);

% Regen settings
regenMode = string(safe_get(P, {'batt','regen','mode'}, "clamp"));
etaRegen  = safe_get(P, {'batt','regen','eta'}, 0.9);
etaRegen  = clamp(etaRegen, 1e-3, 1.0);

% Command saturation
VmaxCmd = safe_get(P, {'driver','V_max_cmd'}, 1.0);
VmaxCmd = clamp(VmaxCmd, 0.0, 1.0);
u_cmd_norm_sat = clamp(u_cmd_norm, -VmaxCmd, +VmaxCmd);

% Battery model selection
model = string(safe_get(P, {'batt','model'}, "Rint"));

% OCV
Voc = ocv_from_soc(mem.soc, P);

% RC polarization branch
Vp = mem.Vp;
if model == "RC"
    V_internal_offset = Vp;
else
    V_internal_offset = 0.0;
end

%% ---------------- Fixed-point iteration ----------------
% We solve for consistent (Vbatt, Ibatt) where:
%   u = u_cmd_norm_sat * Vbatt
%   P_elec = u*i_motor
%   P_batt demanded/absorbed depends on motoring vs regen and eta
%   Ibatt = P_batt / Vbatt
%   Vbatt = Voc - Ibatt*Rint - V_internal_offset
%
% Start guess: no sag
Vbatt = max(Voc - V_internal_offset, Vmin);
Ibatt = 0.0;

nIter = 3;  % small and stable

for it = 1:nIter
    % Motor terminal voltage
    u = u_cmd_norm_sat * Vbatt;

    % Motor electrical terminal power
    P_elec = u * i_motor;

    % Convert to battery-side power with efficiency
    if P_elec >= 0
        % Motoring: battery must supply more
        P_batt = P_elec / etaDrv;
    else
        % Regen: battery absorbs less magnitude (losses)
        % P_elec is negative, P_batt should also be negative
        P_batt = P_elec * etaRegen;
    end

    % Battery current from power (avoid divide by tiny V)
    Ibatt = P_batt / max(Vbatt, max(Vmin,1e-6));

    % Apply regen policy
    switch regenMode
        case "clamp"
            % no charging allowed: clamp Ibatt to >=0
            Ibatt = max(Ibatt, 0.0);

        case "none"
            % ignore regen completely
            if Ibatt < 0, Ibatt = 0.0; end

        case "allow"
            % allow Ibatt < 0
            % keep as is

        otherwise
            regenMode = "clamp";
            Ibatt = max(Ibatt, 0.0);
    end

    % Update Vbatt with internal resistance and polarization
    Vbatt = Voc - Ibatt*Rint - V_internal_offset;
    Vbatt = max(Vbatt, Vmin);
end

%% ---------------- Final recompute with converged values ----------------
u = u_cmd_norm_sat * Vbatt;
P_elec = u * i_motor;

if P_elec >= 0
    P_batt = P_elec / etaDrv;
else
    P_batt = P_elec * etaRegen;
end

Ibatt = P_batt / max(Vbatt, max(Vmin,1e-6));

% Regen policy (final)
switch regenMode
    case "clamp"
        Ibatt = max(Ibatt, 0.0);
    case "none"
        if Ibatt < 0, Ibatt = 0.0; end
    case "allow"
        % keep
end

Vbatt = Voc - Ibatt*Rint - V_internal_offset;
Vbatt = max(Vbatt, Vmin);

% Recompute final battery power (consistent with final V and I)
P_batt = Vbatt * Ibatt;

%% ---------------- Update RC polarization state (if enabled) ----------------
if model == "RC"
    Rp = max(safe_get(P, {'batt','R_p'}, 0.1), 1e-9);
    Cp = max(safe_get(P, {'batt','C_p'}, 200.0), 1e-9);

    % dVp/dt = -(1/(RpCp))Vp + (1/Cp)Ibatt
    dVp = (-(1/(Rp*Cp))*mem.Vp + (1/Cp)*Ibatt);
    mem.Vp = mem.Vp + dt * dVp;
else
    mem.Vp = 0.0;
end

%% ---------------- SOC bookkeeping ----------------
% dAh signed: positive = discharge, negative = charge
dAh = (Ibatt * dt) / 3600;

if regenMode == "allow"
    if dAh > 0
        % discharge
        mem.soc = mem.soc - dAh / Ah_cap;
        mem.soc_Ah_used = mem.soc_Ah_used + dAh;
        % Wh_used counts discharge energy only
        mem.Wh_used = mem.Wh_used + max(P_batt,0) * (dt/3600);
    else
        % charge -> SOC increases, but with efficiency already in P_batt model
        mem.soc = mem.soc - dAh / Ah_cap; % dAh negative => increases SOC
        % Do not "un-use" Ah_used to keep that metric monotonic (by design)
    end
else
    % no charging modeled: only discharge reduces SOC
    if dAh > 0
        mem.soc = mem.soc - dAh / Ah_cap;
        mem.soc_Ah_used = mem.soc_Ah_used + dAh;
        mem.Wh_used = mem.Wh_used + max(P_batt,0) * (dt/3600);
    end
end

mem.soc = clamp(mem.soc, 0.0, 1.0);

%% ---------------- Output struct ----------------
batt = struct();
batt.u_cmd_norm_sat = u_cmd_norm_sat;

batt.Voc   = Voc;
batt.Vbatt = Vbatt;
batt.Ibatt = Ibatt;

batt.u      = u;
batt.P_elec = P_elec;
batt.P_batt = P_batt;

batt.soc   = mem.soc;
batt.Vp    = mem.Vp;

batt.model     = model;
batt.regenMode = regenMode;
batt.etaDrv    = etaDrv;
batt.etaRegen  = etaRegen;

end

%% =================== Helper functions ===================

function Voc = ocv_from_soc(soc, P)
%OCV_FROM_SOC  Smooth OCV curve vs SOC for a ~12V Li-ion pack.
% Replace with a lookup table if you have your exact chemistry curve.

soc = clamp(soc, 0.0, 1.0);

Vnom   = safe_get(P, {'batt','V_nom'}, 12.0);
Vfull  = safe_get(P, {'batt','V_full'}, 1.08*Vnom);
Vempty = safe_get(P, {'batt','V_empty'}, 0.88*Vnom);

a = 0.65;
k = 10;
sig = 1 ./ (1 + exp(-k*(soc-0.5)));

Voc = Vempty + (Vfull - Vempty) * (a*soc + (1-a)*sig);
end

function v = safe_get(S, path, defaultVal)
%SAFE_GET  Safe nested getter
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
    if isempty(tmp)
        v = defaultVal;
    else
        v = tmp;
    end
catch
    v = defaultVal;
end
end

function y = clamp(x, lo, hi)
%CLAMP  Saturation helper
y = min(max(x, lo), hi);
end
