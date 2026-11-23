function Lim = compute_sys_limits(P)
%COMPUTE_SYS_LIMITS Resolve conservative ("weakest-link") limits.
%
% Returns:
%   Lim.I_cont, Lim.I_peak
%   Lim.Ibatt_cont, Lim.Ibatt_peak
%   Lim.Vbus_max
%   Lim.omega_w_max   (used as relative wheel/output speed limit)

Lim = struct();
Lim.src = struct();

get = @(path,def) safe_get(P, path, def);
warnMissing = logical(get({'safety','warn_missing_limits'}, false));

%% ================= Motor-path current limits =================
cand_Icont = [];
lab_Icont  = {};

[cand_Icont, lab_Icont] = add_candidate(cand_Icont, lab_Icont, get({'limits','motor','I_cont'}, NaN), 'limits.motor.I_cont');
[cand_Icont, lab_Icont] = add_candidate(cand_Icont, lab_Icont, get({'limits','driver','I_cont'}, NaN), 'limits.driver.I_cont');
[cand_Icont, lab_Icont] = add_candidate(cand_Icont, lab_Icont, get({'limits','wire','I_cont'}, NaN),  'limits.wire.I_cont');
[cand_Icont, lab_Icont] = add_candidate(cand_Icont, lab_Icont, get({'limits','conn','I_cont'}, NaN),  'limits.conn.I_cont');
[cand_Icont, lab_Icont] = add_candidate(cand_Icont, lab_Icont, get({'ctrl','current','i_max'}, NaN),  'ctrl.current.i_max');

[Lim.I_cont, Lim.src.I_cont] = min_valid_with_source(cand_Icont, lab_Icont);
if ~isfinite(Lim.I_cont) || Lim.I_cont <= 0
    Lim.I_cont = 1.0;
    Lim.src.I_cont = 'fallback: 1.0A';
end

%% ================= Peak motor-path current =================
cand_Ipeak = [];
lab_Ipeak  = {};

[cand_Ipeak, lab_Ipeak] = add_candidate(cand_Ipeak, lab_Ipeak, get({'limits','motor','I_peak'}, NaN),  'limits.motor.I_peak');
[cand_Ipeak, lab_Ipeak] = add_candidate(cand_Ipeak, lab_Ipeak, get({'limits','driver','I_peak'}, NaN),  'limits.driver.I_peak');
[cand_Ipeak, lab_Ipeak] = add_candidate(cand_Ipeak, lab_Ipeak, get({'limits','wire','I_peak'}, NaN),   'limits.wire.I_peak');
[cand_Ipeak, lab_Ipeak] = add_candidate(cand_Ipeak, lab_Ipeak, get({'limits','conn','I_peak'}, NaN),   'limits.conn.I_peak');

[Lim.I_peak, Lim.src.I_peak] = min_valid_with_source(cand_Ipeak, lab_Ipeak);
if ~isfinite(Lim.I_peak) || Lim.I_peak <= 0
    Lim.I_peak = max(1.5*Lim.I_cont, Lim.I_cont);
    Lim.src.I_peak = 'fallback: 1.5*I_cont';
end
if Lim.I_peak < Lim.I_cont
    Lim.I_peak = Lim.I_cont;
    Lim.src.I_peak = 'clamped: I_peak >= I_cont';
end

%% ================= Bus voltage max =================
cand_V = [];
lab_V  = {};

[cand_V, lab_V] = add_candidate(cand_V, lab_V, get({'limits','driver','Vbus_max'}, NaN), 'limits.driver.Vbus_max');
[cand_V, lab_V] = add_candidate(cand_V, lab_V, get({'limits','batt','V_max'}, NaN),       'limits.batt.V_max');

[Lim.Vbus_max, Lim.src.Vbus_max] = min_valid_with_source(cand_V, lab_V);
if ~isfinite(Lim.Vbus_max) || Lim.Vbus_max <= 0
    Vfull = get({'batt','V_full'}, NaN);
    if isfinite(Vfull) && Vfull > 0
        Lim.Vbus_max = Vfull;
        Lim.src.Vbus_max = 'fallback: batt.V_full';
    else
        Vnom = get({'batt','V_nom'}, 12.0);
        Lim.Vbus_max = 1.10 * Vnom;
        Lim.src.Vbus_max = 'fallback: 1.10*batt.V_nom';
    end
end

%% ================= Relative wheel/output speed max =================
Lim.omega_w_max = get({'limits','wheel','omega_max'}, Inf);
Lim.src.omega_w_max = 'limits.wheel.omega_max';
if ~isfinite(Lim.omega_w_max) || Lim.omega_w_max <= 0
    Lim.omega_w_max = Inf;
    Lim.src.omega_w_max = 'fallback: Inf';
end
Lim.omega_max = Lim.omega_w_max;

%% ================= Battery-only current limits =================
Lim.Ibatt_cont = get({'limits','batt','I_cont'}, NaN);
Lim.Ibatt_peak = get({'limits','batt','I_peak'}, NaN);
if ~isfinite(Lim.Ibatt_cont) || Lim.Ibatt_cont <= 0, Lim.Ibatt_cont = NaN; end
if ~isfinite(Lim.Ibatt_peak) || Lim.Ibatt_peak <= 0, Lim.Ibatt_peak = NaN; end
Lim.src.Ibatt_cont = 'limits.batt.I_cont';
Lim.src.Ibatt_peak = 'limits.batt.I_peak';

%% ================= Report =================
Lim.meta.note = "Weakest-link limits; invalid candidates ignored; sources in Lim.src.*";
Lim.report.I_cont      = struct('value',Lim.I_cont,'src',Lim.src.I_cont,'candidates',{lab_Icont},'cand_values',cand_Icont);
Lim.report.I_peak      = struct('value',Lim.I_peak,'src',Lim.src.I_peak,'candidates',{lab_Ipeak},'cand_values',cand_Ipeak);
Lim.report.Vbus_max    = struct('value',Lim.Vbus_max,'src',Lim.src.Vbus_max,'candidates',{lab_V},'cand_values',cand_V);
Lim.report.omega_w_max = struct('value',Lim.omega_w_max,'src',Lim.src.omega_w_max);

if warnMissing
    warn_if_nan(get({'limits','motor','I_cont'}, NaN), 'limits.motor.I_cont');
    warn_if_nan(get({'limits','motor','I_peak'}, NaN), 'limits.motor.I_peak');
    warn_if_nan(get({'limits','batt','I_cont'},  NaN), 'limits.batt.I_cont');
    warn_if_nan(get({'limits','wheel','omega_max'}, NaN), 'limits.wheel.omega_max');
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
            v = defaultVal;
            return;
        end
    end
    if isempty(tmp), v = defaultVal; else, v = tmp; end
catch
    v = defaultVal;
end
end

function [vals, labels] = add_candidate(vals, labels, value, label)
vals(end+1,1) = value; %#ok<AGROW>
labels{end+1,1} = label; %#ok<AGROW>
end

function [m, src] = min_valid_with_source(vals, labels)
m = NaN;
src = 'none';
if isempty(vals), return; end
mask = isfinite(vals) & (vals > 0);
if any(mask)
    vv = vals(mask);
    idxAll = find(mask);
    [m, k] = min(vv);
    src = labels{idxAll(k)};
end
end

function warn_if_nan(v, name)
if ~(isfinite(v) && v > 0)
    warning('compute_sys_limits:MissingOrInvalid','Missing/invalid limit: %s (NaN/<=0).', name);
end
end
