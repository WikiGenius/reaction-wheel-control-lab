function E = energy_audit(S, P)
%ENERGY_AUDIT Battery/electrical/mechanical consistency audit.
%
% This audit separates three levels:
%   1) electrical residual inside the motor electrical model
%   2) mechanical residual inside the rigid-body dynamics
%   3) total battery-boundary residual including driver + gear losses
%
% Notes
% -----
% - The motor/gear model uses a shaft-side electrical model and an output-side
%   torque model with gearbox efficiency eta. Therefore the audit includes an
%   explicit positive gearbox-loss estimate.
% - Instantaneous residual spikes can occur at command discontinuities and from
%   numerical differentiation. End-drift is the more trustworthy summary metric.

 t = S.t(:);
 x = S.x;
 sig = S.sig;

 if numel(t) < 2
     error('energy_audit: time vector must contain at least 2 samples.');
 end

 dt = [diff(t); t(end)-t(end-1)];
 dt(isnan(dt) | dt<=0) = mean(diff(t));

 omega_b = x(:,2);
 omega_w = x(:,4);
 i       = x(:,5);
 omega_rel = omega_w - omega_b;

 u     = fetch(sig,'u',nan(size(t)));
 Vbatt = fetch(sig,'Vbatt',nan(size(t)));
 Ibatt = fetch(sig,'Ibatt',nan(size(t)));
 tau_w = fetch(sig,'tau_w',nan(size(t)));

 tau_ext_b = fetch(sig,'tau_ext_b',zeros(size(t)));
 tau_ext_w = fetch(sig,'tau_ext_w',zeros(size(t)));

 % Electrical powers
 P_batt = Vbatt .* Ibatt;
 P_elec = u .* i;

 R = P.motor.R;
 L = getfield_def(P.motor,'L',0);
 P_cu = (i.^2) * R;
 E_L  = 0.5 * L .* (i.^2);
 dE_L_dt = gradient(E_L, t);

 Ke = P.motor.Ke;
 N  = P.gear.N;
 eta = getfield_def(P.gear,'eta',1.0);
 omega_m = N * omega_rel;
 P_emf = Ke .* omega_m .* i;

 % Mechanical energies
 Jb  = P.base.Jb;
 Jw  = P.wheel.Jw;
 Jeq = getfield_def(P.motor,'Jeq_out',0.0);
 E_K_base   = 0.5*Jb.*(omega_b.^2);
 E_K_wheel  = 0.5*Jw.*(omega_w.^2);
 E_K_relact = 0.5*Jeq.*(omega_rel.^2);
 E_K = E_K_base + E_K_wheel + E_K_relact;
 dE_K_dt = gradient(E_K, t);

 % Internal actuator power exchange at the output side
 P_mech_exchange = tau_w .* omega_rel;

 % External mechanical power into the simulated rigid-body system
 P_ext = tau_ext_b .* omega_b + tau_ext_w .* omega_w;

 % Dissipation logged by the plant
 P_fric_b  = fetch(sig,'P_fric_b',zeros(size(t)));
 P_fric_w  = fetch(sig,'P_fric_w',zeros(size(t)));
 P_brake_b = fetch(sig,'P_brake_b',zeros(size(t)));
 P_brake_w = fetch(sig,'P_brake_w',zeros(size(t)));
 P_wire    = fetch(sig,'P_wire',zeros(size(t)));
 P_mech_diss = P_fric_b + P_fric_w + P_brake_b + P_brake_w + P_wire;

 % Motor electrical-side residual
 res_elec = P_elec - (P_cu + dE_L_dt + P_emf);

 % Mechanical residual from the rigid-body model directly
 res_mech = dE_K_dt - (P_mech_exchange + P_ext - P_mech_diss);

 % Driver / bus-side dissipation.
 % Positive in motoring, allowed regen, and clamped-regen cases.
 P_drv_loss = abs(P_batt - P_elec);

 % Gear loss estimate from shaft-side electromechanical power down to the
 % output-side exchange power. Keep it non-negative.
 P_gear_loss = max(abs(P_emf) - abs(P_mech_exchange), 0);

 % Total battery-boundary residual.
 res_total = P_batt + P_ext - (P_drv_loss + P_cu + dE_L_dt + P_gear_loss + P_mech_diss + dE_K_dt);

 E_drift_elec  = cumsum(res_elec .* dt);
 E_drift_mech  = cumsum(res_mech .* dt);
 E_drift_total = cumsum(res_total .* dt);

 E = struct();
 E.t = t;
 E.P_batt = P_batt;
 E.P_elec = P_elec;
 E.P_cu   = P_cu;
 E.P_emf  = P_emf;
 E.P_mech_exchange = P_mech_exchange;
 E.P_ext  = P_ext;
 E.P_mech_diss = P_mech_diss;
 E.P_drv_loss = P_drv_loss;
 E.P_gear_loss = P_gear_loss;
 E.E_L = E_L;
 E.E_K_base = E_K_base;
 E.E_K_wheel = E_K_wheel;
 E.E_K_relact = E_K_relact;
 E.E_K = E_K;
 E.res_elec = res_elec;
 E.res_mech = res_mech;
 E.res_total = res_total;
 E.E_drift_elec = E_drift_elec;
 E.E_drift_mech = E_drift_mech;
 E.E_drift_total = E_drift_total;

 if safe_get_plot(P)
     fprintf('\n=== Energy audit ===\n');
     fprintf('max |res_elec|  = %.3g W\n', max(abs(res_elec(~isnan(res_elec)))));
     fprintf('max |res_mech|  = %.3g W\n', max(abs(res_mech(~isnan(res_mech)))));
     fprintf('max |res_total| = %.3g W\n', max(abs(res_total(~isnan(res_total)))));
     fprintf('end drift total = %.3g J\n', E_drift_total(end));
     fprintf('end drift mech  = %.3g J\n', E_drift_mech(end));
     fprintf('end drift elec  = %.3g J\n', E_drift_elec(end));
 end
end

function tf = safe_get_plot(P)
tf = true;
try
    if isfield(P,'sim') && isfield(P.sim,'doAuditPrint')
        tf = logical(P.sim.doAuditPrint);
    end
catch
    tf = true;
end
end

function v = fetch(sig, field, defaultVal)
if isstruct(sig) && isfield(sig, field) && ~isempty(sig.(field))
    tmp = sig.(field);
    n = numel(defaultVal);
    if isscalar(tmp), v = repmat(tmp,n,1); return; end
    if numel(tmp)==n, v = tmp(:); return; end
    v = defaultVal;
else
    v = defaultVal;
end
end

function v = getfield_def(S, field, defaultVal)
v = defaultVal;
if isstruct(S) && isfield(S,field) && ~isempty(S.(field))
    v = S.(field);
end
end
