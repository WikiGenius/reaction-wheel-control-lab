function P = set_motor_variant(P, gearChoice)
%SET_MOTOR_VARIANT Apply a 12 V Pololu 37D encoder gearmotor variant.
%
% Supported gearChoice values:
%   6  -> 6.25:1 family  (reported as "6:1 class" in comparison plots)
%   10 -> 10:1 family
%   19 -> 18.75:1 family (reported as "19:1 class")
%
% The same brushed-DC motor model is kept across the family:
%   - same electrical resistance / inductance
%   - same shaft-side Kt and Ke
%   - same 5.5 A stall-current scale
%
% Only the gearbox ratio, output no-load speed, output CPR, and implied
% gearbox efficiency are adjusted to match the published output-side family
% data more tightly.

if nargin < 2
    error('set_motor_variant: gearChoice is required. Use 6, 10, or 19.');
end

if ~isstruct(P)
    error('set_motor_variant: first input must be parameter struct P.');
end

Istall = 5.5;                  % [A] family stall-current scale
Kt     = P.motor.Kt;           % [N*m/A] shaft-side torque constant

switch round(double(gearChoice))
    case 6
        exactRatio = 6.25;
        noLoadRPM  = 1600;
        tauStallOut = 3.0 * 0.0980665;  % 3.0 kg*cm -> N*m
        outputCPR  = 400;
        label = "6:1 class (6.25 exact)";
    case 10
        exactRatio = 10.0;
        noLoadRPM  = 1000;
        tauStallOut = 4.9 * 0.0980665;  % 4.9 kg*cm -> N*m
        outputCPR  = 640;
        label = "10:1";
    case 19
        exactRatio = 18.75;
        noLoadRPM  = 530;
        tauStallOut = 8.5 * 0.0980665;  % 8.5 kg*cm -> N*m
        outputCPR  = 1200;
        label = "19:1 class (18.75 exact)";
    otherwise
        error('set_motor_variant: unsupported gearChoice. Use 6, 10, or 19.');
end

eta = tauStallOut / max(exactRatio * Kt * Istall, 1e-12);
eta = min(max(eta, 0.30), 0.98);

P.gear.family = round(double(gearChoice));
P.gear.N = exactRatio;
P.gear.eta = eta;
P.gear.outputSpeedNoLoad_rpm = noLoadRPM;
P.gear.outputSpeedNoLoad_rad_s = noLoadRPM * 2*pi/60;
P.gear.outputCPR = outputCPR;
P.gear.tau_stall_out = tauStallOut;
P.gear.torque_per_amp_out = P.gear.eta * P.gear.N * P.motor.Kt;
P.gear.label = label;

% Conservative wheel-speed safety limit tied to no-load family speed.
if ~isfield(P,'limits') || ~isstruct(P.limits)
    P.limits = struct();
end
if ~isfield(P.limits,'wheel') || ~isstruct(P.limits.wheel)
    P.limits.wheel = struct();
end
P.limits.wheel.omega_max = 0.98 * P.gear.outputSpeedNoLoad_rad_s;

% Metadata
if ~isfield(P,'meta') || ~isstruct(P.meta)
    P.meta = struct();
end
P.meta.motor_family = "Pololu_37D_12V_encoder_family";
P.meta.motor_variant = label;
end
