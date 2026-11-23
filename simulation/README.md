# MATLAB simulation

The model couples wheel and base dynamics with motor current and drive limits. The default study uses the nominal 19:1 motor variant and modeled inertias of `6.6e-4 kg m²` for the wheel and `1.34e-3 kg m²` for the base.

Open the [MATLAB source folder](matlab/) in MATLAB and run one of these entry points.

| Command | Output |
| --- | --- |
| `RUN_TESTS_FIRST` | Summaries for an open-loop pulse, wheel-speed PI, and base-angle PD. The checks cover fault status, peak current, relative wheel speed, and energy consistency. |
| `RUN_OPEN_LOOP_COMPARE` | A summary table and figures for nominal 6:1, 10:1 and 19:1 gear variants under a common 0.20 normalized voltage command from 0.20 to 0.80 seconds. |
| `main` | The default simulation and plots. |

Model parameters are in [params_default.m](matlab/params_default.m). [rw_dynamics.m](matlab/rw_dynamics.m) defines the plant and [rw_controller.m](matlab/rw_controller.m) contains the control modes. [run_validation_cases.m](matlab/run_validation_cases.m) defines the numerical pass criteria. The checks use model data.

## Verification in MATLAB R2025a

| Mode | Peak absolute current (A) | Peak relative wheel speed (rad/s) | Final total energy residual (J) |
| --- | ---: | ---: | ---: |
| Open-loop pulse | 1.183 | 11.392 | −0.02618 |
| Wheel-speed PI | 0.497 | 31.669 | −0.02464 |
| Base-angle PD | 1.180 | 14.468 | −0.01318 |

All three cases passed the criteria in the validation script and completed without a latched fault.

The common voltage pulse produced the following simulated responses.

| Nominal gear class | Exact ratio | Peak absolute base angle (°) | Peak relative wheel speed (rpm) | Peak absolute current (A) |
| --- | ---: | ---: | ---: | ---: |
| 6:1 | 6.25 | 137.61 | 209.52 | 1.1834 |
| 10:1 | 10.00 | 126.26 | 178.10 | 1.1834 |
| 19:1 | 18.75 | 74.54 | 110.09 | 1.1834 |
