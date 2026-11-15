# ELEC 481 — ECP Model 505 Inverted Pendulum

**Project:** Modelling and Control of a Sliding-Rod Inverted Pendulum

**Course:** ELEC 481 — Linear Systems  
**Institution:** Concordia University  
**Lab section:** EE-J

## Overview

This project develops a mathematical model and controllers for the Educational Control Products (ECP) Model 505 sliding-rod inverted pendulum. It compares classical control methods with state feedback and observer-based control using MATLAB and Simulink.

The modelling process starts with the nonlinear equations of motion, linearizes them about the upright equilibrium, and obtains state-space and transfer-function representations. Controller performance is assessed through stability analysis, reference tracking, initial-condition responses, and robustness tests.

**Status:** Work in progress. The model and classical controller designs are drafted. Simulation results and controller coefficients require verification using the corrected balance-mass position, `l_w2 = -0.1084 m`. Full-state feedback, observer designs, and the final controller comparison remain to be completed.

## System Model

The actuator applies a force to the sliding rod. Rod position is measured in metres, pendulum angle in radians, and actuator force in newtons. The continuous-time state vector and input are

```math
\mathbf{x}(t) =
\begin{bmatrix}
\theta(t) \\
\dot{\theta}(t) \\
x(t) \\
\dot{x}(t)
\end{bmatrix},
\qquad
u(t) = F(t).
```

Here, the bold symbol denotes the full state vector, while the scalar position denotes the sliding-rod displacement.

Linearization about the upright equilibrium gives

```math
\dot{\mathbf{x}}(t) = A\mathbf{x}(t) + Bu(t),
\qquad
\mathbf{y}(t) = C\mathbf{x}(t) + Du(t).
```

For measured outputs consisting of pendulum angle and rod position, the output matrices are

```math
\mathbf{y}(t) =
\begin{bmatrix}
\theta(t) \\
x(t)
\end{bmatrix},
\qquad
C =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0
\end{bmatrix},
\qquad
D =
\begin{bmatrix}
0 \\
0
\end{bmatrix}.
```

If all four states are exposed as simulation outputs, use a separate output definition with a four-by-four identity matrix for `C` and a four-by-one zero matrix for `D`.

### Transfer Functions

The linearized model provides the following relationships under zero initial conditions:

```math
G_x(s) = \frac{X(s)}{F(s)},
\qquad
G_{\theta x}(s) = \frac{\Theta(s)}{X(s)},
\qquad
G_{\theta F}(s) = \frac{\Theta(s)}{F(s)}.
```

The force-to-position plant is fourth order. The rod-position-to-pendulum-angle relationship is second order and is derived from the coupled linear equations. The force-to-angle transfer function describes the pendulum response to the actuator force.

Full expressions and derivations are documented in the project report.

## Physical Parameters

The current initialization uses the manufacturer's sample configuration.

| Parameter | Value | Description |
| --- | ---: | --- |
| `m_w1` | 0.110 kg | Sliding-rod donut-mass parameter |
| `m_w2` | 1.000 kg | Balance mass |
| `l_w2` | -0.1084 m | Signed balance-mass centre-of-gravity location |
| `l_o` | 0.330 m | Pendulum geometry parameter |
| `g` | 9.807 m/s² | Gravitational acceleration |

The balance-mass position is a signed quantity. Changing its sign changes the calculated centre of gravity and pendulum dynamics. Derived parameters, controller coefficients, and the reference prefilter must therefore be recomputed after correcting it. The sample configuration must also be checked against the laboratory apparatus.

## Hardware Scaling

The hardware interface converts physical variables into encoder and controller units.

| Parameter | Value | Description |
| --- | ---: | --- |
| `k_f` | 0.0013 N/DAC increment | Actuator force conversion |
| `k_x` | 50,200 counts/m | Sliding-rod encoder scale |
| `k_a` | 2,546 counts/rad | Pendulum-angle encoder scale |
| `k_s` | 32 | Controller firmware scale |

The physical and controller variables are related by

```math
\theta_c = k_s k_a \theta,
\qquad
x_c = k_s k_x x,
\qquad
F = k_f F_c.
```

The combined force-to-position hardware gain is

```math
k_{\mathrm{hw}} = k_s k_x k_f.
```

The scaled force-to-position transfer function is

```math
\frac{X_c(s)}{F_c(s)}
=
k_s k_x k_f \frac{X(s)}{F(s)}.
```

The scaled rod-position-to-pendulum-angle relationship is

```math
\frac{\Theta_c(s)}{X_c(s)}
=
\frac{k_a}{k_x} \frac{\Theta(s)}{X(s)}.
```

Use either physical-unit transfer functions with separate interface Gain blocks or transfer functions that already include hardware scaling. Apply each conversion once. Label signals consistently as physical units, raw encoder counts, or firmware-scaled controller units.

The reference prefilter belongs to the controller design and is separate from hardware unit conversion.

## Classical Control Design

### Inner-Loop PD Controller

The inner loop regulates sliding-rod position. The manual's approximate rigid-body plant and ideal PD controller are

```math
G_{\mathrm{inner,approx}}(s)
=
\frac{k_{\mathrm{hw}}}{m^{*}s^2},
\qquad
C_{\mathrm{PD}}(s) = k_p + k_d s.
```

Here, the effective mass is denoted by `m*`. Matching the closed-loop denominator to a standard second-order polynomial gives

```math
k_p = \frac{m^{*}\omega_n^2}{k_{\mathrm{hw}}},
\qquad
k_d = \frac{2\zeta\omega_n m^{*}}{k_{\mathrm{hw}}}.
```

The initial design uses a natural frequency of approximately 10 Hz and critical damping:

```math
\omega_n = 2\pi(10) \approx 62.83\ \mathrm{rad/s},
\qquad
\zeta = 1.
```

If a filtered derivative is used in Simulink, record its filter coefficient because its dynamics differ from those of the ideal PD controller.

### Outer-Loop Pendulum-Angle Controller

The outer loop regulates pendulum angle through the inner position loop. For the initial design, the inner loop is approximated as unity gain. Define the scaled outer plant as

```math
G_2(s) = \frac{\Theta_c(s)}{X_c(s)}
= \frac{N_2(s)}{D_2(s)},
```

where the numerator already includes the encoder-scale ratio. Define the controller polynomials as

```math
R(s) = r_1s + r_0,
\qquad
S(s) = s_1s + s_0.
```

For the control law

```math
R(s)X_c(s)
=
k_{\mathrm{pfc}}\Theta_{\mathrm{ref},c}(s)
-
S(s)\Theta_c(s),
```

the idealized closed-loop transfer function is

```math
\frac{\Theta_c(s)}{\Theta_{\mathrm{ref},c}(s)}
=
\frac{k_{\mathrm{pfc}}N_2(s)}
{D_2(s)R(s) + N_2(s)S(s)}.
```

The controller coefficients are obtained by matching every power of the denominator polynomial to the desired characteristic polynomial. For a stable closed loop with finite, nonzero plant DC gain, unity steady-state tracking is obtained with

```math
k_{\mathrm{pfc}}
=
S(0) + \frac{R(0)}{G_2(0)}.
```

Recompute this gain using the corrected physical parameters. Final validation must include the actual inner-loop dynamics and the complete interconnected model.

## Software and Models

MATLAB, Control System Toolbox, and Simulink are used for modelling and simulation. Symbolic Math Toolbox supports symbolic derivations and coefficient matching. The report is written in LaTeX using Overleaf.

| Model | Purpose |
| --- | --- |
| `ECP_SimplifiedInvertedPendulum.slx` | Study the approximate plant and inner/outer-loop controller design. |
| `ECP_InvertedPendulum.slx` | Evaluate the complete physical plant with the controller interfaces. |

### Running the Simulations

1. Run the MATLAB parameter and controller initialization script before opening or running a model.
2. Confirm `l_w2 = -0.1084` and recompute all dependent physical parameters and controller gains.
3. Run the simplified model and record whether the inner loop is represented explicitly or replaced by unity gain.
4. Run the full model with consistent hardware scaling.
5. Evaluate closed-loop poles, steady-state gain, reference tracking, and actuator demand.
6. Compare responses to step, square-wave, and sinusoidal references, then test nonzero initial conditions, measurement noise, and parameter variations.

If the plotted angle is in firmware-scaled controller units, convert it to degrees using

```math
\theta_{\mathrm{deg}}
=
\frac{180}{\pi k_s k_a}\theta_c.
```

## Validation and Report Tasks

The following checklist tracks the required analysis and documentation. Unchecked items have not yet been marked as fully verified.

- [ ] Derive and verify the nonlinear equations, equilibrium, and linear state/output model.
- [ ] Derive and verify the open-loop transfer functions.
- [ ] Obtain controllable, observable, and Jordan canonical forms.
- [ ] Analyse impulse and step responses, including nonzero initial conditions.
- [ ] Generate and interpret Bode plots and the uncompensated root locus.
- [ ] Design a classical compensator and document its specifications, transient response, and steady-state error.
- [ ] Evaluate classical-control responses to step, square-wave, and sinusoidal references.
- [ ] Test robustness to measurement noise and parameter variations.
- [ ] Design full-state feedback using the same performance specifications.
- [ ] Evaluate state-feedback responses and compare them with the classical design.
- [ ] Design and test full-order and reduced-order observers.
- [ ] Derive the observer-based controller transfer function and identify its structure.
- [ ] Compare the advantages and limitations of classical and modern control.
- [ ] Support conclusions with labelled MATLAB/Simulink plots and numerical performance measures.

Stability and actuator limits must be checked using the full model before hardware testing. A stable response from the simplified model alone does not establish stability of the complete apparatus.

## References

1. Educational Control Products, *Manual for Model 505 Inverted Pendulum*, Instructor's Edition, 1994. Chapters 5 and 6 cover modelling, hardware scaling, and controller experiments.
2. Concordia University, *ELEC 481 General Guidelines for the Lab Work*.
