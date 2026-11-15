
# ELEC 481 — ECP Model 505 Inverted Pendulum

**Project:** Modelling and Control of a Sliding-Rod Inverted Pendulum  
**Course:** ELEC 481 — Linear Systems  
**Institution:** Concordia University  
**Lab section:** EE-J  
**Report:** ELEC 481 — Inverted Pendulum Report (Overleaf, LaTeX)

## 1. Project Overview

This project models and controls the Educational Control Products (ECP) Model 505 sliding-rod inverted pendulum.

The objective is to apply and compare classical and modern control techniques to a nonlinear mechanical system linearized about its upright equilibrium.

The project includes mathematical modelling, open-loop analysis, controller design, state-space analysis, observer design, and numerical validation using MATLAB and Simulink.

The final laboratory report follows the 14 requirements outlined in the ELEC 481 General Guidelines for the Lab Work.

### Current Progress

The following work has been started:

- Nonlinear equations of motion using Euler–Lagrange mechanics
- Linearization about the upright equilibrium
- Four-state state-space representation
- Open-loop transfer-function derivation
- Hardware scaling using the ECP manual
- Inner-loop PD controller design
- Outer-loop pole-placement controller design
- Prefilter gain derivation
- MATLAB and Simulink modelling
- Preliminary closed-loop stability and reference-tracking analysis

The remaining work includes canonical forms, system response analysis, robustness testing, modern state-feedback design, observer design, and the comparison of classical and modern controllers.

## 2. Mathematical Model

The physical system consists of a pendulum mounted on a rotating assembly with a sliding mass whose position is controlled through a motor-driven belt.

The state variables are:

- theta: Pendulum angle (rad)
- theta_dot: Pendulum angular velocity (rad/s)
- x: Sliding rod position (m)
- x_dot: Sliding rod velocity (m/s)

The system input is the applied actuator force F(t), measured in newtons.

The continuous-time state vector is

$$
\mathbf{x}(t)=
\begin{bmatrix}
\theta & \dot{\theta} & x & \dot{x}
\end{bmatrix}^{T}
$$

The linear state-space model is

$$
\dot{\mathbf{x}}(t)=A\mathbf{x}(t)+Bu(t)
$$

$$
\mathbf{y}(t)=C\mathbf{x}(t)+Du(t)
$$

with

$$
u(t)=F(t)
$$

If the selected outputs are the pendulum angle and sliding rod position,

$$
\mathbf{y}(t)=
\begin{bmatrix}
\theta(t)\\
x(t)
\end{bmatrix}
$$

then the appropriate output matrices are

$$
C=
\begin{bmatrix}
1&0&0&0\\
0&0&1&0
\end{bmatrix},
\qquad
D=
\begin{bmatrix}
0\\
0
\end{bmatrix}.
$$

The existing Overleaf document currently uses an identity matrix for C. This must be corrected or justified by redefining the outputs as all four states.

## 3. Open-Loop Transfer Functions

The linearized equations of motion are used to derive three transfer functions.

### 3.1 Pendulum Angle / Sliding Rod Position

$$
G_2(s)=\frac{\Theta(s)}{X(s)}
$$

This transfer function relates pendulum angle to sliding rod displacement.

It is derived from the complete linearized coupled dynamics and is second order.

### 3.2 Pendulum Angle / Applied Force

$$
G_{\theta F}(s)=\frac{\Theta(s)}{F(s)}
$$

This describes the pendulum response to the applied actuator force.

### 3.3 Sliding Rod Position / Applied Force

$$
G_1(s)=\frac{X(s)}{F(s)}
$$

This is the complete fourth-order physical plant.

The full symbolic equations and derivations are documented in the Overleaf report.

## 4. Physical Parameters

The MATLAB initialization script uses the ECP manufacturer's sample parameters.

| Parameter | Value | Description |
|---|---:|---|
| m_1o | 0.103 kg | Sliding rod assembly mass |
| m_w1 | 0.110 kg | Donut weights |
| m_w2 | 1.000 kg | Balance mass |
| m_2o | 0.785 kg | Pendulum assembly mass excluding specified masses |
| l_o | 0.330 m | Pendulum geometry |
| l_co | 0.0711 m | Initial centre-of-gravity parameter |
| l_w2 | -0.1084 m | Signed balance-mass distance |
| J_star_o | 0.0246 kg m² | Reference inertia |
| g | 9.807 m/s² | Gravitational acceleration |

### Important Correction

The ECP manufacturer's initialization script specifies

```matlab
l_w2 = -0.1084;
```

The negative sign follows the coordinate convention used for the balance mass.

The original MATLAB model incorrectly used a positive value.

Correcting this changes the calculated centre of gravity, the pendulum dynamics, and the outer-loop controller coefficients.

All dependent calculations must be updated after changing this parameter.

The sample parameters should also be checked against the actual hardware configuration.

## 5. Hardware Scaling

The ECP manual expresses physical variables in controller-scaled units.

According to Equations 6.1-5 through 6.1-7,

$$
\theta_c=k_sk_a\theta
$$

$$
x_c=k_sk_xx
$$

$$
F=k_fF_c
$$

where the subscript c denotes controller-scaled quantities.

The hardware scaling factors are:

| Factor | Value | Description |
|---|---:|---|
| k_f | 0.0013 N/DAC increment | Actuator force conversion |
| k_x | 50200 counts/m | Sliding rod encoder |
| k_a | 2546 counts/rad | Pendulum angle encoder |
| k_s | 32 | Controller firmware scaling |

The combined hardware gain is

$$
k_{hw}=k_xk_sk_f.
$$

The scaled force-to-position transfer function is

$$
\frac{X_c(s)}{F_c(s)}
=
k_xk_sk_f\frac{X(s)}{F(s)}.
$$

The scaled angle-to-position transfer function is

$$
\frac{\Theta_c(s)}{X_c(s)}
=
\frac{k_a}{k_x}\frac{\Theta(s)}{X(s)}.
$$

### Simulink Convention

Two implementations are possible:

1. Use physical transfer functions with separate hardware scaling Gain blocks.
2. Incorporate scaling directly into the transfer functions.

The same scaling factor must not be introduced twice.

The reference prefilter gain is part of the controller design and is not a hardware conversion factor.

## 6. Classical Control Design

### 6.1 Inner-Loop PD Controller

The inner loop regulates sliding rod position.

The ECP manual approximates the inner-loop plant as

$$
G_{\mathrm{inner}}(s)
=
\frac{k_{hw}}{m^*s^2}.
$$

The PD controller is

$$
C_{\mathrm{PD}}(s)=k_p+k_ds.
$$

The selected design specifications are

$$
\omega_n=63\ \mathrm{rad/s}
$$

$$
\zeta=1.
$$

The PD gains are determined by coefficient matching:

$$
k_p=\frac{m^*\omega_n^2}{k_{hw}}
$$

$$
k_d=\frac{2\zeta\omega_nm^*}{k_{hw}}.
$$

The manual uses this simplified second-order representation for inner-loop controller design.

The complete fourth-order transfer function is retained separately for more representative validation.

### 6.2 Outer-Loop Pole-Placement Controller

The outer loop controls pendulum angle by commanding the sliding rod position.

The outer-loop plant is

$$
G_2(s)=\frac{\Theta_c(s)}{X_c(s)}.
$$

The controller polynomials are

$$
R(s)=r_1s+r_0
$$

$$
S(s)=s_1s+s_0.
$$

The ECP manual assumes that the inner closed-loop transfer function can be approximated by unity gain:

$$
\frac{X_c(s)}{E^*(s)}\approx1.
$$

The outer-loop characteristic polynomial is then

$$
D_{cl}(s)
=
D_{ax}(s)R(s)+k^*N_{ax}(s)S(s).
$$

The coefficients of R(s) and S(s) are obtained by matching the expanded characteristic polynomial with the desired polynomial.

All polynomial coefficients, including the highest-order coefficient, must be accounted for.

### 6.3 Reference Prefilter

The prefilter gain is used to achieve the desired steady-state reference tracking.

From the outer-loop block diagram,

$$
e^*(s)
=
\frac{k_{\mathrm{pfc}}}{R(s)}r_c(s)
-
\frac{S(s)}{R(s)}\theta_c(s).
$$

Assuming unity gain for the inner loop, the prefilter becomes

$$
k_{\mathrm{pfc}}
=
S(0)+\frac{R(0)}{G_2(0)}.
$$

The value previously calculated in MATLAB was

$$
k_{\mathrm{pfc}}=104.564652.
$$

**This value must be recalculated and verified after correcting the balance-mass sign.**

The complete nested-loop system must also be checked using the actual inner-loop dynamics rather than relying exclusively on the unity-gain approximation.

## 7. MATLAB and Simulink Workflow

### MATLAB

The MATLAB script is responsible for:

1. Initializing physical parameters.
2. Calculating derived system parameters.
3. Constructing open-loop transfer functions.
4. Determining PD controller gains.
5. Determining outer-loop controller coefficients.
6. Calculating the reference prefilter.
7. Evaluating poles and closed-loop stability.
8. Generating root-locus and frequency-response plots.

Relevant MATLAB functions include:

```matlab
tf()
feedback()
pole()
zero()
isstable()
rlocus()
bode()
step()
dcgain()
ss()
eig()
rank()
ctrb()
obsv()
place()
lqr()
```

Some functions require Control System Toolbox.

Symbolic derivations use Symbolic Math Toolbox.

### Simulink Models

**ECP_SimplifiedInvertedPendulum.slx**

Used to investigate the simplified inner- and outer-loop controller designs from Section 6.2 of the ECP manual.

The unity-inner-loop approximation is useful for verifying the outer-loop pole-placement procedure.

**ECP_InvertedPendulum.slx**

Used for the complete fourth-order linear plant with the hardware scaling gains and nested feedback loops.

The complete model is used to investigate whether the designed controllers stabilize the original coupled linear dynamics.

### Validation

The following quantities should be checked:

- Open-loop poles
- Closed-loop poles
- Stability
- Reference tracking
- Steady-state error
- Settling time
- Overshoot
- Control effort

A stable idealized outer-loop response does not automatically demonstrate stability of the complete fourth-order system.

## 8. ELEC 481 Laboratory Requirements

The laboratory report must address all 14 tasks specified in the course guidelines.

### Modelling and Open-Loop Analysis

- [ ] 1. Derive the state and output equations from the input-output equations.
- [ ] 2. Find the open-loop transfer function.
- [ ] 3. Obtain the controllable, observable, and Jordan canonical forms.
- [ ] 4. Determine impulse and step responses with arbitrary initial conditions.
- [ ] 5. Plot the uncompensated Bode plot and open-loop root locus.

### Classical Control

- [ ] 6. Design a lead-lag or PID controller to meet selected transient and steady-state specifications.
- [ ] 7. Obtain step, square-wave, and sinusoidal responses.
- [ ] 8. Analyze robustness by introducing noise and parameter uncertainties.

### Modern Control

- [ ] 9. Design a full-state feedback controller meeting the specifications from Task 6.
- [ ] 10. Obtain step, square-wave, and sinusoidal responses with arbitrary initial conditions and compare them with Task 7.
- [ ] 11. Design full-order and reduced-order observers, including step and sinusoidal responses.
- [ ] 12. Determine the observer/controller transfer function and identify the controller type.

### Comparative Analysis

- [ ] 13. Compare the advantages and disadvantages of classical and modern control approaches.
- [ ] 14. Support the comparison with numerical MATLAB simulations.

## 9. Proposed Laboratory Report Structure

The final Overleaf document should follow this organization:

### 1. Introduction

Introduce the ECP inverted pendulum, the control objectives, and the classical and modern methods investigated.

### 2. Mathematical Modelling

Present the nonlinear equations of motion, Euler–Lagrange derivation, operating point, and linearization.

### 3. State-Space and Transfer-Function Representations

Present the A, B, C, and D matrices and derive the open-loop transfer functions.

### 4. Canonical Forms

Determine the controllable, observable, and Jordan canonical forms.

### 5. Open-Loop Analysis

Discuss poles, stability, impulse response, step response, Bode plot, and root locus.

### 6. Classical Controller Design

Describe the selected compensator, controller gains, pole placement, prefilter design, and tracking performance.

### 7. Robustness Analysis

Investigate measurement noise and variations in plant parameters.

### 8. Modern Control Design

Present full-state feedback design, including pole placement or LQR where appropriate.

### 9. Observer Design

Develop full-order and reduced-order observers and evaluate their performance.

### 10. Comparison of Classical and Modern Control

Compare transient response, steady-state tracking, robustness, control effort, and implementation complexity.

### 11. Conclusions

Summarize the results and limitations of the selected controllers.

### 12. References and Appendices

Include detailed derivations, MATLAB scripts, supplementary figures, and numerical results.

## 10. Corrections Required in the Overleaf Draft

The current LaTeX report requires the following corrections and additions:

- [ ] Change the running header from AERO 480 to ELEC 481.
- [ ] Replace "LQR for life" with a technical introduction.
- [ ] Reconcile the output vector with the state-space output matrix C.
- [ ] Explicitly define the feedthrough matrix D.
- [ ] Verify consistency between l_0, l_o, J_0, J_oe, and J_star.
- [ ] Review the nonlinear equations and the specified upright equilibrium.
- [ ] Check the fourth-order transfer-function denominator for LaTeX syntax errors.
- [ ] Correct the MATLAB comment about donut weights.
- [ ] Recalculate outer-loop coefficients using negative l_w2.
- [ ] Recalculate the reference prefilter gain.
- [ ] Add canonical-form calculations.
- [ ] Add open-loop and closed-loop response plots.
- [ ] Add Bode and root-locus analysis.
- [ ] Add robustness simulations.
- [ ] Add state-feedback and observer designs.
- [ ] Add the classical versus modern control comparison.
- [ ] Update the abstract after completing the results.

## 11. Report Guidelines

The ELEC 481 guidelines specify that the report should provide sufficient detail concerning:

- Design procedures
- Validation and verification
- Simulation results
- Evaluation and analysis

The main body should focus on observations, results, and discussion.

Detailed calculations and simulation material may be included in appendices to improve readability.

Only one report is submitted per group.

The submission deadline is the date of the final exam.

## 12. References

1. Educational Control Products, *Manual for Model 505 Inverted Pendulum*, Instructor's Edition, 1994.

2. Concordia University, *ELEC 481 General Guidelines for the Lab Work*.

3. ELEC 481 course lecture notes, Concordia University.
