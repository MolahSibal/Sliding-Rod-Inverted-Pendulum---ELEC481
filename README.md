

# ELEC 481 — ECP Model 505 Inverted Pendulum

**Project:** Modelling and Control of a Sliding-Rod Inverted Pendulum  
**Course:** ELEC 481 — Linear Systems  
**Institution:** Concordia University  
**Department:** Mechanical, Industrial and Aerospace Engineering (MIAE)  
**Lab Section:** EE-J  
**Report:** ELEC 481 — Inverted Pendulum Report  
**Academic Term:** Fall 2026

---

## 1. Project Overview

This project investigates the modelling and control of the Educational Control Products (ECP) Model 505 sliding-rod inverted pendulum.

The objective is to apply classical and modern control theory to a mechanical system linearized about an equilibrium operating point.

The project includes mathematical modelling, open-loop analysis, controller design, state-space analysis, observer design, and numerical validation using MATLAB and Simulink.

The final laboratory report follows the 14 requirements outlined in the ELEC 481 General Guidelines for the Lab Work.

### 1.1 Current Progress

The following work has been started:

- Nonlinear equations of motion using Euler–Lagrange mechanics
- Linearization about the equilibrium
- Four-state state-space representation
- Open-loop transfer-function derivation
- Hardware scaling using the ECP manual
- Inner-loop PD controller design
- Outer-loop pole-placement controller design
- Prefilter gain derivation
- MATLAB and Simulink modelling
- Preliminary closed-loop stability and reference-tracking analysis

The remaining work includes canonical forms, system response analysis, robustness testing, modern state-feedback design, observer design, and the comparison of classical and modern controllers.

---

## 2. Mathematical Model

The physical system consists of a pendulum assembly with a sliding mass driven by a motor and belt mechanism.

The four state variables are:

| Variable | Description | Unit |
|---|---|---|
| theta | Pendulum angle | rad |
| theta_dot | Pendulum angular velocity | rad/s |
| x | Sliding rod displacement | m |
| x_dot | Sliding rod velocity | m/s |

The system input is the applied actuator force F(t), measured in newtons.

### 2.1 State-Space Representation

The continuous-time state vector is

$$\mathbf{x}(t)=\begin{bmatrix}\theta(t)&\dot{\theta}(t)&x(t)&\dot{x}(t)\end{bmatrix}^{T}$$

The linearized state-space representation is

$$\dot{\mathbf{x}}(t)=A\mathbf{x}(t)+Bu(t)$$

$$\mathbf{y}(t)=C\mathbf{x}(t)+Du(t)$$

where the input is

$$u(t)=F(t)$$

If the selected measured outputs are the pendulum angle and sliding rod displacement, the output vector is

$$\mathbf{y}(t)=\begin{bmatrix}\theta(t)\\x(t)\end{bmatrix}$$

The corresponding output matrices are

$$C=\begin{bmatrix}1&0&0&0\\0&0&1&0\end{bmatrix}$$

$$D=\begin{bmatrix}0\\0\end{bmatrix}$$

The full expressions for A and B are included in the Overleaf laboratory report.

**Note:** The current Overleaf draft uses an identity matrix for C. This should be corrected if the outputs are defined as theta and x. Alternatively, the identity matrix may be retained if the outputs are explicitly defined as all four states.

---

## 3. Equations of Motion

### 3.1 Nonlinear Equations

The nonlinear equations of motion are derived using the Euler–Lagrange formulation.

The modelling process involves:

1. Identifying the generalized coordinates.
2. Calculating the kinetic energy.
3. Calculating the potential energy.
4. Constructing the Lagrangian.
5. Applying the Euler–Lagrange equations.
6. Deriving the nonlinear equations of motion.

The corresponding derivations are documented in the Overleaf report.

### 3.2 Linearization

The nonlinear equations are linearized about the selected equilibrium configuration.

The linearized equations are used to derive the state-space representation and the open-loop transfer functions.

The validity of the linear model is restricted to sufficiently small perturbations around the chosen equilibrium.

---

## 4. Open-Loop Transfer Functions

Three transfer functions are obtained from the linearized equations of motion.

### 4.1 Pendulum Angle / Sliding Rod Position

$$G_2(s)=\frac{\Theta(s)}{X(s)}$$

This transfer function relates pendulum angle to sliding rod displacement.

It is derived from the complete linearized coupled equations.

Although the transfer function is second order, it is not the same as the rigid-body approximation used for the inner-loop PD design.

### 4.2 Pendulum Angle / Applied Force

$$G_{\theta F}(s)=\frac{\Theta(s)}{F(s)}$$

This transfer function describes the pendulum angular response to an applied actuator force.

### 4.3 Sliding Rod Position / Applied Force

$$G_1(s)=\frac{X(s)}{F(s)}$$

This is the complete fourth-order sliding-rod plant derived from the coupled linearized dynamics.

The symbolic expressions for all three transfer functions are provided in the Overleaf document.

### 4.4 Model Verification

The derived transfer functions should be checked against the state-space representation using MATLAB.

The following quantities should be compared:

- Open-loop poles
- Open-loop zeros
- Transfer-function coefficients
- Equilibrium behavior
- State-space and transfer-function responses

---

## 5. Physical Parameters

The MATLAB initialization script uses physical parameters based on the ECP Model 505 instructor's manual.

### 5.1 Fixed Parameters

| Parameter | Value | Description |
|---|---|---|
| m_1o | 0.103 kg | Sliding rod assembly mass |
| m_w1 | 0.110 kg | Donut weights |
| m_w2 | 1.000 kg | Balance mass |
| m_2o | 0.785 kg | Pendulum assembly mass |
| l_o | 0.330 m | Pendulum geometry |
| l_co | 0.0711 m | Reference centre-of-gravity parameter |
| l_w2 | -0.1084 m | Signed balance-mass distance |
| J_star_o | 0.0246 kg m² | Reference inertia |
| g | 9.807 m/s² | Gravitational acceleration |

### 5.2 Derived Parameters

The combined sliding rod and pendulum masses are

$$m_1=m_{1o}+m_{w1}$$

$$m_2=m_{2o}+m_{w2}$$

The pendulum centre-of-gravity parameter is

$$l_c=\frac{m_{w2}l_{w2}+m_{2o}l_{co}}{m_2}$$

The equivalent inertia is

$$J_{oe}=J_o^*+m_1l_o^2+m_{w2}l_{w2}^2$$

The reduced inertia is

$$J^*=J_{oe}-m_1l_o^2$$

These parameters are used in the state-space equations and transfer-function derivations.

### 5.3 Important Parameter Correction

The manufacturer's initialization script specifies

```matlab
l_w2 = -0.1084;
```

The negative sign follows the coordinate convention used for the balance mass located below the pivot.

An earlier version of the MATLAB model incorrectly used

```matlab
l_w2 = 0.1084;
```

This changes the calculated centre of gravity and consequently affects the pendulum dynamics.

After correcting the parameter, all dependent calculations must be updated, including the outer-loop transfer function, controller coefficients, and reference prefilter.

The manufacturer's sample values should ultimately be checked against the physical apparatus used in the laboratory.

---

## 6. Hardware Scaling

The ECP manual expresses physical variables in controller-scaled units.

According to Equations 6.1-5 through 6.1-7:

$$\theta_c=k_sk_a\theta$$

$$x_c=k_sk_xx$$

$$F=k_fF_c$$

where the subscript c denotes controller-scaled quantities.

### 6.1 Hardware Scaling Constants

| Factor | Value | Description |
|---|---|---|
| k_f | 0.0013 N/DAC increment | Actuator force conversion |
| k_x | 50200 counts/m | Sliding rod encoder |
| k_a | 2546 counts/rad | Pendulum angle encoder |
| k_s | 32 | Controller firmware scaling |

The combined hardware gain is

$$k_{hw}=k_xk_sk_f$$

### 6.2 Scaled Transfer Functions

The scaled force-to-position transfer function is

$$\frac{X_c(s)}{F_c(s)}=k_xk_sk_f\frac{X(s)}{F(s)}$$

The scaled angle-to-position transfer function is

$$\frac{\Theta_c(s)}{X_c(s)}=\frac{k_a}{k_x}\frac{\Theta(s)}{X(s)}$$

Notice that the firmware scaling factor cancels in the angle-to-position transfer function.

### 6.3 Simulink Implementation

Two implementations are possible:

1. Use physical transfer functions with separate hardware scaling Gain blocks.
2. Incorporate scaling directly into the transfer functions.

The same scaling factor must not be introduced twice.

The reference prefilter gain is part of the controller design and is not a hardware conversion factor.

---

## 7. Classical Control Design

The ECP manual uses successive loop closure to design the classical controller.

The control architecture consists of:

1. An inner sliding-rod position-control loop.
2. An outer pendulum-angle control loop.

### 7.1 Inner-Loop PD Controller

The inner loop regulates sliding rod position.

The ECP manual approximates the inner-loop plant as

$$G_{\mathrm{inner}}(s)=\frac{k_{hw}}{m^*s^2}$$

The effective mass is

$$m^*=\frac{m_1m_2^*}{m_1+m_2^*}$$

where

$$m_2^*=\frac{J^*}{l_o^2}$$

The PD controller is

$$C_{\mathrm{PD}}(s)=k_p+k_ds$$

The selected design specifications are

$$\omega_n=63\ \mathrm{rad/s}$$

$$\zeta=1$$

The PD gains are determined by matching the characteristic equation with the standard second-order polynomial.

The proportional gain is

$$k_p=\frac{m^*\omega_n^2}{k_{hw}}$$

The derivative gain is

$$k_d=\frac{2\zeta\omega_nm^*}{k_{hw}}$$

The manual uses this simplified second-order model for inner-loop controller design.

The complete fourth-order model is retained separately for validation.

### 7.2 Inner-Loop Stability Analysis

The complete physical plant is

$$G_{\mathrm{full}}(s)=\frac{X(s)}{F(s)}$$

The hardware-scaled open-loop transfer function is

$$L_{\mathrm{inner}}(s)=C_{\mathrm{PD}}(s)k_{hw}G_{\mathrm{full}}(s)$$

The corresponding closed-loop transfer function is

$$T_{\mathrm{inner}}(s)=\frac{L_{\mathrm{inner}}(s)}{1+L_{\mathrm{inner}}(s)}$$

The simplified and complete inner-loop responses should be compared to identify the effects of neglected dynamics.

The inner-loop controller is primarily responsible for sliding rod position regulation.

The outer controller is responsible for pendulum-angle stabilization.

### 7.3 Outer-Loop Pole-Placement Controller

The outer loop controls the pendulum angle by commanding the sliding rod position.

The outer-loop plant is

$$G_2(s)=\frac{\Theta_c(s)}{X_c(s)}$$

The controller polynomials are

$$R(s)=r_1s+r_0$$

$$S(s)=s_1s+s_0$$

The ECP manual assumes that the inner closed-loop transfer function can be approximated by unity gain:

$$\frac{X_c(s)}{E^*(s)}\approx1$$

Under this assumption, the outer-loop characteristic polynomial is

$$D_{cl}(s)=D_{ax}(s)R(s)+k^*N_{ax}(s)S(s)$$

The coefficients of R(s) and S(s) are obtained by equating the expanded characteristic polynomial with the desired polynomial.

The selected desired poles are

$$p_{1,2}=-\pi\pm j\pi$$

$$p_3=-3\pi$$

The corresponding desired characteristic polynomial is

$$D_{\mathrm{desired}}(s)=(s+\pi+j\pi)(s+\pi-j\pi)(s+3\pi)$$

All polynomial coefficients, including the highest-order coefficient, must be accounted for.

### 7.4 Reference Prefilter

The reference prefilter is used to achieve the desired steady-state tracking.

From Figure 6.2-3, the outer-loop command signal is

$$e^*(s)=\frac{k_{\mathrm{pfc}}}{R(s)}r_c(s)-\frac{S(s)}{R(s)}\theta_c(s)$$

Assuming unity gain for the inner loop, the prefilter becomes

$$k_{\mathrm{pfc}}=S(0)+\frac{R(0)}{G_2(0)}$$

The previously calculated prefilter gain was

$$k_{\mathrm{pfc}}=104.564652$$

**This historical value must be recalculated and verified using the corrected physical parameters.**

The complete nested-loop system must also be checked using the actual inner-loop dynamics rather than relying exclusively on the unity-gain approximation.

### 7.5 Complete Nested-Loop Stability

The outer-loop plant with the actual inner closed-loop dynamics is

$$P_{\mathrm{outer}}(s)=T_{\mathrm{inner}}(s)G_2(s)$$

The outer-loop controller is

$$C_{\mathrm{outer}}(s)=\frac{S(s)}{R(s)}$$

The complete outer-loop open-loop transfer function is

$$L_{\mathrm{outer}}(s)=C_{\mathrm{outer}}(s)P_{\mathrm{outer}}(s)$$

The closed-loop poles must be evaluated to determine stability.

The complete nested-loop response should be compared with the simplified controller-design model.

---

## 8. MATLAB and Simulink Workflow

### 8.1 MATLAB

The MATLAB initialization script is responsible for:

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

Control System Toolbox is required for many of these functions.

Symbolic calculations use Symbolic Math Toolbox.

### 8.2 Simulink Models

**ECP_SimplifiedInvertedPendulum.slx**

Used to investigate the simplified inner- and outer-loop controller designs from Section 6.2 of the ECP manual.

The unity-inner-loop approximation is useful for verifying the outer-loop pole-placement procedure.

**ECP_InvertedPendulum.slx**

Used to investigate the complete fourth-order linear plant with hardware scaling and nested feedback loops.

The complete model is used to determine whether the designed controllers stabilize the coupled linear dynamics.

### 8.3 Simulation Validation

The following quantities should be evaluated:

- Open-loop poles
- Closed-loop poles
- Stability
- Reference tracking
- Steady-state error
- Settling time
- Overshoot
- Control effort

A stable response from the idealized outer-loop model does not automatically prove stability of the complete fourth-order system.

The simulation should also verify that the input and output scaling conventions are consistent.

---

## 9. ELEC 481 Laboratory Requirements

The laboratory report must address all 14 tasks specified in the course guidelines.

### 9.1 Mathematical Modelling

- [ ] **1.** Starting with the input-output equations, derive the state and output equations.
- [ ] **2.** Find the transfer function of the open-loop system.
- [ ] **3.** Obtain the controllable, observable, and Jordan canonical forms.
- [ ] **4.** Obtain the impulse and step responses with arbitrary initial conditions.
- [ ] **5.** Plot the Bode plot of the uncompensated system and the root locus of the open-loop system.

### 9.2 Classical Control

- [ ] **6.** Design a lead-lag or PID controller to meet selected design specifications, including transient and steady-state characteristics.
- [ ] **7.** Obtain step, square-wave, and sinusoidal responses.
- [ ] **8.** Investigate robustness by introducing measurement noise and parameter variations or uncertainty.

### 9.3 Modern Control

- [ ] **9.** Design a full-state feedback controller to meet the specifications selected in Task 6.
- [ ] **10.** Obtain step, square-wave, and sinusoidal responses with arbitrary initial conditions and compare them with Task 7.
- [ ] **11.** Design full-order and reduced-order observers and obtain step and sinusoidal responses.
- [ ] **12.** Determine the observer and controller transfer functions and identify the resulting controller type.

### 9.4 Comparative Analysis

- [ ] **13.** Compare the advantages and disadvantages of classical and modern control design.
- [ ] **14.** Support the comparison through numerical MATLAB simulations.

---

## 10. Proposed Laboratory Report Structure

The final Overleaf report should follow the organization below.

### 10.1 Introduction

Introduce the ECP inverted pendulum, the control objectives, and the classical and modern methods investigated.

### 10.2 Mathematical Modelling

Present the nonlinear equations of motion, Euler–Lagrange derivation, operating point, and linearization.

### 10.3 State-Space and Transfer-Function Representations

Present the A, B, C, and D matrices and derive the open-loop transfer functions.

### 10.4 Canonical Forms

Determine the controllable, observable, and Jordan canonical forms.

### 10.5 Open-Loop Analysis

Discuss open-loop poles, stability, impulse response, step response, Bode plot, and root locus.

### 10.6 Classical Controller Design

Describe the selected compensator, controller gains, pole placement, prefilter design, and tracking performance.

### 10.7 Robustness Analysis

Investigate measurement noise and variations in plant parameters.

### 10.8 Modern Control Design

Present full-state feedback design, including pole placement or LQR where appropriate.

### 10.9 Observer Design

Develop full-order and reduced-order observers and evaluate their performance.

### 10.10 Comparison of Classical and Modern Control

Compare transient response, steady-state tracking, robustness, control effort, and implementation complexity.

### 10.11 Conclusions

Summarize the results and limitations of the selected controllers.

### 10.12 References and Appendices

Include detailed derivations, MATLAB scripts, supplementary figures, and numerical results.

---

## 11. Corrections Required in the Overleaf Draft

The current LaTeX report requires the following corrections and additions.

### 11.1 Formatting and Notation

- [ ] Change the running header from AERO 480 to ELEC 481.
- [ ] Replace "LQR for life" with a technical introduction.
- [ ] Reconcile the output vector with the state-space output matrix C.
- [ ] Explicitly define the feedthrough matrix D.
- [ ] Verify consistency between l_0, l_o, J_0, J_oe, and J_star.
- [ ] Correct typographical errors in the equations and explanations.
- [ ] Review the fourth-order transfer-function denominator for missing parentheses.

### 11.2 Mathematical Model

- [ ] Verify the nonlinear equations of motion.
- [ ] Verify the selected equilibrium and linearization.
- [ ] Correct the sign of l_w2 in the MATLAB initialization script.
- [ ] Correct the MATLAB comment about the donut weights.
- [ ] Recalculate the outer-loop coefficients.
- [ ] Recalculate the reference prefilter gain.
- [ ] Verify the complete model's stability.

### 11.3 Required Analysis

- [ ] Add canonical-form calculations.
- [ ] Add controllability and observability analysis.
- [ ] Add impulse and step responses.
- [ ] Add Bode and root-locus plots.
- [ ] Add transient and steady-state performance measurements.
- [ ] Add robustness simulations.
- [ ] Add state-feedback design.
- [ ] Add full-order and reduced-order observer designs.
- [ ] Add the classical versus modern control comparison.
- [ ] Update the abstract after completing the results.

---

## 12. Report Guidelines

The ELEC 481 guidelines require sufficient detail concerning:

- Design procedures
- Validation and verification
- Simulation results
- Evaluation and analysis

The main body should focus on observations, results, and discussion.

Detailed derivations, MATLAB scripts, and supplementary simulation results may be included in appendices to improve readability.

Only one report is submitted per group.

The submission deadline is the date of the final exam.

---

## 13. References

1. Educational Control Products, *Manual for Model 505 Inverted Pendulum*, Instructor's Edition, 1994.

2. Concordia University, *ELEC 481 General Guidelines for the Lab Work*.

3. ELEC 481 course lecture notes, Concordia University.
