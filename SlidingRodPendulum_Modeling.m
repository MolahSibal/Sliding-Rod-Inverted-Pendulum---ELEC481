%% ELEC 481 - Sliding Rod Inverted Pendulum
clc;clear;close all;
s=tf('s');
%% List of fixed physical parameters
l_o = 0.330;        %Length of pendulum rod from pivot to the sliding rod T section
m_1o = 0.103;       %Mass of the sliding rod with belt, belt clamps, and rubber end guards
m_w1 = 0.110;       %This is set to zero since "donut" weights aren't used
m_w2 = 1.000;       %Mass of brass balance weight
m_2o = 0.785 ;      %Mass of the complete moving assembly minus m1 and m_w2
l_co = 0.0711;      %Position of cg of the complete pendulum with the sliding rod and balance weight removed
J_star_o = 0.0246;  %Inertia evaluated when m_w2 = 0;
l_w2 = 0.1084;      %Signed distance from pivot to c.g of balance mass
g = 9.807;
k_f = 0.0013;       %Combined DAC-amplifier-motor-pulley gain
k_x = 50200;        %Sliding rod encoder
k_a = 2546;         %Pendulum angle encoder
k_s = 32;           %DSP controller firmware scale factor
%% List of derived physical parameters
m_1 = m_1o+m_w1;    %Mass of complete sliding rod including all attached elements
m_2 = m_2o+m_w2;    %Mass of the complete assembly minus m1
l_c = (m_w2*l_w2+m_2o*l_co)/(m_2);
J_oe = J_star_o + m_1*l_o^2 + m_w2*l_w2^2;
J_star = J_oe-m_1*l_o^2;
%% PD Control of the Sliding Rod
m_2_star = J_star/(l_o^2);
m_star = (m_1*m_2_star)/(m_1+m_2_star);
k_hw = k_x*k_s*k_f;
zeta = 1;
wn = 63;
kd = 2*zeta*wn*m_star/k_hw;
kp = m_star*wn^2/k_hw;
C = kp+kd*s;
num_g1 = k_hw;
den_g1 = [m_star 0 0];
G1 = tf(num_g1,den_g1);
figure;
rlocus(C*G1);
fprintf('The value of the Kp gain is:  ');
disp(kp);
fprintf('The value of the Kd gain is: ');
disp(kd);
%% Outer Loop for theta(s)/x(s)
k_star = (k_a*m_1*l_o)/(k_x*J_oe);
% 1. Numerical Model for Control System Toolbox
num_g2 = [-1 0 g/l_o];
den_g2 = [1 0 -(m_1*l_o+m_2*l_c)*g/J_oe];
G2 = k_star * tf(num_g2, den_g2);
syms r_1 r_0 s_1 s_0 s_sym;
num_g2_sym = -s_sym^2 + g/l_o;
den_g2_sym = s_sym^2 - (m_1*l_o + m_2*l_c)*g/J_oe;
R_s = r_1*s_sym + r_0;
S_s = s_1*s_sym + s_0;
%Current characteristic polynomial
D_cl = expand(den_g2_sym*R_s + k_star*num_g2_sym*S_s);
disp(D_cl);
%Desired characteristic polynomial
D_cl_desired = expand((s_sym+pi+pi*1i)*(s_sym+pi-pi*1i)*(s_sym+3*pi));
disp(D_cl_desired);
% Extract ALL coefficients, including s^3
coeffs_cl = coeffs(D_cl, s_sym, 'All');
coeffs_des = coeffs(D_cl_desired, s_sym, 'All');
% Match all four coefficients
eqns = coeffs_cl == coeffs_des;
% Solve for the four unknowns
sol = solve(eqns, [r_1, r_0, s_1, s_0]);
r_1_val = double(sol.r_1);
r_0_val = double(sol.r_0);
s_1_val = double(sol.s_1);
s_0_val = double(sol.s_0);
% Numerical validation
s = tf('s');
R_num = r_1_val*s + r_0_val;
S_num = s_1_val*s + s_0_val;
G2_num = k_star*tf(num_g2,den_g2);
D_cl_numeric = (den_g2*s*R_num + k_star*num_g2*s*S_num);
T_cl = feedback(G2_num*S_num/R_num,1);
disp('Actual closed-loop poles:')
disp(pole(T_cl))
disp('Desired closed-loop poles:')
disp([-pi+1i*pi; -pi-1i*pi; -3*pi])
