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
num_g = k_hw;
den_g = [m_star 0 0];
G = tf(num_g,den_g);

figure;
rlocus(C*G);

disp(kp);
disp(kd);
%Validate the previous results and document in on Overleaf
