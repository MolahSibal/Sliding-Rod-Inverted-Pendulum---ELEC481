clc;clear;
syms d2 d1 d0 r1 r0 n2 n1 n0 s1 s0 s real;
D_cl = (d2*s^2+d1*s+d0)*(r1*s+r0)+(n2*s^2+n1*s+n0)*(s1*s+s0);
D_cl = collect(expand(D_cl),s);
disp(D_cl);
expr = (s+pi+i*pi)*(s+pi-i*pi)*(s+3*pi);
expr = collect(expand(expr),s);
disp(expr);