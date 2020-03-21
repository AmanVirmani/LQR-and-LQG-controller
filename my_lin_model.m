function [s_dot,A,B] = my_lin_model(M,m1,m2,l1,l2,g);
%% nonlinear model
syms a b c d e f F x1
s = [a,b,c,d,e,f];

x = s(1);
x_dot = s(2);
t1 = s(3);
t1_dot = s(4);
t2 = s(5);
t2_dot = s(6);

%s_dot = zeros(6,1);
s_dot(1) = x_dot;
s_dot(2) = (F - m1*g*sin(2*t1)/2 - m2*g*sin(2*t2)/2 - m1*l1*sin(t1)*(t1_dot)^2 - m2*l2*sin(t2)*(t2_dot)^2)/(M + m1*(sin(t1)^2) + m2*(sin(t2)^2));
s_dot(3) = t1_dot;
s_dot(4) = (1/l1)*(cos(t1)*(F - m1*g*sin(2*t1)/2 - m2*g*sin(2*t2)/2 - m1*l1*sin(t1)*(t1_dot)^2 - m2*l2*sin(t2)*(t2_dot)^2)/(M + m1*(sin(t1)^2) + m2*(sin(t2)^2)) - g*sin(t1));
s_dot(5) = t2_dot;
s_dot(6) = (1/l2)*(cos(t2)*(F - m1*g*sin(2*t1)/2 - m2*g*sin(2*t2)/2  - m1*l1*sin(t1)*(t1_dot)^2 - m2*l2*sin(t2)*(t2_dot)^2)/(M + m1*(sin(t1)^2) + m2*(sin(t2)^2)) - g*sin(t2));

f1x1 = diff(s_dot(2),x1);
f1t1 = diff(s_dot(2),t1);
f1t2 = diff(s_dot(2),t2);
f2x1 = diff(s_dot(4),x1);
f2t1 = diff(s_dot(4),t1);
f2t2 = diff(s_dot(4),t2);
f3x1 = diff(s_dot(6),x1);
f3t1 = diff(s_dot(6),t1);
f3t2 = diff(s_dot(6),t2);

f2F = diff(s_dot(2),F);
f4F = diff(s_dot(4),F);
f6F = diff(s_dot(6),F);

A11 = subs(f1x1, {c,d,e,f}, {0,0,0,0});
A12 = subs(f1t1, {c,d,e,f}, {0,0,0,0});
A13 = subs(f1t2, {c,d,e,f}, {0,0,0,0});
A21 = subs(f2x1, {c,d,e,f}, {0,0,0,0});
A22 = subs(f2t1, {c,d,e,f}, {0,0,0,0});
A23 = subs(f2t2, {c,d,e,f}, {0,0,0,0});
A31 = subs(f3x1, {c,d,e,f}, {0,0,0,0});
A32 = subs(f3t1, {c,d,e,f}, {0,0,0,0});
A33 = subs(f3t2, {c,d,e,f}, {0,0,0,0});

b1 = subs(f2F,{c,d,e,f},{0,0,0,0}); 
b2 = subs(f4F,{c,d,e,f},{0,0,0,0});
b3 = subs(f6F,{c,d,e,f},{0,0,0,0});

%% linearised model

A = double([0 1 0 0 0 0;A11 0 A12 0 A13 0;0 0 0 1 0 0;A21 0 A22 0 A23 0;0 0 0 0 0 1;A31 0 A32 0 A33 0]);
B = double([0;b1;0;b2;0;b3]);
