function s_dot = my_nonlinear(s,t,M,m1,m2,l1,l2,g,F) 
x = s(1);
x_dot = s(2);
t1 = s(3);
t1_dot = s(4);
t2 = s(5);
t2_dot = s(6);
s_dot=zeros(6,1);
s_dot(1) = x_dot;
s_dot(2) = (F-m1*(g*sin(t1)*cos(t1)+l1*sin(t1)*t1_dot^2)-m2*(g*sin(t2)*cos(t2)+l2*sin(t2)*t2_dot^2))/(M+m1*sin(t1)^2+m2*sin(t2)^2);
s_dot(3) = t1_dot;
s_dot(4) = (cos(t1)/l1)*((F-m1*(g*sin(t1)*cos(t1)+l1*sin(t1)*t1_dot^2)-m2*(g*sin(t2)*cos(t2)+l2*sin(t2)*t2_dot^2))/(M+m1*sin(t1)^2+m2*sin(t2)^2))-(g*sin(t1)/l1);
s_dot(5) = t2_dot;
s_dot(6) = (cos(t2)/l2)*((F-m1*(g*sin(t1)*cos(t1)+l1*sin(t1)*t1_dot^2)-m2*(g*sin(t2)*cos(t2)+l2*sin(t2)*t2_dot^2))/(M+m1*sin(t1)^2+m2*sin(t2)^2))-(g*sin(t2)/l2);
end
