function ydot = my_cart_EOM(y,t,A,B,K)
%% Linear Model
ydot = (A-B*K)*y;
end
