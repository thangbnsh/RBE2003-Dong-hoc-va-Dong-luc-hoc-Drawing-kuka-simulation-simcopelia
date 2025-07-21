function dx=pendulum(t,x,flag,control_input)
global A B C D;
u=control_input;
dx=zeros(4,1);
%State equation for one link inverted pendulum
dx=A*x+B*u;
end