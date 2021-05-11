function dx = acv_rp(z)
global A B;
x = z(1:3);
u = z(4);
dx = A*x + B*u + 1*[0;-0.5;0];