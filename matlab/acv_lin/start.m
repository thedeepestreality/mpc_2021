global Ay by bdu H f u_prev xd;
global A B;
u_prev = 0;
u0 = 0;
x0 = [0;0;0];

kp = 20*[-1 1 -1];
xd = [0;0;10*pi/180];

A = [-0.3176 0.852 0;
     -0.0102 -0.1383 0;
     0 1 0];
B = [-0.005;
     -0.0217;
     0];
 
C = eye(3);
D = 0;
sys = ss(A,B,C,D);
dt = 0.1;
dsys = c2d(sys,dt);
[Ad,Bd,~,~] = ssdata(dsys);

roots = [-10;-20;-30;];
%roots = [-100;-1;-1.1;-1.2];
Kmod = -place(A,B,roots);

Q = diag([1 1 1e3]);
R = 1e-2;
Klqr = -lqr(A,B,Q,R);
Kdlqr = -dlqr(Ad,Bd,Q,R);
eig(A+B*Klqr)';

P = 20;
[Kmpc,H,f,M,L] = mpc_lin(Ad,Bd,C,Q,R,P);
eig(A+B*Kdlqr)';

ymax = repmat([6*pi/180;pi;180*pi/180],P,1);
ymin = repmat([-pi;-pi;-180*pi/180],P,1);
Ay = [M;-M];
by = [ymax;-ymin];
bdu =[-L;L];

% regime = 1; %No control
% regime = 2; %Pid
% regime = 3; %Poles
% regime = 4; %LQR
% regime = 5; % MPC Lin Noconstr
 regime = 6; % MPC Lin Constr
