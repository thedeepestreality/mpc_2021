global Ay by bdu H f u_prev xd P;
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

Q = diag([1 1 1e4]);
R = 1e-3;
Klqr = -lqr(A,B,Q,R);
Kdlqr = -dlqr(Ad,Bd,Q,R);
eig(A+B*Klqr)';

P = 20;
[Kmpc,H,f,M,L] = mpc_lin(Ad,Bd,C,Q,R,P);
eig(A+B*Kdlqr)';

ymax = repmat([pi;180*pi/180;0*pi/180],P,1);
ymin = repmat([-pi;-180*pi;-8*pi/180],P,1);
umax = repmat(10000*pi/180,P,1);
umin = repmat(-10000*pi/180,P,1);
n = size(L,2);
% scalar weak variable
% V = ones(n*P,1);
% Ay = [M -V;
%     -M -V;
%     zeros(1,P) -1];
% by = [ymax;
%     -ymin;
%     0];
% bdu =[-L;
%     L;
%     zeros(1,n)];

V = eye(n*P,P);
Ay = [M -V;
    -M -V;
    zeros(P) -eye(P)];
by = [ymax;
    -ymin;
    zeros(P,1)];
bdu =[-L;
    L;
    zeros(P,n)];

% Au = [eye(P);-eye(P)];
% bu = [umax;-umin];
% 
% Ay = [Ay;Au];
% by = [by;bu];
% bdu = [bdu;zeros(2*P,size(L,2))];

% regime = 1; %No control
% regime = 2; %Pid
% regime = 3; %Poles
% regime = 4; %LQR
% regime = 5; % MPC Lin Noconstr
 regime = 6; % MPC Lin Constr
