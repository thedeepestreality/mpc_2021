clear;
global Ay by bdu H f x_prev u_prev xd P;
global Ay_ast by_ast bdu_ast H_ast f_ast Kmpc_ast;
global A B;
u_prev = 0;
u0 = 0;
x0 = [0;0;0];
x_prev = x0;

kp = 20*[-1 1 -1];
xd = [0;0;10*pi/180];
%xd = [0;1*pi/180;0];

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
% umax = repmat(10000*pi/180,P,1);
% umin = repmat(-10000*pi/180,P,1);

[Ay, by, bdu] = mpc_linconstr(P,M,L,ymin,ymax);

% ASTATIC MPC
%{
A_ast = [Ad zeros(3,3);
         C*Ad eye(3,3)];
B_ast = [Bd; C*Bd];
C = [zeros(3,3) eye(3,3)];
[Kmpc_ast,~,~,~,~] = mpc_lin(A_ast,B_ast,C,Q,R,P);
%}

C = [0 0 1];
A_ast = [Ad zeros(3,1);
         C*Ad eye(1)];
B_ast = [Bd; C*Bd];
C_ast = [zeros(1,3) eye(1)];
Q = 1e4;
[Kmpc_ast,H_ast,f_ast,M_ast,L_ast] = mpc_lin(A_ast,B_ast,C_ast,Q,R,P);
ymax = repmat(0*pi/180,P,1);
ymin = repmat(-8*pi/180,P,1);
[Ay_ast, by_ast, bdu_ast] = mpc_linconstr(P,M_ast,L_ast,ymin,ymax);

% regime = 1; %No control
% regime = 2; %Pid
% regime = 3; %Poles
% regime = 4; %LQR
% regime = 5; % MPC Lin Noconstr
% regime = 6; % MPC Lin Constr
% regime = 7; % MPC Ast Noconstr
regime = 8; % MPC Ast Constr
