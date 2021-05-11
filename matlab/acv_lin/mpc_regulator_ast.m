function u = mpc_regulator_ast(x)

global Ay_ast by_ast bdu_ast H_ast f_ast u_prev x_prev xd P;
dx = x - x_prev;
x_prev = x;
y = x(3) - xd(3);
p = [dx;y];
bmpc_ast = by_ast + bdu_ast*p;
fmpc_ast = f_ast*p;
rho = 1e6;
Hmpc_ast  = blkdiag(H_ast,rho*eye(P));
fmpc_ast  = [fmpc_ast;
    zeros(P,1)];
[du,~,flag] = quadprog(Hmpc_ast,fmpc_ast,Ay_ast,bmpc_ast);
if (flag >= 0)
 du = du(1);
 u = u_prev + du;
 u_prev = u;
else
    u=u_prev;
end
