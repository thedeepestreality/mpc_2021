function u = mpc_regulator(x)
global Ay by bdu H f u_prev xd P;
t = x(end);
x = x(1:end-1);
x = x - xd;
bmpc = by + bdu*x;
fmpc = f*x;
rho = 1e6;
Hmpc = blkdiag(H,rho*eye(P));
fmpc = [fmpc;
    zeros(P,1)];
[u,~,flag] = quadprog(Hmpc,fmpc,Ay,bmpc);
if (flag >= 0)
 u = u(1);
 u_prev = u;
else
    u=u_prev;
    disp(t);
end
