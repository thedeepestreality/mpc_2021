function u = mpc_regulator(x)
global Ay by bdu H f u_prev xd;
x = x - xd;
bmpc = by + bdu*x;
fmpc = f*x;
[u,~,flag] = quadprog(H,fmpc,Ay,bmpc);
if (flag >= 0)
 u = u(1);
 u_prev = u;
else
    u=u_prev;
end

