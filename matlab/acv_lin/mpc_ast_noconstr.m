function u = mpc_ast_noconstr(x)
% global u_prev x_prev Kmpc_ast xd;
% 
% p = [x-x_prev;
%     x(3)-xd(3)];
% du = Kmpc_ast*p;
% u = u_prev + du;
% 
% u_prev = u;
% x_prev = x;
u = 0;

end