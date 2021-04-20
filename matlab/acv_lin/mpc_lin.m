function [K,H,f,M,L] = mpc_lin(A,B,C,R0,Q0,P)
R = [];
Q = [];
L = [];
M0= [];
for i=1:P
   R = blkdiag(R,R0);
   Q = blkdiag(Q,Q0);
   L = [L;C*(A^i)];
   M0 = [M0; C*(A^(i-1))*B];
end
M = M0;
[m,n] = size(C*B);
for i=1:(P-1)
   M = [M, [zeros(i*m,n); M0(1:m*(P-i),:)]];
end

H = M'*R*M+Q;
f = M'*R*L;
K = -H\f;
[~,m] = size(B);
K = K(1:m,:);