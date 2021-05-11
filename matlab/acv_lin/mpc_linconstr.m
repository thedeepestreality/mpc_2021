function [Ay, by, bdu] = mpc_linconstr(P,M,L,ymin,ymax)

%n = size(L,2);
n = uint32(length(ymin)/P);

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
    zeros(P,size(L,2))];

% Constraints to the control signal
% Au = [eye(P);-eye(P)];
% bu = [umax;-umin];
% 
% Ay = [Ay;Au];
% by = [by;bu];
% bdu = [bdu;zeros(2*P,size(L,2))];

end