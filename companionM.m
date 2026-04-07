function [K_comp] = companionM(A,B,P)
%Compute Fulll state feedback K matrix with the Companion Matrix Method
%   K = K_tilde/W

%Convert to Controllable Canonical Form
n=length(A);
A_tilde = zeros(n);
for i=1:n-1
    A_tilde(i,i+1) = 1;
end
coeff = flip(poly(A));
A_tilde(n,:) =  -coeff(1:n);
B_tilde = zeros(n,1);
B_tilde(end) = 1;

K_tilde = place(A_tilde,B_tilde,P);
T_tilde = ctrb(A_tilde,B_tilde);
T=ctrb(A,B);
W_tilde = T/T_tilde; %%check this
K_comp = K_tilde/W_tilde;

end

