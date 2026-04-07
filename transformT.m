function [K,T,W] = transformT(A,B,P)
%Compute Fulll state feedback K matrix with the Transformation Matrix T
%method
%   Insert the poles in negative values

syms s;
n = length(A);
a = flip(poly(A));% [an,an-1, ..., a2, a1] , det(s*eye(n) - A)

%%Compute W
W = zeros(n);%Init W
count=0;
w_coeff = a; %Coefficients vector to compute W
if length(a)>n
    w_coeff(1) = [];%delete a4 component
end

for i=1:n
    for j=1:n-i
        W(i,j) = w_coeff(j+count);        
    end
    W(i,j+1)=1;
    count= count+1;
end
W(i,1)=1;
M = ctrb(A,B);%Compute controllability matrix
T = M*W;%Compute T matrix

% Initialize the desired polynomial
desPolynomial = 1;  % Start with a polynomial of 1

% Create the polynomial from the poles
for i = 1:length(P)
    desPolynomial = desPolynomial * (s-P(i));  % Multiply (s - pi) for each pole
end
% Expand to standard form
expanded_polynomial = vpa( expand(desPolynomial) );
coeffDes = coeffs(expanded_polynomial);
%coeffs(chPolynomial,s,"All")  %Gives all the coeff, but in different order

prevMatrix = coeffDes(1:n) - a(1:n); %Does not include a_0 and alpha_0
K = prevMatrix/T; %return K matrix


