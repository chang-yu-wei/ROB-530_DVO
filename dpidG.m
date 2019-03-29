function J = dpidG(G, f)
% dpi/dG = dpidG(G(g(t0), S(x))
%   Calculate Jacobian of function pi w.r.t. G
%   S is a function of (x, f, o), o is not used in Jacobian
%   function pi is 2x1, G is 3x1, f is 2x1, o is 2x1

J = zeros(2, 3); 
J(1, 1) = f(1)/G(3);
J(1, 2) = 0;
J(1, 3) = -G(1)*f(1)/(G(3)*G(3));
J(2, 1) = 0;
J(2, 2) = f(2)/G(3);
J(2, 3) = -G(2)*f(2)/(G(3)*G(3));

end

