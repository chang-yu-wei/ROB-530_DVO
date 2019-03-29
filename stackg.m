function Mg = stackg(g)
%Mg = stackg(g)
%   Transform ksi_hat * g, which is a 4x4 matrix, into Mg * ksi,
%   ksi_hat  : 4x4
%   g        : 4x4
%   ksi      : 6x1
%   Mg       : 16x6
Mg = sym(zeros(16, 6));
Mg(1:4, 2) = g(3, :)';
Mg(1:4, 3) = -g(2, :)';
Mg(4, 4) = 1;
Mg(5:8, 1) = -g(3, :)';
Mg(5:8, 3) = g(1, :)';
Mg(8, 5) = 1;
Mg(9:12, 1) = g(2, :)';
Mg(9:12, 2) = -g(1, :)';
Mg(12, 6) = 1;
end

