function z = sat(x)
% Saturation function
z = zeros(size(x));
g1 = x > 1;
l1 = x < -1;
i1 = abs(x) <= 1;
z(g1) = 1;
z(l1) = -1;
z(i1) = x(i1);
end

