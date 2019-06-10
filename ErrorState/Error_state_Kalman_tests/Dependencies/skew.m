function M = skew(a)
%skew matrix from Groves texbook
    M = [0, -a(3), a(2); a(3), 0, -a(1); -a(2), a(1), 0];
end