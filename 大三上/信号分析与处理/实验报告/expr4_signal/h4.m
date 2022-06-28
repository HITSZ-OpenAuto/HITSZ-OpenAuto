function output = h4(omega)
    omega = sqrt(-1).*omega;
    output = (2.*omega+1) ./ (((2.*omega+1).^3) .* (omega.^2 + 4));
end