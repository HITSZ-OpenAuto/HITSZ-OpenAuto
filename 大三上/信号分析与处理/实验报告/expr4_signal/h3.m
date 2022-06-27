function output = h3(omega)
    omega = sqrt(-1).*omega;
    output = (omega.^2 + 2.*omega + 1)./(omega.^3 + 2.*omega.^2 + 3.*omega + 6);
end