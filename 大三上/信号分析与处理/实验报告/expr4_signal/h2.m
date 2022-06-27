function output = h2(omega)
    omega = sqrt(-1).*omega;
    output = (omega+2)./(omega.^2 + 2.*omega + 2);
end