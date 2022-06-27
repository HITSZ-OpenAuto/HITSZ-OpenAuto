function output = signal1(input)
    A = 444.128;
    alpha = 50 * sqrt(2) * pi;
    omega1 = 50 * sqrt(2) * pi;
    output = A .* exp(-alpha.*input) .* sin(omega1 .* input);
end