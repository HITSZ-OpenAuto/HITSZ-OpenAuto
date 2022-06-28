function output = signal212(n)
    output = zeros(size(n));
    for i = 1:length(n)
        if 0 <= n(i) && n(i) <= 5
            output(i) = 2^n(i);
        elseif 6 <= n(i) && n(i) <= 11
            output(i) = (-1) * (2^(n(i) - 6));
        end
    end
end