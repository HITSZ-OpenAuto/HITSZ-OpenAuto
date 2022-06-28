function output = signal222(n)
    output = zeros(size(n));
    for i = 1:length(n)
        if 0 <= n(i) && n(i) <= 4
            output(i) = n(i) - 1;
        elseif 5 <= n(i) && n(i) <= 12
            output(i) = (-1) * (0.6^(n(i) - 6));
        end
    end
end