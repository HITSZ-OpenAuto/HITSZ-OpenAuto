function output = signal221(n)
    output = zeros(size(n));
    for i = 1:length(n)
        if 0 <= n(i) && n(i) <= 6
            output(i) = 0.8^n(i);
        elseif 7 <= n(i) && n(i) <= 11
            output(i) = n(i) - 3;
        end
    end
end