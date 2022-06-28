function output = sqaure_wave(n)
    output = zeros(size(n));
    for i = 1:length(n)
        if 0 <= n(i) && n(i) <= 6
            output(i) = 1;
        end
    end
end