function output = triangle_wave(n)
    output = zeros(size(n));
    for i = 1:length(n)
        if 0 <= n(i) && n(i) <=4
            output(i) = n(i) + 1;
        elseif 5 <= n(i) && n(i) <= 8
            output(i) = 9 - n(i);
        else
            output(i) = 0;
        end
    end
end