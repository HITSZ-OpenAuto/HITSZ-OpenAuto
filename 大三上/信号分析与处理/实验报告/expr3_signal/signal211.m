function output = signal211(n)
    output = zeros(size(n));
    for i = 1:length(n)
        if 0 <= n(i) && n(i) <= 4
            output(i) = n(i) + 1;
        elseif 5 <= n(i) && n(i) <= 9
            output(i) = 10 - n(i);
        end
    end
end