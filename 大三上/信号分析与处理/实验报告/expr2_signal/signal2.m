function output = signal2(input)
    output = zeros(size(input));
    con1 = find((0 <= input) & (input <= 13));
    output(con1) = input(con1) + 1;
    con2 = find((14 <= input) & (input <= 26));
    output(con2) = 27 - input(con2);
end