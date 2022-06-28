function output = square_wave(t)
    output = zeros(size(t));
    for i = 1:length(t)
        if(0 <= mod(t(i), 4) && mod(t(i), 4) < 2)
            output(i) = -1;
        elseif(2 <= mod(t(i), 4) && mod(t(i), 4) < 4)
            output(i) = 1;
        end
    end
end