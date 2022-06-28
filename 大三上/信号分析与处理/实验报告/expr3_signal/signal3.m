function output = signal3(t)
    output = (sin(100.*t)./(100.*t)) + rand(size(t));
end