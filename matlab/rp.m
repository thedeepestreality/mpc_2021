function dx = rp(in)

    x = in(1);
    u = in(2);
    
    a = 0.5;
    b = 0.1;
    
    dx = a*x + b*u;

end