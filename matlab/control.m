function out = control(in)
    x = in(1);
    ei = in(2);
    xd = 0.5;
    ep = x - xd;
    kp = -12;
    ki = -4;
    
    u = kp*ep + ki*ei;
    out = [u; ep];

end