function [M2, M3, M4] = controller_LQT(x, z, K, Kz)
    u = -K*x(7:12)' + Kz*(z');
    M2 = u(1);
    M3 = u(2);
    M4 = u(3);
end