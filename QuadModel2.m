function [x, xang, xpos,wd, Monit] = QuadModel2(x,u,Ts)
    % Modelo n�o Linear de um Quadrirrotor
    % Fonte: Teppo Luukkunoen - Modeling and Control of Quadcopter, 2011
    % Recebe as entradas:
    % u1 = Empuxo total
    % u2 = Torque de rolagem
    % u3 = Torque de arfagem
    % u4 = Torque de guinada
    % Transforma��o U1,...,U4 -> freq. angulares dos motores
    kf = 1.4351e-5;  % constante de empuxo
    km = 2.4086e-7; % constante de torque
    L = 0.26;       % dist�ncia ao CG 
    T = [kf,kf,kf,kf;
        0, L*kf,0,-L*kf;
        -L*kf,0,L*kf,0;
        -km,km,-km,km];
    T_inv = T^(-1); 
    omega_square = T_inv*u;
    omega = zeros(4,1);
    for i=1:length(omega_square)
        omega(i,:) = sqrt(abs(omega_square(i)));
    end
    % Orienta��o das rota��es
    w1 = omega(1); w2 = - omega(2); w3 = omega(3); w4 = - omega(4);
    % Esses sinais s�o importantes para equilibrar o momento angular da
    % aeronave
    wd = w2+w4-w1-w3;
    
    % ========================== Modelo n�o linear ========================== %
    % Par�metros do modelo de simula��o
    g = 9.81;          % acelera��o da gravidade
    m = 1.03;          % massa total do ve�culo
    Ixx = 16.83e-3;    % Momento de in�rcia em x
    Iyy = 16.836e-3;    % Momento de in�rcia em y
    Izz = 28.34e-3;    % Momento de in�rcia em z
    Jr = 5e-5;         % Momento de in�rcia do rotor (assume-se que seja igual para todos os motores)
    % constantes das equa��es
    I1 = (Iyy-Izz)/Ixx; I2 = (Izz-Ixx)/Iyy; I3 = (Ixx-Iyy)/Izz;
    
    % estados
    % x = [x y z dx dy dz phi theta psi dphi dtheta dpsi]
    % x = [1 2 3 4  5  6   7    8    9    10   11    12 ]
    % Din�mica rotacional n�o linear, � no body frame da aeronave, ou seja,
    % estou preocupado com as vari�veis p, q e r. Sendo assim tenho que
    % converter para body_frame e depois voltar para o inercial
    W = [1, sin(x(7))*tan(x(8)), cos(x(7))*tan(x(8));
        0, cos(x(7)), -sin(x(7));
        0, sin(x(7))/cos(x(8)), cos(x(7))/cos(x(8))];
    W_inv = [1, 0, -sin(x(8));
             0, cos(x(7)), cos(x(8))*sin(x(7));
             0, -sin(x(7)), cos(x(8))*cos(x(7))];
    pqr = W_inv*[x(10); x(11); x(12)];
    % equa��es de movimento de ROTA��O - Body Frame
    dp = I1*pqr(2)*pqr(3) -(Jr/Ixx)*pqr(2)*wd + (L/Ixx)*u(2);
    dq = I2*pqr(1)*pqr(3) +(Jr/Iyy)*pqr(1)*wd + (L/Iyy)*u(3);
    dr = I3*pqr(1)*pqr(2) + (L/Izz)*u(4);
    dpqr = [dp; dq; dr];
    
    % Retorna para o Inertial frame
    dW = [0, x(10)*cos(x(7))*tan(x(8))+x(11)*(sin(x(7))/(cos(x(8))^2)), -x(10)*sin(x(7))*cos(x(8))+x(11)*(cos(x(7))/(cos(x(8))^2));
        0, -x(10)*sin(x(7)), -x(10)*cos(x(7));
        0, x(10)*(cos(x(7))/cos(x(8)))+x(10)*tan(x(8))*(sin(x(7))/cos(x(8))) -x(10)*(sin(x(7))/cos(x(8)))+x(11)*tan(x(8))*(cos(x(7))/cos(x(8)))];
    
    ddang = dW*pqr + W*dpqr;
     
    ddphi = ddang(1); ddtheta = ddang(2); ddpsi = ddang(3);

    % integra��o num�rica das acelera��es (� melhor forma de integrar?)
    x(10) = x(10) + ddphi*Ts;   x(7) = x(7) + x(10)*Ts; %phi
    x(11) = x(11) + ddtheta*Ts; x(8) = x(8) + x(11)*Ts; %theta
    x(12) = x(12) + ddpsi*Ts;   x(9) = x(9) + x(12)*Ts; %psi
    xang = [x(7), x(8), x(9)];
    
    % equa��es de movimento de TRANSLA��O
    ddx = (cos(x(7))*sin(x(8))*cos(x(9))+sin(x(7))*sin(x(9)))*(u(1)/m);
    ddy = (cos(x(7))*sin(x(8))*sin(x(9))-sin(x(7))*cos(x(9)))*(u(1)/m);
    ddz = cos(x(7))*cos(x(8))*(u(1)/m) - g;
    % integra��o num�rica das acelera��es (Essa � a melhor forma de integrar?)
    x(4) = x(4) + ddx*Ts; x(1) = x(1) + x(4)*Ts; %x
    x(5) = x(5) + ddy*Ts; x(2) = x(2) + x(5)*Ts; %y
    x(6) = x(6) + ddz*Ts; x(3) = x(3) + x(6)*Ts; %z
    xpos = [x(1), x(2), x(3)];
    % Vari�veis de monitoramento
    Monit = [pqr', dpqr', ddang'];
end