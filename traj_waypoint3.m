function X_d = traj_waypoint3(t,Ts)
    %Fun��o criada para gerar uma trajet�ria a partir de waypoints
    fs = 1/Ts;
    n = int32(length(t)/15); % Vetor de tempo dividido em 10 partes
    %      Tempo   Waypoint(x,y,z),     Orienta��o
    wps = [ t(1),        0,0,0,              0,0,0;
           t(n),         1,1,1,              0,0,0;
           t(2*n),       2,0,2,              0,0,0;
           t(3*n),       2,-3,1,             0,0,0;
           t(4*n),       -2,2,1,              0,0,0;
           t(5*n),       3,3,3,              0,0,0;
           t(6*n),       2,4,3.5,            0,0,0;
           t(7*n),       5,-2,4,             0,0,0;
           t(8*n),       -1,0,4,             0,0,0;
           t(9*n),       -1,1,4,             0,0,0;
           t(10*n),       0,1,5,             0,0,0;
           t(11*n),       0.5,2,5,             0,0,0;
           t(12*n),       1,2.5,5,             0,0,0;
           t(13*n),       2,1,5,             0,0,0;
           t(end),       0,1,6,             0,0,0;];

    trajetoria = waypointTrajectory(wps(:,2:4),'SampleRate',fs,'TimeOfArrival',wps(:,1),...
        'Orientation',quaternion(wps(:,5:7),'eulerd','ZYX','frame'));
    tInfo = waypointInfo(trajetoria); % Retorna tempo, velocidade e orientacao
%     figure; 
%     plot3(tInfo.Waypoints(1,1),tInfo.Waypoints(1,2),tInfo.Waypoints(1,3),'r*')
%     grid on
    % ponto de inicio
    count = 1;
    while ~isDone(trajetoria)
        [pos(count,:), orient(count), vel(count,:), acc(count,:), angVel(count,:)] = trajetoria();

        %plot3(pos(1),pos(2),pos(3),"gd")

        pause(trajetoria.SamplesPerFrame/trajetoria.SampleRate)
        count = count + 1;
    end
    timeVector = 0:(1/trajetoria.SampleRate):tInfo.TimeOfArrival(end);
    eulerAngles = eulerd(orient,'ZYX','frame');
    X_d = [pos, vel, eulerAngles, angVel];
end