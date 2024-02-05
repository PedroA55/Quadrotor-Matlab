function X_d = traj_waypointFinal(t,Ts)
    %Função criada para gerar uma trajetória a partir de waypoints
    fs = 1/Ts;
    n = int32(length(t)/3); % Vetor de tempo dividido em 3 partes
    %      Tempo   Waypoint(x,y,z),     Orientação
    wps = [ t(1),        0,0,1.5,              0,0,0;
           t(n),         1,-1,1.5,              0,0,0;
           t(end),       1,1,1.5,              0,0,0];

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
