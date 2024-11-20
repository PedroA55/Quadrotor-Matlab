% Este algoritmo é para testar waypoints. Verificar se eles dão uma
% trajetória complexa possível de ser executada.

Fs = 1/0.001; % leva em consideração a taxa de amostragem definida para a simulação: 0,001s 
wps = [0 0 0;
       1 1 1;
       2 0 2;
       2 1 1;
       1 2 -1;
       2 2 2;
       2 2 3.5;
       5 -2 4;
       -1 0 4;
       -1 1 4];
t = 0:(size(wps,1)-1);
    
traj = waypointTrajectory(wps, t, 'SampleRate', Fs);
    
waypointTable = waypointInfo(traj);
waypoints = waypointTable.Waypoints;
    
pos = traj();
while ~isDone(traj)
        pos(end+traj.SamplesPerFrame,:) = traj();
end
% Plot generated positions and specified waypoints.
figure;
plot3(pos(:,1),pos(:,2),pos(:,3),'b','LineWidth',1);
hold on
plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3), '--g','LineWidth',1);
grid minor
title('Position')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend({'Position', 'Waypoints'})