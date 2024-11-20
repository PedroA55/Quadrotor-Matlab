function [dphi_filtered, dtheta_filtered, dpsi_filtered] = Filtro_PB(dphi, dtheta, dpsi, Ts)
%Funcao com todas as estratégias de filtro Passa-Baixa para as derivadas dos
%ângulos de Euler
%   1º Filtro: Filtro Passa-Baixa de Butterworth
%   Frequência de corte de 25 Hz
fc = 25;
dphi_filtered = Filter_butter(dphi,fc,Ts);
dtheta_filtered = Filter_butter(dtheta,fc,Ts);
dpsi_filtered = Filter_butter(dpsi,fc,Ts);
end

