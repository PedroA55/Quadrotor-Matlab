function U = Err_cal(Ref_att, Real_att, cmd,Ts)
% ERR_CAL: Funcao para avaliar desempenho do sistema de controle de atitude
err  = (Ref_att - Real_att).*(pi/180);
ISE  = zeros(3,1);
IAE  = zeros(3,1);
ITSE = zeros(3,1);
ITAE = zeros(3,1);
Uctrl = zeros(3,1);

for i=1:3
    ISE(i) = trapz(err(:,i).^2);
    IAE(i) = trapz(abs(err(:,i)));
    ITSE(i) = trapz((Ts*(0:length(err(:,i))-1))'.*(err(:,i).^2));
    ITAE(i) = trapz((Ts*(0:length(err(:,i))-1))'.*abs(err(:,i)));
    Uctrl(i) = 100*(max(abs(cmd(:,i))));
end

U = [ISE, IAE, ITSE, ITAE, Uctrl];

end
