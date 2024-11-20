function pqr_des = Angle2pqr_Target(z_d,t)
% Transforma referencia de ângulo em ref de taxa no corpo pqr
%Entradas:
%   t: É o vetor de tempo que também define o tamanho dos vetores de dados.
%   z_d: O vetor que carrega as informações de comando para ângulo de Euler e suas
%   derivadas
%Variável intermediária:
%   x_d: vetor que carregará as informações de pqr com tamanho definido de
%   t linhas e 3 colunas.
%Saída:
%   pqr_des: Vmatriz como todos os valores de pqr_des
x_d = zeros(length(t),3);
for i=1:length(t)
    dang = z_d(i,4:6)'; % são as derivadas dos ângulos de euler
    T = [1 0 -sin(z_d(i,2));0 cos(z_d(i,1)) sin(z_d(i,1))*cos(z_d(i,2)); 0 -sin(z_d(i,1)) cos(z_d(i,1))*cos(z_d(i,2))];
    x_d(i,:) = (T*dang)';
end
pqr_des = x_d;

end
