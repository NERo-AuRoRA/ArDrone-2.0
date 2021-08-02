% Testar modelo din�mico em uma trajet�ria em tempo real

close all
clear
clc

try
    fclose(instrfindall);
end
% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


% - Representa��o do rob�
A = ArDrone;

% Alterando posi��o
A.pPos.X(3) = 0.75;


tmax = 60; % Tempo Simula��o em segundos
X = zeros(1,19); % Dados correntes da simula��o

limites = [-2 2 -2 2 0 2];
figure(1)
% Ground = patch(limites([1 1 2 2]),limites([3 4 4 3]),limites([5 5 5 5]),[0.6 1 0.6]);
% Ground.FaceAlpha = 0.4;
view(45,30)
axis equal
axis(limites)
grid on

% Estilizando superficie
lighting phong;
material shiny;
colormap winter;
lightangle(-45,30)


Info = title(['Time: ' num2str(0,'%05.2f') ' | ' num2str(tmax,'%05.2f') 's']);  

% A.mCADplot('indoor',1);
A.mCADplot()

% Alterando cor do rob�
A.mCADcolor([0 51 80]/255);

drawnow
pause(3)
disp('Start............')

% =========================================================================
t = tic;
tc = tic;
tp = tic;

XX = [];
TT = [];
while toc(t) < tmax
    ta = tic;
    
    if toc(tc) > 1/30
        TT = [TT toc(tc)];
        
        
        tc = tic;
        
        w = 0.025;
        % Trajet�ria desejada
        tt = toc(t);
        A.pPos.Xd(1) = 1*sin(2*pi*w*tt);            % x
        A.pPos.Xd(7) = 1*2*pi*w*cos(2*pi*w*tt);     % dx
        A.pPos.Xd(2) = 1*sin(2*pi*2*w*tt);          % y
        A.pPos.Xd(8) = 1*2*pi*2*w*cos(2*pi*2*w*tt); % dy
        A.pPos.Xd(3) = 1;                           % z       
                        
        % Controlador
        A.rGetSensorData
        A = cUnderActuatedController(A);        
        A.rSendControlSignals;
        
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];
        
    end
    if toc(tp) > 0.1
        tp = tic;
        
        if toc(t) > 3*tmax/4
            A.mCADplot('indoor',1);
        elseif toc(t) > 2*tmax/4
            A.mCADplot('indoor',0);
        elseif toc(t) > 1*tmax/4
            A.mCADplot('outdoor',1);
        else 
            A.mCADplot('outdoor',0);
        end
        
        Info.String = ['Time: ' num2str(toc(t),'%05.2f') ' | ' num2str(tmax,'%05.2f') 's'];
        
        drawnow
    end
    
    toc(ta)
end

% figure
% subplot(211),plot(XX(end,:),XX([4 16],:)'*180/pi)
% legend('\phi_{Des}','\phi_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([5 17],:)'*180/pi)
% legend('\theta_{Des}','\theta_{Atu}')
% grid
% 
% figure
% plot(XX([1,13],:)',XX([2,14],:)')
% % axis([-1.5 1.5 -1.5 1.5])
% axis equal
% 
% figure
% subplot(211),plot(XX(end,:),XX([1 13],:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([2 14],:)')
% legend('y_{Des}','y_{Atu}')
% grid

% figure
% subplot(211),plot(XX(end,:),XX(19,:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),[XX(13,:); [0 diff(XX(13,:))]*30])
% legend('y_{Des}','y_{Atu}')
% grid

