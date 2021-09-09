clear;close all;clc

%%tiempo

tf = 10;             % Tiempo de simulacion en segundos (s)
ts = 0.1;            % Tiempo de muestreo en segundos (s)
t = 0: ts: tf;       % Vector de tiempo
N = length(t);       % Muestras
pause(2);
%% PARAMETROS DEL Blue
D=1.1; %distancia entre ejes
a =-.3; %Punto de control
%Posicion deseada en metros
hxd=6;
hyd=3;

vel_u = 1;         %Velocildad lineal maxima en m/s;
ang_st = 25*pi/180;        %Angulo en radianes steering maximo (+0.34,-0.34);
%% CONDICIONES INICIALES 

x2 = zeros(1, N+1);  % Posicion en el centro del robot (eje x) en metros (m)
y2 = zeros(1, N+1);  % Posicion en el centro del robot (eje y) en metros (m)
phi = zeros(1, N+1); % Orientacion del robot en radianes (rad)
psi = zeros(1, N+1); % Steering de las llantas delanteras robot en radianes (rad)

x2(1)=0;             % Posicion inicial eje x
y2(1)=0;             % Posicion inicial eje y
phi(1)=-90*pi/180;   % Orientacion inicial del robot
psi(1)=-25*pi/180;           % Steering inicial del robot

%% PUNTO DE CONTROL 

hx = zeros(1, N+1);  % Posicion en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1);  % Posicion en el punto de control (eje y) en metros (m)
hx(1) = x2(1)+a*cos(phi(1)); % Posicion en el punto de control del robot en el eje x
hy(1) = y2(1)+a*sin(phi(1)); % Posicion en el punto de control del robot en el eje y

%% Velocidades de Referencia
u = zeros(1,N+1);
w = zeros(1,N+1);

%% Errores
hxe = zeros(1,N);
hye = zeros(1,N);

%% Loop simulación 
scene = figure;
 
    for i=1:length(t)-1       

        hxe(i)=hxd(1)-hx(i);
        hye(i)=hyd(1)-hy(i);
        herr=[hxe(i);hye(i)]; % errores en matricial

%          hxdp(i) = (hxd-hxd)/ts;
%          hydp(i) = (hyd-hyd)/ts;
% 
%         hdp=[hxdp(i);hydp(i)]; %usado para trayectorias    

        %Jacobiano hacia adelante    
        % J=[cos(phi(i)) , -abs(a) * sin(phi(i));
        %    sin(phi(i)) ,  abs(a) * cos(phi(i))];

       %Jacobiano hacia atras  
         J=[cos(phi(i)) ,   abs(a) * sin(phi(i));
            sin(phi(i)) , - abs(a) * cos(phi(i))];   

        %distancia
        dist = sqrt((hxd(1)-hxe(i))^2+(hyd(1)-hye(i))^2);
        k_max = 2;
        k1 = 1;
        gain(i) = k_max/(1+k1*dist);

        K2=diag([gain(i),gain(i)]);
        %K2=diag([1,1]);
        K1=diag([5,5]);


        %ley de control
        %vel=inv(J)*(hdp+K1*tanh(K2*herr));
        vel=inv(J)*(K1*tanh(K2*herr));

        u(i)=vel_u * tanh(vel(1)); % velocidad lineal 
        w(i)=vel(2); % velocidad angular 
        if (w(i)> 0.01 || w(i)< -0.01)
            psi(i)=ang_st* tanh(atan(w(i)*D/u(i))); % angulo de Steering
        else
           psi(i) = 0;
        end

        %%Mediciones 
        w(i)=u(i)*tan(psi(i))/D; %velocidad angular nuevamente calculado
        phi(i+1)=phi(i)+ts*w(i); % angulo de giro     

       %%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
        %Modelo hacia adelante.
        %hxp = u(i)*cos(phi(i+1))- (a)*w(i)*sin(phi(i+1));
        %hyp = u(i)*sin(phi(i+1))+ (a)*w(i)*cos(phi(i+1));

        %Modelo hacia atras.
         hxp = u(i)*cos(phi(i+1))+ abs(a)*w(i)*sin(phi(i+1));
         hyp = u(i)*sin(phi(i+1))- abs(a)*w(i)*cos(phi(i+1));

        % Integracion metodo de Euler
        hx(i+1)=hx(i)+ts*hxp;
        hy(i+1)=hy(i)+ts*hyp;

        % Posicion del robot con respecto al punto de control    
        x2(i+1)=hx(i+1)- (a)*cos(phi(i));  
        y2(i+1)=hy(i+1)- (a)*sin(phi(i));  

    end

%% Simulacion 
view(10,25)
axis equal;
axis([-4 10 -2 9 0 1.5]); 
grid on;
hold on;
plot(hxd,hyd,'r*');
plot(x2(1)+a*cos(phi(1)),y2(1)+a*sin(phi(1)),'g*');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); 
camlight('HEADLIGHT');
M1=bluePlot(x2(i),y2(i),0,phi(i),psi(i),0.001);
step=1;
for i=1:step:length(t) 
    
    delete (M1) 
    M1=bluePlot(x2(i),y2(i),0,phi(i),psi(i),0.001); hold on
    plot(hx(1:i),hy(1:i),'b','LineWidth',2);
    pause(ts)
end

figure(2)
plot(hxd,hyd,'r*'), hold on;plot(hx(1:end-1),hy(1:end-1),'b')
hold on
bluePlot(x2(1),y2(1),0,phi(1),psi(1),0.001);  
bluePlot(x2(end-1),y2(end-1),0,phi(end-1),psi(end-1),0.001);
axis([-4 10 -4 10]);
hold off
grid minor

figure(3)
plot(t,hxe,'b');set(gca, 'FontSize', 20); hold on;
plot(t,hye,'r');set(gca, 'FontSize', 20)
legend('x_{error}(m)','y_{error}(m)','FontSize',20)
xlabel('t(s)','FontSize',20);
grid minor

figure(4)
plot(t,u(1:end-1),'b'); hold on;plot(t,w(1:end-1),'g');plot(t,psi(1:end-1),'r');
set(gca, 'FontSize', 20)
grid minor
legend('\mu (m/s)','\omega (rad/s)','\phi(rad)','FontSize',20)
xlabel('t(s)','FontSize',20);

figure
plot(gain)
grid on
