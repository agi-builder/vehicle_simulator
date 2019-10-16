clear
clc
close all

% assign parameters
a = 1;
b = 0.7;
c = 0.4;
mu1 = 0.004; %roling resistance coefficient
mu2 = 0.3;    %friction coefficient
ks = 10;
r = 0.4;
mass = 1000;
I = mass*((a+b)^2+c^2)/12;

vehicle = Vehicle(a,b,c,mu1,mu2,ks,r,mass,I);   

theta = 0;               % initial angles
omega = 0;               % initial angular velocities
x = 0; y = 0;            % initial position
xdot = 25; ydot =0;     % initial velocity

t = 7;                      % simulation time

ic = [x; y; theta; xdot; ydot; omega]; % initial conditions, in body frame
options = odeset('RelTol',1e-4,'AbsTol',1e-8*ones(6,1)); % solver options

delta = -pi/2:0.01:0;
xx = [0:0.01:100, 100+20*cos(delta), 120+zeros(size(0:0.01:100))];
yy = [zeros(size(0:0.01:100)), 20+20*sin(delta), 20:0.01:120];

[T,Y] = vehicle.motion(t,ic,options);

figure
plot(T,[Y(:,1),Y(:,2)]);
title('Distance')
legend({'x','y'})
grid on
figure
plot(T,[Y(:,4),Y(:,5),(Y(:,4).^2+Y(:,5).^2).^(0.5)]);
title('Velocity')
legend({'x','y','Total'})
grid on
figure
grid on
rectangle('Position',[90,0,2,2])
hold on
plot(xx,yy,'r--')
plot(Y(:,1),Y(:,2));
plot(27,0,'r*')
title('Trajectory')
axis([0 140 -20 120])
legend({'Desired Trajectory','Actual Trajectory'})
axis equal
grid on



figure
for i =2:200:length(Y(:,1))
    subplot(211)
    plot(xx,yy,'r--')
    hold on
    plot([Y(i,1)+b*cos(Y(i,3))+c*sin(Y(i,3)),Y(i,1)+b*cos(Y(i,3))-c*sin(Y(i,3))],[Y(i,2)+b*sin(Y(i,3))-c*cos(Y(i,3)),Y(i,2)+b*sin(Y(i,3))+c*cos(Y(i,3))],'r','LineWidth',2)
    
    plot([Y(i,1)-a*cos(Y(i,3))+c*sin(Y(i,3)),Y(i,1)-a*cos(Y(i,3))-c*sin(Y(i,3))],[Y(i,2)-a*sin(Y(i,3))-c*cos(Y(i,3)),Y(i,2)-a*sin(Y(i,3))+c*cos(Y(i,3))],'r','LineWidth',2)
    plot([Y(i,1)+b*cos(Y(i,3))+c*sin(Y(i,3)),Y(i,1)-a*cos(Y(i,3))+c*sin(Y(i,3))],[Y(i,2)+b*sin(Y(i,3))-c*cos(Y(i,3)),Y(i,2)-a*sin(Y(i,3))-c*cos(Y(i,3))],'r','LineWidth',2)
    plot([Y(i,1)+b*cos(Y(i,3))-c*sin(Y(i,3)),Y(i,1)-a*cos(Y(i,3))-c*sin(Y(i,3))],[Y(i,2)+b*sin(Y(i,3))+c*cos(Y(i,3)),Y(i,2)-a*sin(Y(i,3))+c*cos(Y(i,3))],'r','LineWidth',2)
    plot(Y(i,1),Y(i,2),'r.')
    rectangle('Position',[90,0,2,2])
    axis equal
    plot(Y(1:i,1),Y(1:i,2),'b-')
    plot(27,0,'r*')
    axis([Y(i,1)-3 Y(i,1)+3 Y(i,2)-3 Y(i,2)+3])
    grid on
    hold off
    title( {['Time = ', num2str(T(i)),'s']})   
    drawnow
    
    
    subplot(212)
    plot(xx,yy,'r--')
    hold on
    plot([Y(i,1)+b*cos(Y(i,3))+c*sin(Y(i,3)),Y(i,1)+b*cos(Y(i,3))-c*sin(Y(i,3))],[Y(i,2)+b*sin(Y(i,3))-c*cos(Y(i,3)),Y(i,2)+b*sin(Y(i,3))+c*cos(Y(i,3))],'r','LineWidth',2)
    plot([Y(i,1)-a*cos(Y(i,3))+c*sin(Y(i,3)),Y(i,1)-a*cos(Y(i,3))-c*sin(Y(i,3))],[Y(i,2)-a*sin(Y(i,3))-c*cos(Y(i,3)),Y(i,2)-a*sin(Y(i,3))+c*cos(Y(i,3))],'r','LineWidth',2)
    plot([Y(i,1)+b*cos(Y(i,3))+c*sin(Y(i,3)),Y(i,1)-a*cos(Y(i,3))+c*sin(Y(i,3))],[Y(i,2)+b*sin(Y(i,3))-c*cos(Y(i,3)),Y(i,2)-a*sin(Y(i,3))-c*cos(Y(i,3))],'r','LineWidth',2)
    plot([Y(i,1)+b*cos(Y(i,3))-c*sin(Y(i,3)),Y(i,1)-a*cos(Y(i,3))-c*sin(Y(i,3))],[Y(i,2)+b*sin(Y(i,3))+c*cos(Y(i,3)),Y(i,2)-a*sin(Y(i,3))+c*cos(Y(i,3))],'r','LineWidth',2)
    plot(Y(i,1),Y(i,2),'r.')
    rectangle('Position',[90,0,2,2])
    axis equal
    plot(Y(1:i,1),Y(1:i,2),'b-')
    plot(27,0,'r*')
    
    
    axis([min(Y(:,1))-3 Y(i,1)+5 min(Y(:,2))-10 max(Y(:,2))+10])
    grid on
    hold off
    drawnow
end
