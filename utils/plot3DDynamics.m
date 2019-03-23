function [ output_args ] = plot3DDynamics( Data, index, input, output )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Plot input profiles
figure('Color',[1 1 1])
for ii=1:(length(index)-1)   
    plot3(Data(1,index(ii):index(ii+1)-1),Data(2,index(ii):index(ii+1)-1),Data(3,index(ii):index(ii+1)-1),'-*', 'MarkerSize',2); hold on;
end
grid on; 
axis equal;
xlabel('$\xi_r^1$','interpreter','latex')
ylabel('$\xi_r^2$','interpreter','latex')
zlabel('$\xi_r^2$','interpreter','latex')

plot3(Data(1,end),Data(2,end),Data(3,end), '*k', 'MarkerSize',5);hold on;
view([150 11]);
suptitle(strcat('Input (',input,')',' Profiles'));

% Plot output profiles
figure('Color',[1 1 1])
for ii=1:(length(index)-1)   
    plot3(Data(4,index(ii):index(ii+1)-1),Data(5,index(ii):index(ii+1)-1),Data(6,index(ii):index(ii+1)-1),'-*', 'MarkerSize',2); hold on;
end
grid on;
axis equal;
xlabel('$\dot{\xi}_r^1$','interpreter','latex')
ylabel('$\dot{\xi}_r^2$','interpreter','latex')
zlabel('$\dot{\xi}_r^3$','interpreter','latex')
hold on;
plot3(Data(4,end),Data(5,end),Data(6,end), '*k')
view([150 11]);
suptitle(strcat('Output (',output,')',' Profiles'));

% Plot variable interactions
figure('Color',[1 1 1])
subplot(3,3,1)
plot(Data(1,:),Data(4,:),'*r','MarkerSize',2); hold on;
grid on; 
xlabel('$\xi_x^1$','interpreter','latex')
ylabel('$\dot{\xi}_x^1$','interpreter','latex')

subplot(3,3,2)
plot(Data(1,:),Data(5,:),'*g','MarkerSize',2); hold on;
grid on; 
xlabel('$\xi_x^1$','interpreter','latex')
ylabel('$\dot{\xi}_x^2$','interpreter','latex')

subplot(3,3,3)
plot(Data(1,:),Data(6,:),'*b','MarkerSize',2); hold on;
grid on; 
xlabel('$\xi_x^1$','interpreter','latex')
ylabel('$\dot{\xi}_x^3$','interpreter','latex')

subplot(3,3,4)
plot(Data(2,:),Data(4,:),'*r','MarkerSize',2); hold on;
grid on; 
xlabel('$\xi_x^2$','interpreter','latex')
ylabel('$\dot{\xi}_x^1$','interpreter','latex')

subplot(3,3,5)
plot(Data(2,:),Data(5,:),'*g','MarkerSize',2); hold on;
grid on; 
xlabel('$\xi_x^2$','interpreter','latex')
ylabel('$\dot{\xi}_x^2$','interpreter','latex')

subplot(3,3,6)
plot(Data(2,:),Data(6,:),'*b','MarkerSize',2); hold on;
grid on; 
xlabel('$\xi_x^2$','interpreter','latex')
ylabel('$\dot{\xi}_x^3$','interpreter','latex')

subplot(3,3,7)
plot(Data(3,:),Data(4,:),'*r','MarkerSize',2); hold on;
grid on; 
xlabel('$\xi_x^3$','interpreter','latex')
ylabel('$\dot{\xi}_x^1$','interpreter','latex')

subplot(3,3,8)
plot(Data(3,:),Data(5,:),'*g','MarkerSize',2); hold on;
grid on; 
xlabel('$\xi_x^3$','interpreter','latex')
ylabel('$\dot{\xi}_x^2$','interpreter','latex')

subplot(3,3,9)
plot(Data(3,:),Data(6,:),'*b','MarkerSize',2); hold on;
grid on; 
xlabel('$\xi_x^3$','interpreter','latex')
ylabel('$\dot{\xi}_x^3$','interpreter','latex')

suptitle('Input-output variable interactions')

end

