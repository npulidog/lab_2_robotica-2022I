clc, clear all, close all

% Laboratorio 2 - Robótica: Cinemática Directa - Phantom X - ROS
% Nicolas Pulido Gerena

%% Toolbox
% Se crea el robot utilizando SerialLink y los parametros DH
L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',14.5,'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',10.5,'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',10.5,'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
Robot = SerialLink(L,'name','px');

%Se orienta la herramienta utilizando la convención NOA
Robot.tool = [0 0 1 9; -1 0 0 0; 0 -1 0 0; 0 0 0 1];

%Se grafica el Robot
Robot.plot([0 0 0 0], 'notiles', 'noname');
hold on
trplot(eye(4),'rgb','arrow','length',15,'frame','0')
ws = [-50 50];
axis([repmat(ws,1,2) 0 60])
Robot.teach()

% Calculo de la MTH
syms q1 q2 q3 q4 as real

T1_0 = L(1).A(q1);
T1_0(1,2) = 0;
T1_0(2,2) = 0;
T1_0(3,3) = 0;
T2_1 = L(2).A(q2)
T3_2 = L(3).A(q3) 
T4_3 = L(4).A(q4)

MTH = simplify(T1_0*T2_1*T3_2*T4_3*Robot.tool)

% Diferentes posiciones del Robot
% Posición 1
p1 = deg2rad([0 60 -45 90]);
Robot.plot(p1, 'notiles', 'noname');

% Posición 2
p2 = deg2rad([45 -45 90 45]);
Robot.plot(p2, 'notiles', 'noname');

% Posición 3
p3 = deg2rad([60 -90 0 -30]);
Robot.plot(p3, 'notiles', 'noname');


%% Matlab + ROS + Toolbox
rosinit;

% Valores de configuración
q1 = [deg2rad([90 0 0 0]) 0];
q2 = [deg2rad([-20 20 -20 20]) 0];
q3 = [deg2rad([30 -30 30 -30]) 0];
q4 = [deg2rad([-90 15 -55 17]) 0];
q5 = [deg2rad([-90 45 -55 45]) 0];

% Movimiento con q1
Robot.plot(q1(1:4), 'notiles', 'noname');
moveRobot(q1)
% Movimiento con q2
Robot.plot(q2(1:4), 'notiles', 'noname');
moveRobot(q2)
% Movimiento con q3
Robot.plot(q3(1:4), 'notiles', 'noname');
moveRobot(q3)
% Movimiento con q4
Robot.plot(q4(1:4), 'notiles', 'noname');
moveRobot(q4)
% Movimiento con q5
Robot.plot(q5(1:4), 'notiles', 'noname');
moveRobot(q5)

%% Conexión Matlab-ROS

jointSub = rossubscriber('/dynamixel_workbench/joint_states', 'DataFormat','struct');
[msgSub,status,statustext] = receive(jointSub,10); 

disp("Angle in radians for each joint:")
disp(" ")

for i = 1:5
    disp("Joint" + i + ": " + msgSub.Position(i))
end


rosshutdown;

%% Función para mover las juntas y el gripper
function output = moveRobot(q)
    offsetID = 0;
    motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command');
    motorCommandMsg = rosmessage(motorSvcClient);

    motorCommandMsg.AddrName = "Torque_Limit";
    torque = [600, 400, 400, 400, 400];
    for i= 1: length(q) 
        motorCommandMsg.Id = i+offsetID;
        motorCommandMsg.Value = torque(i);
        call(motorSvcClient,motorCommandMsg);
    end
   
    motorCommandMsg.AddrName = "Goal_Position";
    for i= 1: length(q)
        disp(i)
        motorCommandMsg.Id = i+offsetID;
        disp(round(mapfun(rad2deg(q(i)),-150,150,0,1023)))
        motorCommandMsg.Value = round(mapfun(rad2deg(q(i)),-150,150,0,1023));
        call(motorSvcClient,motorCommandMsg); 
        pause(1);
    end
    
end