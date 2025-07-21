clear; clc;
%% Infor: This code is refered to https://github.com/MohamedRaslan/UR10PickAndPlace
%%% Note:
    % You Need to install Matlab Robotics Toolbox by Peter Corke
    % URL:['http://petercorke.com/wordpress/toolboxes/robotics-toolbox']
    % Or you can Simply(From within the MATLAB file browser double click on 
    % The file(Robotics Toolbox for MATLAB.mltbx) ,it will install and configure
    %          the paths correctly) ... You can find this file
    %          inside(RoboticsToolbox_installation)Folder

%% Include Path and M.Files 
addpath('..');
addpath('../VrepConnection','../Mfunctions/Robotics_Functions',...
    '../Mfunctions/URn_Functions','../Mfunctions/Vrep_Functions');
  
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);      % just in case, close all opened connections
id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (id < 0)
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end

fprintf('Connection %d to remote API server open.\n', id);

%% GetObjectHandle 
%%% For the UR10 Joints 
handles = struct('id',id);
jointNames={'UR10_joint1','UR10_joint2','UR10_joint3','UR10_joint4',...
    'UR10_joint5','UR10_joint6'};

handles.ur10Joints = -ones(1,6); 
for i = 1:6
    [res, handles.ur10Joints(i)] = vrep.simxGetObjectHandle(id, ...
                     jointNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
end
%%% For the Cuboids & Tables
CuboidNames={'Left_Cuboid','Front_Cuboid','Right_Cuboid'};

TableNames={'Left_Table','Front_Table','Right_Table'};

handles.ur10Cuboids = -ones(1,3); 
handles.ur10Tables = -ones(1,3); 

 for i = 1:3
    [res, handles.ur10Cuboids(i)] = vrep.simxGetObjectHandle(id,...
                    CuboidNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
    
    [res, handles.ur10Tables(i)] = vrep.simxGetObjectHandle(id,...
                    TableNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);       
end

%%% For the Base frame [Frame0]
handles.base=-1;
[res, handles.base] = vrep.simxGetObjectHandle(id, ...
    'Frame0', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);
%%% For the Target frame [Frame1]
handles.Target=-1;
[res, handles.Target] = vrep.simxGetObjectHandle(id, ...
    'Frame1', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

%%% For the EndGripper And AngEndGeipperJoints
handles.EndGripper= -1 ;
    [res, handles.EndGripper] = vrep.simxGetObjectHandle(id, ...
                  'RG2_openCloseJoint', vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);

%% Stream ...

%%% Cuboids [position/orientation & Tables [position/orientation]
relativToRef = handles.base;
for i = 1:3
    %Cuboids
    res = vrep.simxGetObjectPosition(id,handles.ur10Cuboids(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
    res = vrep.simxGetObjectOrientation(id,handles.ur10Cuboids(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);  
    
    % Tables
    res = vrep.simxGetObjectPosition(id,handles.ur10Tables(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
    res = vrep.simxGetObjectOrientation(id,handles.ur10Tables(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
end


% %%%% The UR10 Joints
for i = 1:6
    res= vrep.simxGetJointPosition(id, handles.ur10Joints(i),...
               vrep.simx_opmode_streaming); 
    vrchk(vrep, res, true);
end

%%% EndGripper & AngEndGeipperJoints    
res = vrep.simxGetJointPosition(id,handles.EndGripper,...
         vrep.simx_opmode_streaming); 
vrchk(vrep, res, true);


%% Start simulation
% to make sure that streaming data has reached to client at least once
vrep.simxGetPingTime(id);
vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait)
pause(0.0001);

% Set the threshold to check if the end effector has reached its destination
handles.threshold = 0.01;
%Set The Arm Parameters Using Peter Corke robotics toolbox
handles.ur10Robot= URnSerial_fwdtrans('UR10');

%Rest Joint for 1st time
handles.startingJoints = [0,0,0,0,0,0];
        res = vrep.simxPauseCommunication(id, true);
        vrchk(vrep, res);
    for j = 1:6
            vrep.simxSetJointTargetPosition(id, handles.ur10Joints(j),...
                  handles.startingJoints(j),vrep.simx_opmode_oneshot);
            vrchk(vrep, res);
    end
        res = vrep.simxPauseCommunication(id, false);
        vrchk(vrep, res);
pause(1);

%main program
ConstValue=0.25; % Do cao cua mat ban dat robot
iTable=1;  % 1 for 'Left_Table', 2 for'Front_Table', 3 for'Right_Table'
iCuboid=1; % 1 for 'Left_Cuboid', 2 for'Front_Cuboid', 3 for'Right_Cuboid'
XYZoffset= [0 0 0]; % [ X Y Z]

q=[0, pi/6, pi/4, 0 ,pi,pi/2];  % joint angles
MoveUR10Joints(id, vrep, handles, q );

mode=1;
switch mode
    case 1
        T=handles.ur10Robot.fkine(q);
        T06=zeros(4,4);
        T06(:,1)=[T.n;0];
        T06(:,2)=[T.o;0];
        T06(:,3)=[T.a;0];
        T06(:,4)=[T.t;1];        
    case 2
        T06=UR10_forwardkinematic(q);
end

moveFrame( id, vrep, T06, handles.Target, relativToRef);

vrep.simxPauseSimulation(id, vrep.simx_opmode_oneshot_wait);
%% END ..
vrep.delete;



