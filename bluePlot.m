function  graphRobot=bluePlot(x,y,z,phi,psi,scaleRobot)
% x,y,z      ->  position
% phi        ->  angle orientation
% psi        ->  steering angle
% scaleRobot ->  Scale Robot in mm
%% Load strcuts (faces and vertices of STL files)
load ('blueSTL.mat');           

%% General Rotation matrix in z axis
Rz=[ cos(phi) -sin(phi) 0; 
     sin(phi) cos(phi)  0; 
     0        0         1];

%% Rotation matrix of the Left Wheel on the z axis
Rzl=[ cos(phi+psi) -sin(phi+psi) 0; 
      sin(phi+psi) cos(phi+psi)  0; 
      0            0             1];
%% Traslation of the Left Wheel  
dyl=(1.145*scaleRobot*1000)*sin(phi+0.32);
dxl=(1.145*scaleRobot*1000)*cos(phi+0.32);

%% Rotation matrix of the Right Wheel on the z axis
Rzr=[ cos(phi+psi) -sin(phi+psi) 0; sin(phi+psi) cos(phi+psi) 0; 0 0 1];

%% Traslation of the Right Wheel 
dyr=(1.145*scaleRobot*1000)*sin(phi-0.32);
dxr=(1.145*scaleRobot*1000)*cos(phi-0.32);

%% Rotation matrix of Front axle on the z axis
RzF_axle=[ cos(phi) -sin(phi) 0; 
           sin(phi) cos(phi)  0; 
           0        0         1];
%% Traslation of the Front axle     
dyF_axle=(1.082*scaleRobot*1000)*sin(phi);
dxF_axle=(1.082*scaleRobot*1000)*cos(phi);

%% Rotation matrix of Left Rim on the z axis
Rz_rim_l=[ cos(phi+psi) -sin(phi+psi) 0;
           sin(phi+psi) cos(phi+psi)  0; 
           0            0             1];
       
%% Traslation of the Left Rim         
dyrl=(1.145*scaleRobot*1000)*sin(phi+0.3316);
dxrl=(1.145*scaleRobot*1000)*cos(phi+0.3316);

%% Rotation matrix of Left Rim on the z axis
Rz_rim_r=[ cos(phi+psi) -sin(phi+psi) 0; 
           sin(phi+psi) cos(phi+psi)  0; 
           0            0             1];
%% Traslation of the Rigth Rim         
dyrr=(1.145*scaleRobot*1000)*sin(phi-0.3316);
dxrr=(1.145*scaleRobot*1000)*cos(phi-0.3316);

%% Plot base
robotPatch = Rz*base.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;
graphRobot(1) = patch('Faces',base.faces,'Vertices',robotPatch','FaceColor',[0,0,0.7],'EdgeColor','none');

%% Plot Rear axle
robotPatch = Rz*eje_trasero.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;
graphRobot(2) = patch('Faces',eje_trasero.faces,'Vertices',robotPatch','FaceColor',[0.8,0.8,0.8],'EdgeColor','none');

%% Plot Rear wheels
robotPatch = Rz*r_trasera.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;
graphRobot(3) = patch('Faces',r_trasera.faces,'Vertices',robotPatch','FaceColor','k','EdgeColor','none');

%% Plot Velodyne
robotPatch = Rz*velodyne.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;
graphRobot(4) = patch('Faces',velodyne.faces,'Vertices',robotPatch','FaceColor',[0.8,0.8,0.8],'EdgeColor','none');

%% Plot Front left wheel
robotPatch = Rzl*r_delantera_i.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x+dxl; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y+dyl;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;
graphRobot(5) = patch('Faces',r_delantera_i.faces,'Vertices',robotPatch','FaceColor','k','EdgeColor','none');

%% Plot Front rigth wheel
robotPatch = Rzr*r_delantera_d.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x+dxr; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y+dyr;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;
graphRobot(6) = patch('Faces',r_delantera_d.faces,'Vertices',robotPatch','FaceColor','k','EdgeColor','none');

%% Plot Front axle
robotPatch = RzF_axle*eje_delantero.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x+dxF_axle; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y+dyF_axle;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;
graphRobot(7) = patch('Faces',eje_delantero.faces,'Vertices',robotPatch','FaceColor',[0.8,0.8,0.8],'EdgeColor','none');

%% Plot Front lefth wheel
robotPatch = Rz_rim_l*aro_delantero.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x+dxrl; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y+dyrl;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;
graphRobot(8) = patch('Faces',aro_delantero.faces,'Vertices',robotPatch','FaceColor',[0.8,0.8,0.8],'EdgeColor','none');

%% Plot Front rigth wheel
robotPatch = Rz_rim_r*aro_delantero.vertices';
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x+dxrr; 
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+y+dyrr;
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+z;
graphRobot(9) = patch('Faces',aro_delantero.faces,'Vertices',robotPatch','FaceColor',[0.8,0.8,0.8],'EdgeColor','none');

