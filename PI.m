clear all ;
close all ;
CameraId = Camera_construct;
ComId = Com_construct;
OmniDriveId = OmniDrive_construct;
BumperId = Bumper_construct;
OdometryId = Odometry_construct;
DistanceSensor0Id = DistanceSensor_construct(0);
DistanceSensor1Id = DistanceSensor_construct(1);
DistanceSensor8Id = DistanceSensor_construct(8);
Com_setAddress(ComId, '127.0.0.1:8080');
Com_connect(ComId);
OmniDrive_setComId(OmniDriveId, ComId);
Bumper_setComId(BumperId, ComId);
Odometry_setComId( OdometryId, ComId );
DistanceSensor_setComId(DistanceSensor0Id, ComId);
DistanceSensor_setComId(DistanceSensor1Id, ComId);
DistanceSensor_setComId(DistanceSensor8Id, ComId);
Camera_setComId(CameraId, ComId);
xd=2000;
yd=1500;
Q1=0;
Q2=0;
Odometry_set( OdometryId, 0, 0, 0 );
tStart = tic;
T = table()
Ta = table();
Taa=table();
erreurx=table();
erreury=table();
erreur=table();

while (Bumper_value(BumperId) ~= 1)
    
    tElapsed = toc(tStart);
    % If 60 seconds are elapsed then exit while loop
    if(tElapsed >= 10 )
        break;
    end;
    if ~(Camera_setStreaming(CameraId, 1) == 1)
            disp('Camera_setStreaming failed.');
    end;
    if  (Camera_grab(CameraId) == 1)
        image = Camera_getImage( CameraId );
        load ('netTransfer.mat', 'netTransfer')
        
        I = imresize(image, [227 227]);
        [label,scores] = classify(netTransfer,I);
        imshow(I)
        title( " typ: " +  string(label) + ", " + num2str(100*max(scores),4) + "%");
        title(label);
    end;
     
    
    value0 = DistanceSensor_voltage(DistanceSensor0Id);
    value1 = DistanceSensor_voltage(DistanceSensor1Id);
    value8 = DistanceSensor_voltage(DistanceSensor8Id);
    
    if((0.7 <= value0)|(0.7 <=value1)|(0.7 <= value8))
        % Approaching obstacle
        disp(['you re approaching to an obstacle']);
        
        OmniDrive_setVelocity(OmniDriveId, 100, 0 ,0);   
    else
        
        [x, y, phi]= Odometry_get(OdometryId);
        
        x=[x];
        y=[y];
        result = table(x,y)
        T = [T ; result]
        [ex]=[xd-x];
        [ey]=[yd-y];
        err=table(ex,ey)
        erreur=[erreur ; err];
        errx= table(ex,x)
        erreurx = [erreurx ; errx]
        erry= table(ey,y)
        erreury = [erreury ; erry]
        
        KP=0.5;
        KI=0.01;
        t=0.01;
        Q1=Q1+t*ex;
        Q2=Q2+t*ey;
        Vx=KP*ex+KI*Q1;
        Vy=KP*ey+KI*Q2;
        Vx=[Vx];
        tElapsed=[tElapsed];
        Vy=[Vy];
        resultat = table(Vx,tElapsed)
        Ta = [Ta ; resultat]
        res = table(Vy,tElapsed);
        Taa= [Taa;res];
        OmniDrive_setVelocity(OmniDriveId, Vx, Vy ,0);
       
     end;
      
       
   
end;   
  


figure ;
plot(Ta.tElapsed ,Ta.Vx,Taa.tElapsed,Taa.Vy);
xlabel('t (s)');
title('variation de la vitesse selon axe des x et y en fonction du temps ')



figure ;
plot(Ta.tElapsed,  erreur.ex ,Ta.tElapsed ,erreur.ey );
title(' erreur de suivi en fonction du temps  ')
xlabel('t (s) ');

Bumper_destroy(BumperId);
Camera_destroy(CameraId);
OmniDrive_destroy(OmniDriveId);
Odometry_destroy( OdometryId );
DistanceSensor_destroy(DistanceSensor0Id);
DistanceSensor_destroy(DistanceSensor1Id);
DistanceSensor_destroy(DistanceSensor8Id);
Com_disconnect(ComId);
Com_destroy(ComId);