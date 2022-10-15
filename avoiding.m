ComId = Com_construct;
OmniDriveId = OmniDrive_construct;
BumperId = Bumper_construct;
OdometryId = Odometry_construct;
DistanceSensor0Id = DistanceSensor_construct(0);
DistanceSensor1Id = DistanceSensor_construct(1);
DistanceSensor8Id = DistanceSensor_construct(8);
%CameraId = Camera_construct;
Com_setAddress(ComId, '127.0.0.1:8080');
Com_connect(ComId);
OmniDrive_setComId(OmniDriveId, ComId);
Bumper_setComId(BumperId, ComId);
DistanceSensor_setComId(DistanceSensor0Id, ComId);
DistanceSensor_setComId(DistanceSensor1Id, ComId);
DistanceSensor_setComId(DistanceSensor8Id, ComId);
%Camera_setComId(CameraId, ComId);
Odometry_setComId( OdometryId, ComId );
xd=2000;
yd=500;
Odometry_set( OdometryId, 0, 0, 0 );

tStart = tic;

while (Bumper_value(BumperId) ~= 1)
    tElapsed = toc(tStart);
    % If 60 seconds are elapsed then exit while loop
    if(tElapsed >= 10 )
        break;
    end;
    
    value0 = DistanceSensor_voltage(DistanceSensor0Id);
    value1 = DistanceSensor_voltage(DistanceSensor1Id);
    value8 = DistanceSensor_voltage(DistanceSensor8Id);
    
    if((0.7 <= value0)|(0.7 <=value1)|(0.7 <= value8))
        % Approaching obstacle
        disp(['you re approaching to an obstacle']);
        
        OmniDrive_setVelocity(OmniDriveId, 100, 0 ,0);
        
         
    else
       % No obstacle ahead
        [x, y, phi]= Odometry_get(OdometryId);
        %disp(['ma position']);
        disp([x,y,phi]);
        ex=xd-x;
        ey=yd-y;
        %disp(['calcul erreur']);
        %disp([ex,ey,phi]);
        KP=1;
        KI=0.1;
        t=0.1;
        Q1=t*ex;
        Q2=t*ey;
        Vx=KP*ex+KI*Q1;
        Vy=KP*ey+KI*Q2;
        %disp(['calcul de vitesse']);
        %disp([Vx,Vy,phi]);
        OmniDrive_setVelocity(OmniDriveId, Vx, Vy ,0);     
    end;
end;
disp(['you re CHANGING MY POSITION ']);
%[x, y, phi]= Odometry_get(OdometryId);
%disp([x,y,phi]);

Com_disconnect(ComId);
Bumper_destroy(BumperId);
OmniDrive_destroy(OmniDriveId);
Odometry_destroy( OdometryId );
Com_destroy(ComId);