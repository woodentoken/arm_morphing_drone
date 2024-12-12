
load('StatesVectSampleMissionMinE')
SeqStruct3 = load('SeqStruct3Waypoints.mat')
Positions=[statesvect(1,:);statesvect(2,:);statesvect(3,:);statesvect(7,:)]';

u=statesvect(6,:)*0;
v=-statesvect(6,:);
w=statesvect(10,:);
q0=cos(u/2).*cos(v/2).*cos(w/2)+sin(u/2).*sin(v/2).*sin(w/2);
q1=sin(u/2).*cos(v/2).*cos(w/2)-cos(u/2).*sin(v/2).*sin(w/2);
q2=cos(u/2).*sin(v/2).*cos(w/2)+sin(u/2).*cos(v/2).*sin(w/2);
q3=cos(u/2).*cos(v/2).*sin(w/2)-sin(u/2).*sin(v/2).*cos(w/2);
Angles=[statesvect(1,:);q0;q1;q2;q3]';

waypoints1 = SeqStruct3.SeqStruct.SeqD';
waypoints2 = SeqStruct3.SeqStruct.SeqE';
%waypoints = [waypoints1;waypoints2];
startLocation = [waypoints1(1,:)];
homeLocation = enu2lla(startLocation,referenceLocation,"ellipsoid");


numwpts1 = size(waypoints1);
numwpts2 = size(waypoints2);

vels1 = 2*ones(numwpts1);
accs1 = ones(numwpts1);
jerks1 = zeros(numwpts1);
snaps1 = zeros(numwpts1);
yaws1 = zeros(1,numwpts1(1));
%toas1 = SeqStruct3.SeqStruct.StatesE(1,:);
starttime = 0;
endtime = 30;
toas1 = linspace(starttime,endtime,numwpts1(1));
mrft1 = multirotorFlightTrajectory(waypoints1,vels1,accs1,jerks1,snaps1,yaws1,toas1);
%m = uavMission(mrft1,Frame="LocalENU",Speed=3,InitialYaw=90);
ax1 = show(mrft1,NumSamples=200);
title("Multirotor Flight Trajectory")
view([0 0])


vels2 = 2*ones(numwpts2);
accs2 = ones(numwpts2);
jerks2 = zeros(numwpts2);
snaps2 = zeros(numwpts2);
yaws2 = zeros(1,numwpts2(1));
%toas2 = SeqStruct3.SeqStruct.StatesD(1,:);
toas2 = linspace(starttime,endtime,numwpts2(1));
mrft2 = multirotorFlightTrajectory(waypoints2,vels2,accs2,jerks2,snaps2,yaws2,toas2);
ax2 = show(mrft2,NumSamples=200);
title("Multirotor Flight Trajectory")
view([0 0])



%[positionTbl,rotationTbl,traj] = ShowUAVTrajectory(waypoints1)

[positionTbl,rotationTbl,traj] = ShowUAVTrajectory2(waypoints,Positions,Angles)
