%% MAIN START HEADER

global Blues Yellows Balls Rules RP PAR Modul activeAlgorithm obstacles

if isempty(RP)
    addpath tools RPtools MODUL
end
%

mainHeader();
%MAP();

if (RP.Pause) 
    return;
end

zMain_End=RP.zMain_End;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%PAR.HALF_FIELD=-1;
%PAR.MAP_X=4000;
%PAR.MAP_Y=3000;
%PAR.RobotSize=200;
%PAR.KICK_DIST=200;
%PAR.DELAY=0.15;
%PAR.WhellR=5;
%PAR.LGate.X=-2000;
%PAR.RGate.X=2000;
%PAR.BorotArm=225;



%% CONTRIL BLOCK

%{
robots = [8, 7, 1, 4, 3, 2];

%{
for k = robots
    RP.Blue(k).Nrul = k;
end
%}
RP.Blue(robots(1)).Nrul = robots(1);
RP.Blue(robots(2)).Nrul = robots(2);

R = 400;
obst = zeros(numel(robots), 3);
for k = 1: numel(robots)
    rid = robots(k);
    obst(k, 1) = RP.Blue(rid).x;
    obst(k, 2) = RP.Blue(rid).y;
    obst(k, 3) = R;
end

%RP.Blue(7).rul = Attacker(RP.Blue(7), RP.Ball, [-2000, 0]);

G1 = [-2270, 147];
G2 = [-283, -168];
RP.Blue(robots(1)).rul = MoveToAvoidance(RP.Blue(robots(1)), G1, obst(2:6, 1:3));
RP.Blue(robots(2)).rul = MoveToAvoidance(RP.Blue(robots(2)), G1, obst([1, 3:6], 1:3));

%RP.Blue(robots(1)).rul = MoveToLinear(RP.Blue(robots(1)), G, 2/750, 30, 50); 
%}


border1 = [-2000 -1500];
border2 = [2000 1500];
point1 = [-1.7683   -0.2785] * 1000;
point2 = [1500 800];

BlueIDs = [3 5 1 2 9];
YellowIDs = [3 8];

commonSize = numel(BlueIDs) + numel(YellowIDs);
if isempty(obstacles) || size(obstacles, 1) < commonSize
    obstacles = zeros(commonSize, 3);
end
        
radius = 220;
eps = 30;

BCnt = numel(BlueIDs);
for k = 1: BCnt;
    id = BlueIDs(k);
    if RP.Blue(id).I == 1 && r_dist_points(RP.Blue(id).z, [obstacles(k, 1), obstacles(k, 2)]) > eps
        obstacles(k, 1) = RP.Blue(id).x;
        obstacles(k, 2) = RP.Blue(id).y;
        obstacles(k, 3) = radius;
    end
end

for k = 1: numel(YellowIDs)
    id = YellowIDs(k);
    if RP.Yellow(id).I == 1 && r_dist_points(RP.Yellow(id).z, [obstacles(k + BCnt, 1), obstacles(k + BCnt, 2)]) > eps
        obstacles(k + BCnt, 1) = RP.Yellow(id).x;
        obstacles(k + BCnt, 2) = RP.Yellow(id).y;
        obstacles(k + BCnt, 3) = radius;
    end
end
%{
RP.Blue(1).Nrul = 1;
RP.Blue(2).Nrul = 2;
RP.Blue(3).Nrul = 3;
RP.Blue(5).Nrul = 5;
%}

%RP.Blue(4).rul = kickBall(RP.Blue(4), RP.Ball, RP.Blue(7).z, kickBallPreparation());
%RP.Blue(4).rul = RotateToPID(RP.Blue(4), RP.Ball.z, 3, 10, 0.1, -30, 0.01);
%Speed1 = StabilizationXPID(RP.Blue(4), -2300, 10, 1/750, 0.000005, -0.8, 50);
%Speed2 = StabilizationYPID(RP.Blue(4), -187, 40, 4/750, 0, -1.5, 100);
%Speed = Speed1;
%RP.Blue(4).rul = Crul(Speed(1), Speed(2), 0, 0, 0);
% RP.Blue(4).rul = RotateToPID(RP.Blue(4), RP.Ball.z, 3, 30, 0, -50, 0.05, false);
%RP.Blue(4).rul = kickBall(RP.Blue(4), RP.Ball, RP.Blue(7).z, kickBallPreparation());
G1 = [-1000, -1200];
G2 = [-1000, 1200];
V = [0 1];
nV = [1 0];

% G1 = [-1800 0];
% G2 = [1800 0];
% V = [1 0];
% nV = [0 1];

%для отладки-----------------------------------------
global BPosHX; %history of ball coordinate X
global BPosHY; %history of ball coordinate Y
global ballFastMoving;
global ballSaveDir;
historyLength = 5;
if isempty(BPosHX) || isempty(BPosHY)
    BPosHX = zeros(1, historyLength);
    BPosHY = zeros(1, historyLength);
end

if isempty(ballFastMoving)
    ballFastMoving = false;
end

if isempty(ballSaveDir)
    ballSaveDir = true;
end
%----------------------------------------------------


        %{
        if (RP.Blue(7).x > border1(1) &&  RP.Blue(7).x < border2(1) && RP.Blue(7).y > border1(2) && RP.Blue(7).x < border2(2))
            RP.Blue(7).rul = catchBall(RP.Blue(7), RP.Ball);
        else 
            RP.Blue(7).rul = Crul(0, 0, 0, 0, 0);
        end
        if (RP.Blue(4).x > border1(1) &&  RP.Blue(4).x < border2(1) && RP.Blue(4).y > border1(2) && RP.Blue(4).x < border2(2))
            RP.Blue(4).rul = attack(RP.Blue(4), RP.Ball, RP.Blue(7).z, ballInside);
        else 
            RP.Blue(4).rul = Crul(0, 0, 0, 0, 0);
        end
        %}        
        %RP.Blue(7).rul = MoveToWithFastBuildPath(RP.Blue(7), RP.Ball.z, 250, obstacles);
        %{
        RP.Blue(4).rul = Crul(0, 0, 0, 0, 0);
        RP.Blue(7).rul = Crul(0, 0, 0, 0, 0);
        %}
        %G = [-1900, 0];
        %[RP.Blue(8).rul, RP.Blue(5).rul, RP.Blue(3).rul] = goBaseStateForGoalKeeperAgainstTwoAttakers(RP.Blue(8), RP.Blue(5), RP.Blue(3), G, V);
        minSpeed = 15;
        P = 4/750;
        D = -1.5;
        vicinity = 50;
            
        RP.Blue(3).rul = Crul(50, 0, 0, 0, 0);
        RP.Blue(2).rul = Crul(50, 0, 0, 0, 0);
        %RP.Blue(7).rul = Crul(50, 50, 0, 0, 0);
        %RP.Blue(8).rul = Crul(50, 50, 0, 0, 0);
        %RP.Blue(1).rul = Crul(50, 50, 0, 0, 0);
        %RP.Blue(4).rul = Crul(50, 50, 0, 0, 0);
        %RP.Blue(5).rul = Crul(50, 50, 0, 0, 0);
        %RP.Blue(6).rul = Crul(50, 50, 0, 0, 0);
        
%         
 %       RP.Blue(3).rul = MoveToPD(RP.Blue(1), G1 , minSpeed, P, D, vicinity);
 %       RP.Blue(2).rul = MoveToPD(RP.Blue(2), G1 + 600 * V, minSpeed, P, D, vicinity);
 %       RP.Blue(8).rul = MoveToPD(RP.Blue(3), G2 , minSpeed, P, D, vicinity);
 %       RP.Blue(4).rul = MoveToPD(RP.Blue(4), G2 - 600 * V, minSpeed, P, D, vicinity);


%RP.Blue(4).rul = GoalKeeperOnLine(RP.Blue(4), RP.Ball, G, V);
%RP.Blue(4).rul = TakeAim(RP.Blue(4), RP.Ball.z, RP.Blue(7).z);
%RP.Blue(7).rul = goAroundPoint(RP.Blue(7), RP.Ball.z, 220, 1000, 5, 40);
%RP.Blue(7).rul = RotateToLinear(RP.Blue(7), RP.Ball.z, 5, 2, 0);
%RP.Blue(7).rul = MoveToLinear(RP.Blue(7), G, 0, 40, 50);

%% END CONTRIL BLOCK

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% MAIN END

%Rules

zMain_End = mainEnd();