%
% File  :   Project 1 - Fuzzy Robot Controller
%
% Author:   Eric Yang
% Date  :   October 4, 2021
%
% Course:   CSE 454
%
% Description:
%       A controller using Fuzzy logic to control a robot to move towards a
%       destination.
%


main();


function main()

    curr = input("Input current location - X,Y\n", 's');
    dest = input("Input target destination - X,Y\n",'s');
    curr = split(curr, ',');
    dest = split(dest, ',');

    curr = str2double(curr);
    dest = str2double(dest);
                    %NOTE: Removes potential excess values from array
    curr = [curr(1:2,1)];
    dest = [dest(1:2,1)];
        
                    %NOTE:stores both the direct value and the current velocity of the angle/distance
    angle = [0;0];
    distance = [0;0];

    iteration = 0;
                    %NOTE: Runs until robot is within a reasonable distance to avoid infinite loops
    while abs(dest(1,1)-curr(1,1))>0.25 || abs(dest(2,1)-curr(2,1))>0.25 
        newValues = ad_calc(curr(1,1),curr(2,1),dest(1,1),dest(2,1),angle(1,1));
        
        aFuzz = aFuzzify(newValues(1,1));
        dFuzz = dFuzzify(newValues(2,1), distance(2,1), -1);
        aDistFuzz = dFuzzify(newValues(1,1),angle(2,1), -45);
        angle = angRules(aFuzz,aDistFuzz,angle);
        distMoved = distRules(dFuzz,distance);
        curr = newPosCalc(curr, angle(1,1),distMoved(1,1));
        distance = [newValues(2,1);distMoved(2,1)];
        
        time = iteration*tf();
        fprintf(""+time+" - "+curr(1,1)+","+curr(2,1)+"\n");
        fprintf("   Angular Velocity: "+angle(2,1)+"degrees/s\n");
        fprintf("   Forward Speed: "+distance(2,1)+"m/s\n");
        fprintf("   Angular Fuzz Weights: "+aFuzz(1,1)+" "+aFuzz(2,1)+" "+aFuzz(3,1)+" "+aFuzz(4,1)+" "+aFuzz(5,1)+"\n");
        fprintf("   Angular Distance Fuzz Weights: "+aDistFuzz(1,1)+" "+aDistFuzz(2,1)+" "+aDistFuzz(3,1)+" "+aDistFuzz(4,1)+" "+aDistFuzz(5,1)+"\n");
        fprintf("   Distance Fuzz Weights: "+dFuzz(1,1)+" "+dFuzz(2,1)+" "+dFuzz(3,1)+" "+dFuzz(4,1)+" "+dFuzz(5,1)+"\n");
        iteration = iteration+1;
    end
    fprintf("Completed - destination: "+dest(1,1)+","+dest(2,1));
end

%Timeframe in use (seconds)
function time = tf()
    time = 0.1;
end


%{
Determines the angle and distance between current direction and destination
Angular difference is calculated by the angle made from connecting the
two points and the x axis and is adjusted for each quadrant.
Angular range: (-180:180]
Inputs:
        current location(x1,y1),
        destination(x2,y2),
        current direction angle(ca)
Outputs:
        angle between current direction and destination(out(1,1),
        distance between current location and destination(out(2,1))
%}
function out = ad_calc(x1,y1,x2,y2,ca)
    xd = x2-x1;
    yd = y2-y1;
    angDest=0;
    if yd==0
        if xd<0
            angDest = -180;
        else
            angDest = 0;
        end
    elseif xd==0
        if yd<0
            angDest = -90;
        else
            angDest = 90;
        end
    else
        angDest = atand(yd/xd);
        if xd<0
            if yd>0
                angDest = angDest + 180;
            else
                angDest = angDest - 180;
            end
        end
    end
    angNew = angDest-ca;
                %NOTE: Adjusts for out of bound angles
    if angNew>180
        angNew = angNew-360;
    elseif angNew<=-180
        angNew = angNew+360;
    end
    dist = sqrt((y2-y1)^2+(x2-x1)^2);
    out = [angNew;dist];
end

%{
Fuzzifies the angle for left and right turning
Membership graph can be seen by uncommenting the plots
Inputs:
        Angle between current direction and destination(angle)
Outputs:
        Fuzzified angle(angFuzzed)
%}
function angFuzzed = aFuzzify(angle)
    syms x;
    a=-144;
    b=-72;
    fRRr(x) = (x-b)/b;
    pwRR = piecewise(x<a,1,a<x<b,fRRr,x>b,0);
    
    a=-144;
    b=-72;
    c=0;
    fRr1(x) = -(x-a)/b;
    fRr2(x) = x/b;
    pwR = piecewise(x<a,0,a<x<b,fRr1,b<x<c,fRr2,x>c,0);
    
    a=-72;
    b=0;
    c=72;
    fNr1(x) = -(x-a)/a;
    fNr2(x) = (x-c)/a;
    pwN = piecewise(x<a,0,a<x<b,fNr1,b<x<c,fNr2,x>c,0);

    a=0;
    b=72;
    c=144;
    fLr1(x) = x/b;
    fLr2(x) = -(x-c)/b;
    pwL = piecewise(x<a,0,a<x<b,fLr1,b<x<c,fLr2,x>c,0);
    
    a=72;
    b=144;
    fLLr(x) = (x-a)/a;
    pwLL = piecewise(x<a,0,a<x<b,fLLr,x>b,1);
    
%     fplot(pwRR,[-180,180]);
%     hold on
%     fplot(pwR,[-180,180]);
%     fplot(pwN,[-180,180]);
%     fplot(pwL,[-180,180]);
%     fplot(pwLL,[-180,180]);
%     hold off

    y=sym(angle);
    f1 = double(fRRr(y));
    fRRmin = min([f1,1]);
    fRRmember = max(fRRmin,0);

    f1 = double(fRr1(y));
    f2 = double(fRr2(y));
    fRmin = min([f1,f2]);
    fRmember = max([fRmin,0]);

    f1 = double(fNr1(y));
    f2 = double(fNr2(y));
    fNmin = min([f1,f2]);
    fNmember = max([fNmin,0]);

    f1 = double(fLr1(y));
    f2 = double(fLr2(y));
    fLmin = min([f1,f2]);
    fLmember = max([fLmin,0]);

    f1 = double(fLLr(y));
    fLLmin = min([f1,1]);
    fLLmember = max([fLLmin,0]);

    angFuzzed = [fRRmember;fRmember;fNmember;fLmember;fLLmember];
end

%{
Application of rules to fuzzy angle and fuzzy angular acceleration
Inputs:
        Fuzzified angle between current and destination(aFuzz),
        Fuzzified angular difference(aDistFuzz),
        Current angle/direction(currAngle(1,1)),
        Current angular velocity(currAngle(2,1)
Outputs:
        Updated angle/direction(newAngle(1,1))
        Updated angular velocity(newAngle(2,1))
%}
function newAngle = angRules(aFuzz, aDistFuzz, currAngle)
    angValues = [-4;-1;0;1;4];
    accelValues = [-90;-45;0;45;90];
    angWeight = aFuzz.*angValues;
    accelWeight = aDistFuzz.*accelValues;
    angAccelValue = sum(accelWeight);
    angDir = sum(angWeight);
    angAccel = angAccelValue*angDir;
    newASpeed = currAngle(2,1) + (angAccel*tf()); %vf = vi + at

                                    %NOTE: Speed is adjusted to 0 if
                                    %deceleration changes speed past 0 to
                                    %avoid moving backwards
    if angAccelValue<0 && newASpeed*currAngle(2,1)<0
        newASpeed = 0;  %stop when decelerated to/past 0
    end
                                    %NOTE: Changes to angle  are instantaneous
    placeholderAngle = currAngle(1,1)+newASpeed*tf();%d = di+vt
                                
    if placeholderAngle > 180
        placeholderAngle = placeholderAngle-360;
    elseif placeholderAngle <=-180
        placeholderAngle = placeholderAngle+360;
    end
    newAngle = [placeholderAngle;newASpeed];
end

%{
Determines minimum distance to start deceleration
Inputs:
        Current speed/angular velocity(vi),
        Deceleration value(a)
Outputs:
        Distance for deceleration(decelDist)
%}
function decelDist = decel_calc(vi, a)
    decelDist = -(vi^2)/(2*a);
end


%fuzzy set for distance and angular difference
%{
Fuzzifies the the current distance from the destination or the angular difference
These membership functions are made variable based on the minimum deceleration distance
The membership graph can be seen by uncommenting the plots
Inputs:
        Distance/angular difference(dist),
        Current speed/angular velocity(v),
        Deceleration value(decel)
Outputs:
        Fuzzified distance/difference(distFuzzed)
%}
function distFuzzed = dFuzzify(dist, v, decel)

    decelDist = decel_calc(v,decel);
    adjDecelDist = max([decelDist,0.1]);

    syms x;
    a=0.5*adjDecelDist;
    b=1*adjDecelDist;
    fM10r(x) = -((x-b)/a);
    pwM10 = piecewise(x<a,1,a<x<b,fM10r,x>b,0);
    
    a=0.5*adjDecelDist;
    b=1*adjDecelDist;
    c=1.5*adjDecelDist;
    fM5r1(x) = (x-a)/a;
    fM5r2(x) = (x-c)/(b-c);
    pwM5 = piecewise(x<a,0,a<x<b,fM5r1,b<x<c,fM5r2,x>c,0);
    
    a=1*adjDecelDist;
    b=1.5*adjDecelDist;
    c=1.5*adjDecelDist;
    d=2*adjDecelDist;
    f0r1(x) = (x-a)/(b-a);
    f0r2(x) = (x-d)/(c-d);
    pw0 = piecewise(x<a,0,a<x<b,f0r1,b<x<c,1,c<x<d,f0r2,x>d,0);
    
    a=1.5*adjDecelDist;
    b=2*adjDecelDist;
    c=3*adjDecelDist;
    d=4*adjDecelDist;
    fP5r1(x) = (x-a)/(b-a);
    fP5r2(x) = (x-d)/(c-d);
    pwP5 = piecewise(x<a,0,a<x<b,fP5r1,b<x<c,1,c<x<d,fP5r2,x>d,0);
    
    a=3*adjDecelDist;
    b=4*adjDecelDist;
    fP10r(x) = (x-a)/(b-a);
    pwP10 = piecewise(x<a,0,a<x<b,fP10r,x>b,1);
    
%     fplot(pwM10,[0,5]);
%     hold on
%     fplot(pwM5,[0,5]);
%     fplot(pw0,[0,5]);
%     fplot(pwP5,[0,5]);
%     fplot(pwP10,[0,5]);
%     hold off
    absDist = abs(dist);
    y = sym(absDist);

    f1 = double(fM10r(y));
    fM10min = min([f1,1]);
    fM10member = max([fM10min,0]);

    f1 = double(fM5r1(y));
    f2 = double(fM5r2(y));
    fM5min = min([f1,f2]);
    fM5member = max([fM5min,0]);

    f1 = double(f0r1(y));
    f2 = double(f0r2(y));
    f0min = min([f1,f2,1]);
    f0member = max([f0min,0]);

    f1 = double(fP5r1(y));
    f2 = double(fP5r2(y));
    fP5min = min([f1,f2,1]);
    fP5member = max([fP5min,0]);

    f1 = double(fP10r(y));
    fP10min = min([f1,1]);
    fP10member = max([fP10min,0]);

    distFuzzed = [fM10member;fM5member;f0member;fP5member;fP10member];
end

%{
Application of rules to fuzzified distance
Inputs:
        Fuzzified distance(fuzzDist),
        Current distance(currDist(1,1)),
        Current speed(currDist(2,1))
Outputs:
        Distance being traversed(distMoved(1,1)),
        New speed(distMoved(2,1))
%}
function distMoved = distRules(fuzzDist, currDist)
    accValues = [-3;-1;0;1;3];
    placeholder = fuzzDist.*accValues;
    accel = sum(placeholder);
    newSpeed = currDist(2,1) + accel * tf();
    newSpeed = max([newSpeed, 0]);  %NOTE: dont decelerate past 0
    placeholderDist = newSpeed*tf();
    distMoved = [placeholderDist;newSpeed];
end

%uses change in distance
%{
Calculates the new position of the robot using below formula
(X0,Y0) -> previous position
New (x,y) -> (X0+distance*cos(angle),Y0+distance*sin(angle))
Inputs:
        Current position(pos),
        Robot direction(angle),
        Distance moved(distance)
Outputs:
        New Position(newPos)
%}
function newPos = newPosCalc(pos,angle,distance)
                        %NOTE: Different quadrants of the graph need
                        %adjustments to the formula to work correctly
    switch true

        case angle==0 || angle==180
            newX = pos(1,1)+distance;
            newPos = [newX;pos(2,1)];

        case angle==90 || angle==-90
            newY = pos(2,1)+distance;
            newPos = [pos(1,1);newY];

        case angle>0 && angle<90        %quadrant 1
            newX = pos(1,1)+distance*cosd(angle);
            newY = pos(2,1)+distance*sind(angle);
            newPos = [newX;newY];

        case angle>90 && angle<180      %quadrant 2
            adjAngle = 180-angle;
            newX = pos(1,1)-distance*cosd(adjAngle);
            newY = pos(2,1)+distance*sind(adjAngle);
            newPos = [newX;newY];

        case angle>-90 && angle<0       %quadrant 4
            adjAngle = abs(angle);
            newX = pos(1,1)+distance*cosd(adjAngle);
            newY = pos(2,1)-distance*sind(adjAngle);
            newPos = [newX;newY];

        case angle>-180 && angle<-90    %quadrant 3
            adjAngle = 180-abs(angle);
            newX = pos(1,1)-distance*cosd(adjAngle);
            newY = pos(2,1)-distance*sind(adjAngle);
            newPos = [newX;newY];
    end
end
