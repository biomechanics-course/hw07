function runshotput
% simulation of Alexander's shotput model
% with vertical motion of elbow and knee
% and knee to elbow activation pattern
% Alexander, R. M. (1991) Optimum timing of muscle activation for simple
% models of throwing. J. theor. Biol. 150: 349-372.

% The following parameters are defined for nested subfunctions:
%   a G g M mball thkdmax thedmax Temax Tkmax turnkneeoff

% a is thigh length; M is body mass
% set them all to 1 to have things computed in dimensionless units
% which means lengths are in multiples of a, masses in M, and
% time in sqrt(a/g)
a = 1; M = 1; g = 1; 

% or, to use real units, just set these values:
%a = 0.44; M = 70; g = 9.81;

% G is force-velocity shape parameter
% thkdmax is the max angular velocity of knee, thedmax of elbow
% describing force-velocity curve in angular velocities
G = 3; thkdmax = 8*sqrt(g/a); thedmax = 8*sqrt(g/a);
% Max torque at knee and elbow, given in terms of M*g*a
Tkmax = 2.5*M*g*a; Temax = 0.7*M*g*a;
% Mass of the shotput, as fraction of total body mass
mball = 0.1*M;

% Here are initial conditions for knee and elbow joints
thk0 = 75*pi/180; thkd0 = 0; % which need to be converted into
the0 = 30*pi/180; thed0 = 0; % y and ydot

% The vertical velocity of the hip and ball depend on the joint velocities
yhip0 = a*(0.2 + 2*sin(thk0/2));
yhipd0 = a*thkd0*cos(thk0/2);
yball0 = a*(1.4 + 2*sin(thk0/2) + 1.6*sin(the0/2));
yballd0 = a*(thkd0*cos(thk0/2) + 0.8*thed0*cos(the0/2));

% Here is the state vector, in terms of y
x0 = [yhip0; yball0; yhipd0; yballd0];

% Only the knee is activated initially, and after a delay tdelay
% the elbow is activated as well

options = odeset('events',@eventreleaseball);

%% Perform a simulation with an intermediate delay time
tdelay = 0.6*sqrt(a/g);
% first integrate equations of motion for knee only
turnkneeoff = 0;
[t1,x1] = ode45(@fshotput1, [0 tdelay], x0); % knee only
% and now integrate from where the previous simulation left off,
% but now there is both a knee and elbow joint
[t2,x2,te,ye,ie] = ode45(@fshotput2, [tdelay 2], x1(end,:)', options); % knee and elbow
% if knee reaches maximum angular velocity, keep knee torque zero
% but continue with elbow only
t3 = []; x3 = []; % store elbow-only simulation here
if ie == 1        % knee-max-speed detected
    turnkneeoff = 1;
    [t3,x3,te,ye,ie] = ode45(@fshotput2, [t2(end) 2], x2(end,:)', options); % elbow only
end

t = [t1;t2;t3]; % concatenate all segments of simulation
x = [x1;x2;x3];

% Translational states
yhip = x(:,1);
yball = x(:,2);
yhipd = x(:,3);
yballd = x(:,4);

% Use translational states to compute joint angles and velocities
qandqdots = invkinematics(x);
thk = qandqdots(:,1);  % theta-knee
the = qandqdots(:,2);  % theta-elbow
thkd = qandqdots(:,3); % theta-knee-dot
thed = qandqdots(:,4); % theta-elbow-dot
if turnkneeoff
    thk(end-length(t3)+1:end) = NaN;
    thkd(end-length(t3)+1:end) = NaN;
end

subplot(121)
plot(t/sqrt(a/g), [thk the], t, [thkd thed]);
xlabel('time sqrt(a/g)'); ylabel('theta and thetadots'); legend('knee', 'elbow', 'knee vel', 'elbow vel')
subplot(122)
plot(t/sqrt(a/g), x)
xlabel('time sqrt(a/g)'); ylabel('y and ydots'); legend('yhip', 'yball', 'yhipd', 'yballd')
title('Intermediate delay')

fprintf(1, 'Delay = %4.2f\n', tdelay/sqrt(a/g));
fprintf(1, 'End time = %4.2f\n', t(end));         % total time of simulation
tlegs = %% ADD YOUR CODE HERE TO COMPUTE LEG TIME (STARTING AT TIME 0, ENDING AT KNEE OR ELBOW MAX SPEED WHICHEVER FIRST)
tarms = %% ADD YOUR CODE HERE TO COMPUTE ARM TIME (STARTING AFTER DELAY)
fprintf(1, 'Extension time for legs = %4.2f\n', tlegs); 
fprintf(1, 'Extension time for arms = %4.2f\n', tarms);
fprintf(1, 'Final ball speed = %4.2f\n', yballd(end));
energyball = mball*g*(yball-yball0) + 1/2*mball*yballd.^2;
fprintf(1, 'Final ball energy gained = %4.2f\n\n', energyball(end));

disp('press a key to continue')
pause

%% Increase maximum shortening speed of knee
fprintf(1, '\nIncrease max shortening speed of knee')
% G is force-velocity shape parameter
% thkdmax is the max angular velocity of knee, thedmax of elbow
% describing force-velocity curve in angular velocities
G = 3; 
%% VARY MAX SHORTENING SPEEDS HERE
thkdmax = 8*sqrt(g/a); thedmax = 8*sqrt(g/a);
% Max torque at knee and elbow, given in terms of M*g*a
Tkmax = 2.5*M*g*a; Temax = 0.7*M*g*a;
% Mass of the shotput, as fraction of total body mass
mball = 0.1*M;
% Here are initial conditions for knee and elbow joints
thk0 = 75*pi/180; thkd0 = 0;
the0 = 30*pi/180; thed0 = 0;

% The vertical velocity of the hip and ball depend on the joint velocities
yhip0 = a*(0.2 + 2*sin(thk0/2));
yhipd0 = a*thkd0*cos(thk0/2);
yball0 = a*(1.4 + 2*sin(thk0/2) + 1.6*sin(the0/2));
yballd0 = a*(thkd0*cos(thk0/2) + 0.8*thed0*cos(the0/2));

% Here is the state vector
x0 = [yhip0; yball0; yhipd0; yballd0];

% Only the knee is activated initially, and after a delay tdelay
% the elbow is activated as well

options = odeset('events',@eventreleaseball);

% Perform a simulation with an intermediate delay time
tdelay = 1*sqrt(a/g);
% first integrate equations of motion for knee only
turnkneeoff = 0;
[t1,x1] = ode45(@fshotput1, [0 tdelay], x0);
% and now integrate from where the previous simulation left off,
% but now there is both a knee and elbow joint
[t2,x2,te,ye,ie] = ode45(@fshotput2, [tdelay 2], x1(end,:)', options);
t3 = []; x3 = [];
if ie == 1 % knee reached max angular velocity, so continue for a bit
    turnkneeoff = 1;
    [t3,x3,te,ye,ie] = ode45(@fshotput2, [t2(end) 2], x2(end,:)', options);
end

t = [t1;t2;t3];
x = [x1;x2;x3];

qandqdots = invkinematics(x);
thk = qandqdots(:,1); %thk(end-length(t3)+1:end) = NaN;
the = qandqdots(:,2);
thkd = qandqdots(:,3); %thkd(end-length(t3)+1:end) = NaN;
thed = qandqdots(:,4);
if turnkneeoff
    thk(end-length(t3)+1:end) = NaN;
    thkd(end-length(t3)+1:end) = NaN;
end

yhip = x(:,1);
yball = x(:,2);
yhipd = x(:,3);
yballdd = x(:,4);

subplot(121)
plot(t/sqrt(a/g), [thk the], t, [thkd thed]);
xlabel('time sqrt(a/g)'); ylabel('theta and thetadots'); legend('knee', 'elbow', 'knee vel', 'elbow vel')
subplot(122)
plot(t/sqrt(a/g), x)
xlabel('time sqrt(a/g)'); ylabel('y and ydots'); legend('yhip', 'yball', 'yhipd', 'yballd')
title('Faster max speed')

fprintf(1, 'Delay = %4.2f\n', tdelay/sqrt(a/g));
fprintf(1, 'End time = %4.2f\n', t(end));
tlegs = %% ADD YOUR CODE HERE TO COMPUTE LEG TIME (STARTING AT TIME 0, ENDING AT KNEE OR ELBOW MAX SPEED WHICHEVER FIRST)
tarms = %% ADD YOUR CODE HERE TO COMPUTE ARM TIME (STARTING AFTER DELAY)
fprintf(1, 'Extension time for legs = %4.2f\n', tlegs); 
fprintf(1, 'Extension time for arms = %4.2f\n', tarms);
fprintf(1, 'Final ball speed = %4.2f\n', yballd(end));
energyball = mball*g*(yball-yball0) + 1/2*mball*yballd.^2;
fprintf(1, 'Final ball energy gained = %4.2f\n', energyball(end));

disp('Press a key to continue')
pause

%% MAKE ADDITIONAL PLOTS FOR SHORT AND LONG DELAY TIMES
%% Vary delay time and missile mass

fprintf(1,'\nVary both delay time and missile mass\n');
% Use nominal parameters for all cases:
G = 3; thkdmax = 8*sqrt(g/a); thedmax = 8*sqrt(g/a);
Tkmax = 2.5*M*g*a; Temax = 0.7*M*g*a;
thk0 = 75*pi/180; thkd0 = 0;
the0 = 30*pi/180; thed0 = 0;

% An array of masses and delays
mballs = [0.08 0.1 0.15]*M;        %% PUT IN APPROPRIATE BALL MASSES HERE
tdelays = (0.6:0.05:0.8)*sqrt(a/g); %% PUT IN APPROPRIATE DELAYS HERE

clf; hold on;
for i = 1:length(mballs)
    for j = 1:length(tdelays)
        x0 = fwdkinematics([thk0, the0, thkd0, thed0])';
        options = odeset('events',@eventreleaseball);
        tdelay = tdelays(j); 
        mball = mballs(i);
        turnkneeoff = 0;
        [t1,x1] = ode45(@fshotput1, [0 tdelay], x0);
        [t2,x2,te,ye,ie] = ode45(@fshotput2, [tdelay 2], x1(end,:)', options);
        t3 = []; x3 = [];
        if ie == 1 % knee reached max angular velocity, so continue for a bit
            turnkneeoff = 1;
            [t3,x3,te,ye,ie] = ode45(@fshotput2, [t2(end) 2], x2(end,:)', options);
        end
        t = [t1;t2;t3];
        x = [x1;x2;x3];
        yball = x(:,2);
        yballd = x(:,4);
        energyball = g*yball + 1/2*yballd.^2; % mass normalized
        energyballs(i,j) = energyball(end);
    end
    hp(i) = plot(tdelays, energyballs(i,:));
    balllabel{i} = sprintf('mball = %3.2f', mballs(i));
end
legend(hp, balllabel)
xlabel('time delay');
ylabel('ball energy (mass-normalized)')
title('Varying ball mass and delay');
% end of main runshotput code; subfunctions follow

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function yandydots = fwdkinematics(qandqdots)
% fwdkinematics converts joint angles for knee and elbow (thk and the)
% into vertical position and velocity of hip and ball
    thk = qandqdots(:,1);
    the = qandqdots(:,2);
    thkd = qandqdots(:,3);
    thed = qandqdots(:,4);
    
    yhip = a*(0.2 + 2*sin(thk/2));
    yball = a*(1.4 + 2*sin(thk/2) + 1.6*sin(the/2));
    yhipd = a*thkd.*cos(thk/2);
    yballd = a*(thkd.*cos(thk/2) + 0.8*thed.*cos(the/2));
    yandydots = [yhip yball yhipd yballd];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function qandqdots = invkinematics(yandydots)
% invkinematics converts vertical position and velocity of hip and ball
% into corresponding joint angles for knee and elbow (thk and the)
    yhip = yandydots(:,1);
    yball = yandydots(:,2);
    yhipd = yandydots(:,3);
    yballd = yandydots(:,4);
    thk = 2*asin(1/2*(yhip/a-0.2));             % yhip = a*(0.2 + 2*sin(thk/2));
    the = 2*asin(((yball-yhip)/a-1.2)/1.6);     % yball = a*(1.4 + 2*sin(thk/2) + 1.6*sin(the/2));
    thkd = yhipd/a./(cos(thk/2));               % yhipd = a*thkd.*cos(thk/2);
    thed = (yballd-yhipd)./(a*0.8*cos(the/2));  % yballd = a*(thkd.*cos(thk/2) + 0.8*thed.*cos(the/2));
    if length(yandydots)==4 && turnkneeoff
        thk = pi;
        thkd = 0;
    end
    qandqdots = [thk the thkd thed];
end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xdot = fshotput1(t, x)
% state-derivative for Alexander's shotput model, with knee only (elbow isometric)

% The following parameters should be defined in the workspace:
%   a G g M mball thkdmax Tkmax Temax

xdot = zeros(4,1); % yhip yball yhipd yballd

yhip = x(1);
yball = x(2);
yhipd = x(3);
yballd = x(4);

qandqdots = invkinematics(x(:)'); 
thk = qandqdots(1);
the = qandqdots(2);
thkd = qandqdots(3);
thed = qandqdots(4);

% Calculate the torque from the torque-velocity curve for the knee
if thkd < thkdmax, Tk = Tkmax*(thkdmax-thkd)/(thkdmax+G*thkd); else Tk = 0; end

% Convert knee torque to the vertical force applied by legs to the hip
Fhip = Tk / (a*cos(thk/2));

% And then calculate the acceleration of the hip (and body) as a single mass
yhipdd = Fhip / (M + mball) - g;

% When elbow is locked, the ball moves as fast as the hip
yballdd = yhipdd;

xdot = [yhipd; yballd; yhipdd; yballdd];

end % fshotput1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xdot = fshotput2(t, x)
% state-derivative for Alexander's shotput model, with knee only (no elbow)

% The following parameters should be defined in the workspace:
%   a G g M mball thkdmax Tkmax Temax turnkneeoff

yhip = x(1);
yball = x(2);
yhipd = x(3);
yballd = x(4);

qandqdots = invkinematics(x(:)'); 
thk = qandqdots(1);
the = qandqdots(2);
thkd = qandqdots(3);
thed = qandqdots(4);

% Calculate the torque from the torque-velocity curve for the knee and elbow
if thkd < thkdmax, Tk = Tkmax*(thkdmax-thkd)/(thkdmax+G*thkd); else Tk = 0; end
if thed < thedmax, Te = Temax*(thedmax-thed)/(thedmax+G*thed); else Te = 0; end

% Convert knee torque to the vertical force applied by legs to the hip (and body)
Fhip = Tk / (a*cos(thk/2));

if turnkneeoff, Fhip = 0; end; % treat knee as zero torque

% Convert elbow torque to the vertical force applied by arms to ball
Fball = Te / (0.8*a*cos(the/2));

% And then calculate the acceleration of the hip (and body) as a single mass
% Note that legs produce upward vertical force, and reaction force from arms
% push down
yhipdd = (Fhip - Fball)/M - g;
% and the force from arms also pushes up on the ball itself
yballdd = Fball/mball - g;

xdot = [yhipd; yballd; yhipdd; yballdd];

end % fshotput2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [value, isterminal, direction] = eventreleaseball(t, x)
% check for ball release conditions:
%   Force on ball goes to zero
%   Knee reaches full extension
%   Elbow reaches full extension

% The following parameters should be defined in the workspace:
%   a G g M mball thkdmax Tkmax Temax

yhip = x(1);
yball = x(2);
yhipd = x(3);
yballd = x(4);

qandqdots = invkinematics(x(:)'); 
thk = qandqdots(1);
the = qandqdots(2);
thkd = qandqdots(3);
thed = qandqdots(4);

% Calculate the torque from the torque-velocity curve for the knee and elbow
if thkd < thkdmax, Tk = Tkmax*(thkdmax-thkd)/(thkdmax+G*thkd); else Tk = 0; end
if thed < thedmax, Te = Temax*(thedmax-thed)/(thedmax+G*thed); else Te = 0; end

% Convert knee torque to the vertical force applied by legs to the hip (and body)
Fhip = Tk / (a*cos(thk/2));
% Convert elbow torque to the vertical force applied by arms to ball
Fball = Te / (0.8*a*cos(the/2));

if turnkneeoff
    Fhip = 0;
end

% And then calculate the acceleration of the hip (and body) as a single mass
% Note that legs produce upward vertical force, and reaction force from arms
% push down
yhipdd = (Fhip - Fball)/M - g;
% and the force from arms also pushes up on the ball itself
yballdd = Fball/mball - g;

value = [thkd-thkdmax; thed-thedmax]; % check for when joints hit max speed
isterminal = [1; 1];   % stop when any event occurs
direction = [0; 0];   % regardless of slope

end % eventreleaseball
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end % runshotput file
