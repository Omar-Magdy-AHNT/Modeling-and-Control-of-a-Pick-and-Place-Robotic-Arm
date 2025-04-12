

a = arduino();  % Connect to Arduino
servos = [servo(a, 'D9'), servo(a, 'D10'), servo(a, 'D11')]; % Define servo objects for each joint
qeef=servo(a,'D12');
writePosition(qeef,0.6);
ps3=-45;
p1 = (-50+ 90) / 180;
p2 = (45+90)/180;

p3 = (ps3+90)/180;



pause(1);
pinState =0;  
gRP = servo(a, 'D5');
pause(1);
writePosition(gRP,1);
pause(1);
writePosition(gRP,0.75);
pause(1);
writePosition(gRP,1);
pause(1);
load("BackToZero.mat");
load("PickUpTrajec.mat");
load("PlaceTrajec.mat");

% Loop through each row of the trajectory_pickup
for i = 1:size(PickUpTrajec, 1)  % Iterate through rows of the trajectory_pickup array
    joint_angles = PickUpTrajec(i, :);  % Get joint angles for this step (in degrees)
    % Scale joint angles from degrees to servo-compatible range [0, 1]
    servo_positions = (joint_angles + 90) / 180;  % Map [-180°, 180°] to [0, 1]
    
    % Send positions to servos
    for j = 1:length(servos)
        writePosition(servos(j), servo_positions(j));
    end
   
    pause(0.008);  % Pause to match your sampling time
end
pause(1);
writePosition(gRP,0);
disp('GRIPPYYYY');
pause(1);

for i = 1:size(PlaceTrajec, 1)  % Iterate through rows of the trajectory_pickup array
    joint_angles = PlaceTrajec(i, :);  % Get joint angles for this step (in degrees)
    % Scale joint angles from degrees to servo-compatible range [0, 1]
    servo_positions = (joint_angles + 90) / 180;  % Map [-180°, 180°] to [0, 1]
    
    % Send positions to servos
    for j = 1:length(servos)
        writePosition(servos(j), servo_positions(j));
    end
   
    pause(0.008);  % Pause to match your sampling time
end
pause(1);
writePosition(gRP,1);
disp('NOTTTT    GRIPPYYYY');
%YA FALAAAAA7%
pause(1);
%pause(15);
writePosition(gRP,0.75);




    
for i = 1:size(BackToOrigin, 1)  % Iterate through rows of the trajectory_pickup array
    joint_angles = BackToOrigin(i, :);  % Get joint angles for this step (in degrees)
    
    % Scale joint angles from degrees to servo-compatible range [0, 1]
    servo_positions = (joint_angles + 90) / 180;  % Map [-180°, 180°] to [0, 1]
    % Send positions to servos
    for j = 1:length(servos)
        writePosition(servos(j), servo_positions(j));
    end
   
    pause(0.004);  % Pause to match your sampling time
end
pause(2);
%writePosition(gRP,1);

%writePosition(gRP,0.2);


q1 = servos(1);
q2 = servos(2);
q3 = servos(3);
writePosition(q1, p1);
writePosition(q2, p2);
writePosition(q3, p3);
pause(1);
writePosition(gRP,1);
pause(1);
writePosition(gRP,0.3);

ps3=(-45)-15;
p3 = (ps3+90)/180;
pause(0.75);
writePosition(q3, p3);
ps3=(-45)+15;
p3 = (ps3+90)/180;
pause(0.75);
writePosition(q3, p3);
ps3=(-45)-15;
p3 = (ps3+90)/180;
pause(0.75);
writePosition(q3, p3);
ps3=(-45)+15;
p3 = (ps3+90)/180;
pause(0.75);
writePosition(q3, p3);
ps3=(-45)-15;
p3 = (ps3+90)/180;
pause(0.75);
writePosition(q3, p3);

disp('We are here, We are now');
