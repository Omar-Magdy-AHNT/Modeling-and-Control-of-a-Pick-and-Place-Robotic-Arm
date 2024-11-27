

a = arduino();  % Connect to Arduino
servos = [servo(a, 'D9'), servo(a, 'D10'), servo(a, 'D11')]; % Define servo objects for each joint
writePosition(servo(a,'D12'),0.6);
gRP = servo(a, 'D5');

pause(1);
writePosition(gRP,1);
pause(1);
writePosition(gRP,0);
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
pause(1);

for i = 1:size(BackToOrigin, 1)  % Iterate through rows of the trajectory_pickup array
    joint_angles = BackToOrigin(i, :);  % Get joint angles for this step (in degrees)
    
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
pause(1);
disp('We are here, We are now');
