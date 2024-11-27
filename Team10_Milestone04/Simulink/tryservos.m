% Clear previous connections and workspace
clear;
clc;

% Create an Arduino object
a = arduino();

% Define the servo object for pin D12
servoPin = 'D5'; % Specify the pin
gRP = servo(a, 'D5');
q1 = servo(a, 'D9');
q2 = servo(a, 'D10');
q3 = servo(a, 'D11');
q4= servo(a,'D12');
% Test the servo with a specific angle
angle = 0; % Desired angle in degrees (between 0 and 180)

% Convert angle to a normalized position (0 to 1)
position = angle / 180;

writePosition(q4, 0.6);
writePosition(q2, 0.7777777777777778);
writePosition(q3, 0.01);
writePosition(gRP, 1);
pause(2);
writePosition(gRP, 0);


% Move the servo to the specified position
% writePosition(gRP, 0);
% pause(1);
% writePosition(q1, 0.5);
% pause(1);
% writePosition(q2, 0.25);
% pause(1);
% writePosition(q3, 0.25);
% pause(1);
% writePosition(gRP, 1);
% pause(1);
% writePosition(q1, 0.75);
% pause(1);
% writePosition(q2, 0.1);
% pause(1);
% writePosition(q3, 0.5);
% pause(1);
% writePosition(gRP, 0);
% Display the angle for confirmation
disp(['Servo moved to ', num2str(angle), ' degrees.']);

% Pause to observe servo motion
pause(2);

% Reset the servo to 0 degrees (optional)

disp('Servo reset to 0 degrees.');
