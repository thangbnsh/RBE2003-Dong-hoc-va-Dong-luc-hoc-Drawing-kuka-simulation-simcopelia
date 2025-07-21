% Number of samples
N = 30;

% Link lengths
d2 = 1;

% Joint angles 
theta1 = linspace(-180, 180, N)*pi/180;
theta2 = linspace(-180, 180, N)*pi/180;
d3 = linspace(0, 1, N);

x = zeros(N, N, N);
y = zeros(N, N, N);
z = zeros(N, N, N);

% Calculate x, y, z using kinematics
for i = 1:N
    for j = 1:N
        for k = 1:N
            x(i,j,k) = cos(theta1(i))*sin(theta2(j))*d3(k) - sin(theta1(i))*d2;
            y(i,j,k) = sin(theta1(i))*sin(theta2(j))*d3(k) + cos(theta1(i))*d2;
            z(i,j,k) = cos(theta2(j))*d3(k); % Adjust for spherical arm
        end
    end
end

% Plotting
scatter3(x(:), y(:), z(:), '.');
xlabel('x (cm)');
ylabel('y (cm)');
zlabel('z (cm)');
title('Workspace of Spherical Arm');
grid on;
axis equal;
