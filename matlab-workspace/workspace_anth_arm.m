% Number of samples
N = 30;

% Link lengths
a1 = 0;
a2 = 1;
a3 = 1;

% Joint angles 
theta1 = linspace(-180, 180, N)*pi/180;
theta2 = linspace(-180, 180, N)*pi/180;
theta3 = linspace(-180, 180, N)*pi/180;

x = zeros(N, N, N);
y = zeros(N, N, N);
z = zeros(N, N, N);

% Calculate x, y, z using kinematics
for i = 1:N
    for j = 1:N
        for k = 1:N
            %x(i,j,k) = cos(theta1(i))*(a2*cos(theta2(j)) + a3*cos(theta2(j) + theta3(k)));
            %y(i,j,k) = sin(theta1(i))*(a2*cos(theta2(j)) + a3*cos(theta2(j) + theta3(k)));
            %z(i,j,k) = a1 + a2*sin(theta2(j)) + a3 * sin(theta2(j) + theta3(k)); % Corrected calculation
            x(i,j,k) = cos(theta1(i))*cos(theta2(j))*a3*cos(theta3(k)) - cos(theta1(i))*sin(theta2(j))*a3*sin(theta3(k)) + cos(theta1(i))*a2*cos(theta2(j));
            y(i,j,k) = sin(theta1(i))*cos(theta2(j))*a3*cos(theta3(k)) - sin(theta1(i))*sin(theta2(j))*a3*sin(theta3(k)) + sin(theta1(i))*a2*cos(theta2(j));
            z(i,j,k) = sin(theta2(j))*a3*cos(theta3(k)) + sin(theta2(j))*a3*sin(theta3(k)) + a2*sin(theta2(j));
        end
    end
end

% Plotting
scatter3(x(:), y(:), z(:), '.');
xlabel('x (cm)');
ylabel('y (cm)');
zlabel('z (cm)');
title('Workspace of Anthropomorphic Arm');
grid on;
axis equal;
