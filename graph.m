% Load data from file
data = load('data.txt');
sensor_x = data(:,1);
sensor_y = data(:,2);
filter_x = data(:,3);
filter_y = data(:,4);

% Calculate axis limits
all_x = [sensor_x; filter_x];
all_y = [sensor_y; filter_y];
x_lim = [min(all_x)-1, max(all_x)+1];
y_lim = [min(all_y)-1, max(all_y)+1];

% Set up figure
figure;
axis([x_lim y_lim]);
xlabel('X Position');
ylabel('Y Position');
title('Sensor Measurements vs Filter Estimate');
grid on;
hold on;

% Initialize empty plots
h_sensor = plot(nan, nan, 'r-', 'MarkerFaceColor', 'r');
h_filter = plot(nan, nan, 'b-', 'LineWidth', 1.5);
legend('Sensor Measurements', 'Filter Estimate', 'Location', 'northeast');

% Animation loop
for i = 1:length(sensor_x)
    % Update sensor data (scatter points)
    set(h_sensor, 'XData', sensor_x(1:i), 'YData', sensor_y(1:i));
    
    % Update filter data (connected line)
    set(h_filter, 'XData', filter_x(1:i), 'YData', filter_y(1:i));
    
    % Update title with current frame
    title(sprintf('Time Step: %d', i));
    
    % Control animation speed (adjust pause duration as needed)
    pause(0.1);
    
    % Force immediate graphics update
    drawnow;
end

hold off;
