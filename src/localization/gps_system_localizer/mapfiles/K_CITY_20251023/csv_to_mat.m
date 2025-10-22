%for i = 59:61
    % Generate the file name
    i = 61
    file_name = sprintf('link_%d.csv', i);
    
    % Read the CSV file
    data = csvread(file_name);
    
    % Extract east and north coordinates
    east = data(:, 1);
    north = data(:, 2);
        
    % Create a scatter plot
    scatter(east, north);
    hold on; % Hold on to plot all points on the same figure

    % Save the data to a .mat file
    mat_file_name = sprintf('link_%d.mat', i);
    save(mat_file_name, 'east', 'north');
    
    % Clear variables for the next iteration
    clear data east north;
%end

hold off;