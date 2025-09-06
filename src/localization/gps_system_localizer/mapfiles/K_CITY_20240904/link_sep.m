% add option
east_origin = east;
north_origin = north;

east = east_origin(1:34);
north = north_origin(1:34);

LINK_ID = 99;
NEXT_LINK_ID = 120;

guard_zone = 0;

have_to_LangeChange_right = 0;
have_to_LangeChange_left = 0;

left_LaneChange_avail = 0;
right_LaneChange_avail = 0;

RIGHT_LINK_ID = 0;
LEFT_LINK_ID = 0;

is_stop_line = 0;
Speed_Limit = 30;

look_at_signalGroupID = 0;
look_at_IntersectionID = 0;

l = length(east);

station = [0];
dummy = 0;

for i=1:l-1
    dummy = dummy + sqrt((east(i+1) - east(i))^2 + (north(i+1) - north(i))^2);
    station = [station dummy];
end


% Save the data to a .mat file
mat_file_name = sprintf('link_sep/link_%d.mat', LINK_ID);
save(mat_file_name, 'east', 'north', 'have_to_LangeChange_right', 'have_to_LangeChange_left', 'left_LaneChange_avail', ...
    'right_LaneChange_avail', 'LINK_ID', 'NEXT_LINK_ID', 'RIGHT_LINK_ID', 'LEFT_LINK_ID', 'is_stop_line',...
    'Speed_Limit', 'look_at_signalGroupID','look_at_IntersectionID', 'station');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

east = east_origin(34:52);
north = north_origin(34:52);

LINK_ID = 120;
NEXT_LINK_ID = 100;

guard_zone = 0;

have_to_LangeChange_right = 0;
have_to_LangeChange_left = 0;

left_LaneChange_avail = 0;
right_LaneChange_avail = 0;

RIGHT_LINK_ID = 0;
LEFT_LINK_ID = 0;

is_stop_line = 1;
Speed_Limit = 15;

look_at_signalGroupID = 0;
look_at_IntersectionID = 0;

l = length(east);

station = [0];
dummy = 0;

for i=1:l-1
    dummy = dummy + sqrt((east(i+1) - east(i))^2 + (north(i+1) - north(i))^2);
    station = [station dummy];
end


% Save the data to a .mat file
mat_file_name = sprintf('link_sep/link_%d.mat', LINK_ID);
save(mat_file_name, 'east', 'north', 'have_to_LangeChange_right', 'have_to_LangeChange_left', 'left_LaneChange_avail', ...
    'right_LaneChange_avail', 'LINK_ID', 'NEXT_LINK_ID', 'RIGHT_LINK_ID', 'LEFT_LINK_ID', 'is_stop_line',...
    'Speed_Limit', 'look_at_signalGroupID','look_at_IntersectionID', 'station');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

