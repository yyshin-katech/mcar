clear all
close all
clc

ADD_LINK = 119;
FIRST_POINT_LINK = 1;

from_mat_file_name = sprintf('link_%d.mat', FIRST_POINT_LINK);
go_mat_file_name = sprintf('link_%d.mat', ADD_LINK);

% link_2.mat에서 east 변수의 첫 번째 데이터를 불러오기
data2 = load(from_mat_file_name, 'east');
firstData = data2.east(:, 1); % east 변수의 첫 번째 데이터 (열의 경우)

% link_1.mat에서 east 변수 불러오기
data1 = load(go_mat_file_name, 'east');
eastData = data1.east;

% firstData를 eastData의 마지막에 추가
east = [eastData, firstData]; % 행 배열이므로 열 방향으로 결합

% 새로운 east 변수를 link_1.mat 파일에 저장
save(go_mat_file_name, 'east', '-append');

% link_2.mat에서 east 변수의 첫 번째 데이터를 불러오기
data2 = load(from_mat_file_name, 'north');
firstData = data2.north(:, 1); % east 변수의 첫 번째 데이터 (열의 경우)

% link_1.mat에서 east 변수 불러오기
data1 = load(go_mat_file_name, 'north');
northData = data1.north;

% firstData를 eastData의 마지막에 추가
north = [northData, firstData]; % 행 배열이므로 열 방향으로 결합

% 새로운 east 변수를 link_1.mat 파일에 저장
save(go_mat_file_name, 'north', '-append');

data_from_east = load(go_mat_file_name, 'east');
data_from_north = load(go_mat_file_name, 'north');

east = data_from_east.east;
north = data_from_north.north;

l = length(east);

station = [0];
dummy = 0;

for i=1:l-1
    dummy = dummy + sqrt((east(i+1) - east(i))^2 + (north(i+1) - north(i))^2);
    station = [station dummy];
end

save(go_mat_file_name, 'station', '-append');