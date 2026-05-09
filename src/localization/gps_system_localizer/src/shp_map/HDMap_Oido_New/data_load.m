clear;clc;close all;
   
S = m_shaperead('A3_DRIVEWAYSECTION' ); 
for j=1:length(S.ncst)
plot3(S.ncst{j, 1}(:,1),S.ncst{j, 1}(:,2),S.ncst{j, 1}(:,3),'r');hold on;
view(2);
end
% axis equal;

S = m_shaperead('A1_NODE' ); 
for j=1:length(S.ncst)
plot3(S.ncst{j, 1}(:,1),S.ncst{j, 1}(:,2),S.ncst{j, 1}(:,3),'ko','color',[0.5 0.5 0.5]);hold on;
end
% axis equal;

S = m_shaperead('A2_LINK' ); 
for j=1:length(S.ncst)
plot3(S.ncst{j, 1}(:,1),S.ncst{j, 1}(:,2),S.ncst{j, 1}(:,3),'k');hold on;
view(2);
end
% axis equal;

% set(gcf, 'color', 'none')
% set(gca, 'color', 'none')

S = m_shaperead('A2_LINK' ); 
for j=1:length(S.ncst)
plot3(S.ncst{j, 1}(:,1),S.ncst{j, 1}(:,2),S.ncst{j, 1}(:,3),'ko');hold on;
view(2);
% text(S.ncst{j, 1}(end,1),S.ncst{j, 1}(end,2),num2str(j));
end

% axis equal;