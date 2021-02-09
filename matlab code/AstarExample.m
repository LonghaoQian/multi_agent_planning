% a star
close all
% generate the maze
NN = 20;
M = zeros(NN);
C = ones(size(M)); % cost of the transition
% add two barriers
for  i = 1 : 10
    
    M(i,7) = 1;
    
    M(15-i, 15) = 1; 
    
end

start_point = [1 2];
end_point = [10 19];

% NN = 4;
% M = zeros(NN);
% C = ones(size(M)); % cost of the transition
% 
% M(2,3) =1;
% M(3,2) =1;
% M(3,3) =1;
% start_point = [1 2];
% end_point = [4 4];

enable_diagonal = true;
% generate the openlist index x, index, y, g, h, f, parent
h0 = util_Heuristic(start_point(1),start_point(2),end_point(1), end_point(2),enable_diagonal);
% 1. index_x, 2. index_y, 3. g_cost, 4. h_cost, 5. f_cost, 6. parent index_x
optentialpoints = [start_point, 0, h0,  h0,  -1  -1 ];
closelist  = [];
current_point_index = 1; % index of the current point in the openlist
path = [];% path indexes
while(current_point_index~=-1)
    
    current_point_index = util_lowest_f_index(optentialpoints);
    
    if(current_point_index ==-1)% no solutions possible
        disp('No solutions!')
        break;
    end
    
    % remove the current point from the openlist and put it into the closed list
    closelist  = [closelist;
                optentialpoints(current_point_index,:)];
    
    optentialpoints(current_point_index,:) = [];
    
    if(util_CompareLocation(closelist(end,1), closelist(end,2), end_point(1), end_point(2)))
        disp('Solution Found!')
        path = util_ReconstructPath(closelist);
        break;
    end
    % get the g cost for all neighbouring points
    [g,g_list]= util_CostNeighb(M,closelist(end,3),NN,NN,closelist(end,1),closelist(end,2),closelist,enable_diagonal ); % get the g cost
    % updatge h and f cost
    h = zeros(size(g));
    f = zeros(size(g));
    for i = 1: max(size(g))
        h(i) = util_Heuristic(g_list(i,1),g_list(i,2),end_point(1), end_point(2),enable_diagonal );
        f(i) = h(i) + g(i);
    end
    % update    
    dimG = size(g);
    isInOpenlist = false;
    if(dimG(1)~=0)
        for i = 1 : dimG(1)
            [isInOpenlist,location]= util_IsLocationinList(optentialpoints, g_list(i,1), g_list(i,2));
            if(isInOpenlist)
                % if the g value is lower than the g value in the openlist, then update
                % g, f, and parent index
                if(g(i)<optentialpoints(location,3))
                    optentialpoints(location,3) = g(i);
                    optentialpoints(location,5) = f(i);
                    optentialpoints(location,6) = closelist(end,1);
                    optentialpoints(location,7) = closelist(end,2);
                end
            else
                % if the index is not in the open list, then add it to the
                % openlist
                 optentialpoints = [ optentialpoints;
                     g_list(i,1), g_list(i,2), g(i),h(i),f(i), closelist(end,1),closelist(end,2)];
            end
        end
    end
    
    % record all these points in 

end

% plot the grid

figure(1)
hold on
for i = 1:NN
    for j = 1:NN
        if(M(i,j)==0)
            plot(i,j,'kx','LineWidth',2)           
        else
            plot(i,j,'rx','LineWidth',2)    
        end

    end
end

plot(start_point(1),start_point(2),'g+','LineWidth',2)         
plot(end_point(1),end_point(2),'bo','LineWidth',2)     
grid on
axis equal

dimPath = size(path);

plot(path(:,1),path(:,2),'r-','LineWidth',2)     
