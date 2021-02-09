% the g cost and the addmisible transition
% the enable_diagonal is true, the index list will allow diagonal move
function [g,index_list]= util_CostNeighb(M,g_parent,size_x,size_y,node_x,node_y,closelist,enable_diagonal)
g = [];
index_list = [];

% generate indexes for surruonding nodes

index_s = [node_x+1 node_y;
           node_x-1 node_y;
           node_x node_y+1;
           node_x node_y-1];

% if enable_diagonal is true, do the same for the diagonal terms
if(enable_diagonal)
    index_s = [index_s;
               node_x+1 node_y+1;
               node_x+1 node_y-1;
               node_x-1 node_y+1;
               node_x-1 node_y-1];
end


for i = 1:max(size(index_s))
    % determine if addmissible
 
    if((index_s(i,1)>0&&index_s(i,1)<=size_x)&&(index_s(i,2)>0&&index_s(i,2)<=size_y))% first check whether the index is out of bound
        % then check whether the barrier is active
        if( (M(index_s(i,1),index_s(i,2))==0))% make sure the path is within the area
            [isInCloseList,~] = util_IsLocationinList(closelist, index_s(i,1),index_s(i,2));     
            if(~isInCloseList)% should also not in the closed list
                g = [g ;sqrt((index_s(i,1) - node_x)^2+(index_s(i,2) - node_y)^2)+g_parent]; 
                index_list = [index_list;
                            index_s(i,1) index_s(i,2)];            
            end           
        end
    end
end
