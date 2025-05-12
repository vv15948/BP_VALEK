function [q_start, start_point, q_goal, goal_point, next_index] = get_next_waypoints(waypoints,current_index)

    % Dopočítání polohy TCP pro startovní konfiguraci
    q_start = waypoints(current_index, 1:6);
    [start_positions,dir] = forward_kinematics_model(waypoints(current_index,:));
    start_point = start_positions(7,:);
     
    % Index cílového bodu (cyklicky se opakuje)
    if current_index < size(waypoints, 1)
        goal_index = current_index + 1;
    else
        goal_index = 1;
    end
        
    % Načtení cílového bodu
    q_goal = waypoints(goal_index,:);
    
    % Dopočítání polohy TCP pro cílovou konfiguraci
    [goal_positions,dir] = forward_kinematics_model(waypoints(goal_index,:));
    goal_point = goal_positions(7,:);
        
    % Index pro příští iteraci
    next_index = goal_index;
end
