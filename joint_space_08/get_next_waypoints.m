function [q_start, start_point, q_goal, goal_point, next_index] = get_next_waypoints(current_index)
   
% Předdefinované waypointy: [q1–q6, x, y, z]
waypoints =[ -1.339361492787500  -2.251367231408590  -2.077919244766240   1.190111561412480   1.348566412925720   1.570490598678590 -0.032 0.5 0.25;
               -4.687263909970420  -1.988374372521870  -1.188712000846860  -0.991601244812348   1.480629205703740   1.590799450874330 -0.2 -0.69 0.55];
               
    % Načtení aktuálního startu
    q_start = waypoints(current_index, 1:6);
    start_point = waypoints(current_index, 7:9);

    % Index cílového bodu (cyklicky se opakuje)
    if current_index < size(waypoints, 1)
        goal_index = current_index + 1;
    else
        goal_index = 1;
    end
    
    % Načtení cílového bodu
    q_goal = waypoints(goal_index, 1:6);
    goal_point = waypoints(goal_index, 7:9);

    % Index pro příští iteraci
    next_index = goal_index;
end
