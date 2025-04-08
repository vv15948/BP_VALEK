function  plot_path(path) 

        subplot(1, 2, 1);
            plot3(path(:,1), path(:,2), path(:,3), 'c-', 'LineWidth', 2); % Tyrkysová trajektorie
        subplot(1, 2, 2);
            plot3(path(:,1), path(:,2), path(:,3), 'c-', 'LineWidth', 2); % Tyrkysová trajektorie 

end