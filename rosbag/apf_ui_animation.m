function apf_ui_animation(handle_figure ,robot, robot_id)
    handle_figure;
    clf;

    % ロボット図形の頂点
    robot_polygon = 0.8*[[-0.2 0.3];[0 0.4];[0.2 0.3];[0.2 -0.1];[-0.2 -0.1]];

    toggle_ui_vigible_key = 'v';    % UI表示切替キー
    max_potential_value = 3;        % 表示するポテンシャル値の上限
    gridSize = 150;                 % ポテンシャル場グリッドの解像度

    robot_vertices = [robot_polygon; robot_polygon(1,:)];
    obstacle_vertices = 0*robot_vertices;
    
    itr = 1;
    isAnimating = false;
    isVisible = true;
    isCapturing = false;
    
    k_begin = 1;
    k_end = numel(robot{robot_id}.Odom);

    set(handle_figure, 'CloseRequestFcn', {@closeFigure});
    set(handle_figure, 'KeyPressFcn', {@toggle_vigible_key});

    handle_ui = init_ui();

    dt = mean(diff([robot{robot_id}.Odom.Time]));
    animationTimer = timer('ExecutionMode', 'fixedRate', 'Period', round(dt,3), 'TimerFcn', @animatePlot);

    handle_plot = init_figure();
    
    t_pre = -1;
    for i = 1:numel(robot{robot_id}.Odom)
        t = robot{robot_id}.Odom(i).Time;
        potential_field = get_cell(robot{robot_id}.Potential, 'Time', t);
        if potential_field.Time ~= t_pre
            xyz=double(potential_field.PointCloud);
            index = xyz(:, 3) >= max_potential_value;
            xyz(index, 3) = max_potential_value;
            
            x = xyz(:,1);
            y = xyz(:,2);
            z = xyz(:,3);
            
            % 規則的なグリッドを生成
            xlin = linspace(min(x), max(x), gridSize);
            ylin = linspace(min(y), max(y), gridSize);
            [X, Y] = meshgrid(xlin, ylin);
            
            % zデータを格子点に補間
            Z = griddata(x, y, z, X, Y, 'cubic');
        end
        
        apf(i).X = X;
        apf(i).Y = Y;
        apf(i).Z = Z;
        t_pre = potential_field.Time;
    end

    for i = 1:numel(robot{robot_id}.Odom)
        t = robot{robot_id}.Odom(i).Time;
        robot_path = get_cell(robot{robot_id}.Path, 'Time', t);
        
        if isempty(robot_path.Path)
            path(i).X = zeros(1,1);
            path(i).Y = zeros(1,1);
        else
            path(i).X = [robot_path.Path.X];
            path(i).Y = [robot_path.Path.Y];
        end

    end

    video = VideoWriter("Capture", 'MPEG-4');
    video.Quality = 100;
    % video.Colormap.colormap(summer(256));

    plot_potential(itr);

    function h = init_ui()
        h.frame_slider = uicontrol('Style', 'frame', 'Visible', 0, ...
        'Position', [150, 10, 60, 20]);

        h.slider = uicontrol('Style', 'slider', 'Min', k_begin, 'Max', k_end, 'Value', k_begin, ...
            'Position', [h.frame_slider.Position(1), h.frame_slider.Position(2), 300, 20], ...
            'Callback', {@updatePlot}, ...
            'KeyReleaseFcn', @slider_button);
        h.textbox_min = uicontrol('Style', 'edit', 'String', num2str(k_begin), ...
            'Position', [h.slider.Position(1) - 50, 10, 50, 20], ...
            'Callback', {@updateSliderMin});
        h.textbox_max = uicontrol('Style', 'edit', 'String', num2str(k_end), ...
            'Position', [h.slider.Position(1) + h.slider.Position(3), 10, 50, 20], ...
            'Callback', {@updateSliderMax});
        h.text_itr = uicontrol('Style', 'text', 'String', get_timetext(0,1), ...
            'Position', [h.slider.Position(1) + h.slider.Position(3)/2 - 75, h.frame_slider.Position(4) + 15, 150, 15]);
        
        h.frame_buttons = uicontrol('Style', 'frame', 'Visible', 0, ...
            'Position', [h.frame_slider.Position(1) - 120, h.frame_slider.Position(2) + 40, 60, 20]);
    
        h.toggle_animation = uicontrol("Style", "togglebutton", "String", "Play", ...
            'Position', [h.frame_buttons.Position(1), h.frame_buttons.Position(2) 60 20], ...
            "Callback", {@toggle_animation});
        h.toggle_capture = uicontrol("Style", "togglebutton", "String", 'Capture', ...
            'Position', [h.frame_buttons.Position(1), h.frame_buttons.Position(2) - 20 60 20], ...
            "Callback", {@toggle_capture});
        h.toggle_ui_vigible = uicontrol("Style", "pushbutton", "String", ['Visible(' toggle_ui_vigible_key ')'], ...
            'Position', [h.frame_buttons.Position(1), h.frame_buttons.Position(2) - 40 60 20], ...
            "Callback", {@toggle_vigible_button});
    end
    
    function slider_button(src, data)
        if data.Key == "numpad6"
            itr = itr + 1;
            if itr > src.Max
                itr = src.Max;
            end
            plot_potential(itr);
            src.Value = itr;
        elseif data.Key == "numpad4"
            itr = itr - 1;
            if itr < src.Min
                itr = src.Min;
            end
            plot_potential(itr);
            src.Value = itr;
        end
    end

    function text = get_timetext(time, itr)
        text = ['time: ' num2str(time) ' [s]  itr: ' num2str(itr)];
    end

    function closeFigure(~, ~)
        stop(animationTimer);
        delete(animationTimer);
        delete(handle_figure);
    end
    
    function animatePlot(~, ~)
        if isAnimating
            handle_ui.slider.Value = itr;
            plot_potential(itr);
            
            try 
                frame = getframe(handle_figure);
                writeVideo(video,frame);
            catch
            end

            itr = itr + 1;
            if itr > handle_ui.slider.Max
                itr = handle_ui.slider.Max;
                set(handle_ui.toggle_animation, 'String', 'Play');
                set(handle_ui.toggle_animation, 'Value', 0);
                isAnimating = false;
                stop(animationTimer);
            end
        end
    end
    
    function toggle_animation(src, ~)
    
        if get(src, 'Value')
            set(src, 'String', 'Pause');
            isAnimating = true;
            start(animationTimer);
        else
            set(src, 'String', 'Play');
            isAnimating = false;
            stop(animationTimer);
        end

    end

    function toggle_capture(src, ~)
    
        if get(src, 'Value')
            isCapturing = true;
            set(src, 'String', 'Save');
            set(handle_figure, "NextPlot", "replacechildren");
            open(video);
        else
            isCapturing = false;
            set(src, 'String', 'Capture');
            set(handle_figure, "NextPlot", "add");
            close(video);
        end

    end
    
    function toggle_vigible_key(~, data)
        if data.Key == toggle_ui_vigible_key
            toggle_ui_vigible();
        end
    end

    function toggle_vigible_button(~, ~)
        toggle_ui_vigible();
    end

    function toggle_ui_vigible()

        isVisible = not(isVisible);

        set(handle_ui.slider, 'Visible', isVisible);
        set(handle_ui.textbox_min, 'Visible', isVisible);
        set(handle_ui.textbox_max, 'Visible', isVisible);
        set(handle_ui.toggle_animation, 'Visible', isVisible);
        set(handle_ui.toggle_capture, 'Visible', isVisible);
        set(handle_ui.text_itr, 'Visible', isVisible);
        set(handle_ui.toggle_ui_vigible, 'Visible', isVisible);

    end
    
    function updateSliderMax(src, ~)
        val = round(str2double(src.String));
        if itr > val
            itr = val;
            set(handle_ui.slider ,'Value', itr);
            plot_potential(itr);
        end
        set(handle_ui.slider ,'Max', val);
    end
    
    function updateSliderMin(src, ~)
        val = round(str2double(src.String));
        if itr < val
            itr = val;
            set(handle_ui.slider ,'Value', itr);
            plot_potential(itr);
        end
        set(handle_ui.slider ,'Min', val);
    end
    
    function updatePlot(src, ~)
        itr = round(src.Value);
        plot_potential(itr);
    end
    
    function h = init_figure()
        
        maxX = arrayfun(@(s) max(s.PointCloud(:,1)), robot{robot_id}.Potential);
        maxX = max(maxX);
        minX = arrayfun(@(s) min(s.PointCloud(:,1)), robot{robot_id}.Potential);
        minX = min(minX);
        
        maxY = arrayfun(@(s) max(s.PointCloud(:,2)), robot{robot_id}.Potential);
        maxY = max(maxY);
        minY = arrayfun(@(s) min(s.PointCloud(:,2)), robot{robot_id}.Potential);
        minY = min(minY);

        h.potential_mesh = mesh(zeros(2,2),zeros(2,2),zeros(2,2));
        colormap(flipud(jet));
        colorbar;
        h.potential_mesh.FaceAlpha = 0.2;
        h.potential_mesh.EdgeAlpha = 0.2;
        set(gca, 'TickDir', 'in');
        xlabel('$X$ [m]', 'Interpreter','latex');
        ylabel('$Y$ [m]', 'Interpreter','latex');
        zlabel('U', 'Interpreter','latex');
        axis equal;
        
        xlim([minX maxX]);
        ylim([minY maxY]);
        
        hold on
        h.path = plot(0,0, 'Color', 'k');
        
        h.robot_polygon = plot(0,0, 'Color', 'k');
        
        for i = 1:length(robot) - 1
            h.obstacle_polygon{i} = plot(0,0, '.m');
        end
        
        plot(2, 0, '.k', 'MarkerSize', 10);
    
    end
    
    function plot_potential(k)
    
        try
            t = robot{robot_id}.Odom(k).Time;
        catch ME
            return
        end
    
        robot_pose = robot{robot_id}.Odom(k);

        obstacle_pose = {};
        idx = 1;
        for i = 1:length(robot)
            if i == robot_id
                continue;
            end

            try
                obstacle_pose{idx} = robot{i}.Odom(k);
                idx = idx + 1;
            catch ME
                continue;
            end
            
        end
        
        set(handle_plot.potential_mesh,'XData', apf(k).X, 'YData', apf(k).Y, 'ZData', apf(k).Z);
    
        set(handle_plot.path, 'XData', [path(k).X], 'YData', [path(k).Y]);
        
        rp = tranform(robot_vertices, [robot_pose.X robot_pose.Y], robot_pose.Theta - pi/2);
        
        for i = 1:length(obstacle_pose)
            op = tranform(obstacle_vertices, [obstacle_pose{i}.X obstacle_pose{i}.Y], obstacle_pose{i}.Theta - pi/2);
            set(handle_plot.obstacle_polygon{i}, 'XData', op(:,1), 'YData', op(:,2));
        end

        set(handle_plot.robot_polygon, 'XData', rp(:,1), 'YData', rp(:,2));
        set(handle_ui.text_itr, 'String', get_timetext(t,k));

    end
    
    function vecs_out = tranform(vecs_in, translation, rotation)
        R = [cos(rotation), -sin(rotation);
             sin(rotation), cos(rotation)];
        vecs_out = (R * vecs_in' + translation')';
    end
    
    function cell_out = get_cell(struct_in, field, target_value)
        [minDiff, idx] = min(abs([struct_in.(field)] - target_value));
        cell_out = struct_in(idx);
    end

end