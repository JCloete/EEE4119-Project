function make_animation(t, x, y, th, ast_x, ast_y, ast_th, ast_dx, ast_dy, fps)
% warning: this is a crude animation - don't take it too seriously!
%
% It expects t, x, y, etc as regular arrays (pass as x.signals.values)
% and a constant step size, which is likely what you want when doing
% control
%
% fps is the number of frames per second the animation should run at
%
% There are two timings happening here: one to make the animation run in
% roughly real time, and one to set the number of frames shown per second
% The path that the asteroid would take in the absence of gravity or drag
% forces is shown, mainly to make you aware of the fact that predicting
% the asteroid's path is not trivial
%
% Lastly, there is a small line to help you keep track of the asteroid's
% angle

    define_constants;

    dt = t(2) - t(1);

    figure; hold all; shg
    coords = [XMIN XMAX YMIN YMAX];

    cur_t = 0;
    tic;
    for i = 1:length(x)
        % only make a frame every 1/fps seconds
        cur_t = cur_t + dt;
        if cur_t < 1/fps
            continue  % skip to the next for loop iteration
        else
            cur_t = 0;
        end
        
        clf
        
        % a horizontal line for the ground
        plot([coords(1) coords(2)], [0, 0])
        hold on
        
        % a landing pad at x = y = 0 = 0
        plot([  -10   -10   10   10], [0 50 50 0]) % the launch pad
        plot([ 2200  2200 2800 2800], [0 50 50 0]) % the city
        
        add_rocket(x(i), y(i), th(i), WIDTH, HEIGHT)
        add_asteroid(ast_x(i), ast_y(i), ast_th(i))
        add_asteroid_dxdy_line(ast_x(i), ast_y(i), ast_dx(i), ast_dy(i))
        add_asteroid_angle_line(ast_x(i), ast_y(i), ast_th(i))
        
        title(['t = ', num2str(i*dt), 's'])

        axis(coords)
        grid on
        
        % make the plot update in real time, ish
        t_loop = toc;
        t_loop = t_loop*2;  % uncomment this to speed animation by
                              % roughly 2
        if t_loop < 1/fps
            pause(1/fps - t_loop)
        end
        tic;
    end
end


function add_rocket(x, y, th, L1, L2)
    % this is a total hack. I really just don't know matlab's plotting
    % facilities very well, though I think it's annotation bits just
    %  aren't very good
    x_coords = [x x-sin(th)*L1];
    y_coords = [y y+cos(th)*L2];

    ha = annotation('arrow');  % store the arrow information in ha
    ha.Parent = gca;           % associate the arrow the the current axes
    ha.X = x_coords;           % the location in data units
    ha.Y = y_coords;

    ha.LineWidth  = 1;         % make the arrow bolder for the picture
    ha.HeadWidth  = 10;
    ha.HeadLength = 10;
end

function add_asteroid(x, y, th)
    radius = 100;
    t = transpose(-pi:0.01:pi);
    rot = [ cos(th) -sin(th)
            sin(th)  cos(th)];
    rough_edges = [radius * (sin(t) + sin(7*t)/5 + cos(4*t)/7),...
                   radius * (cos(t) + cos(9*t)/4 + cos(5*t)/8)];
    rough_edges = (rot * rough_edges')';
    plot(x + rough_edges(:,1), ...
         y + rough_edges(:,2))
end

function add_asteroid_dxdy_line(x, y, dx, dy)
    plot([x; x + 100*dx], [y; y + 100*dy])
end

function add_asteroid_angle_line(x, y, th)
    length = 300;
    rot = [ cos(th) -sin(th)
            sin(th)  cos(th)];
    pts = length/2 * (rot * [1 -1; 0 0])';
    plot(x + pts(:,1), y + pts(:,2))
end
