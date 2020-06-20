%% make an animation
close all;
define_constants;

% look at the source of make_animation if it doesn't work - you may need to
% lower the fps on slower computers. Otherwise you can uncomment a line to
% make the animation happen faster than real time
set_param(['AsteroidImpact', '/rngSeed'], 'Value', '123');
sim('AsteroidImpact');  % tell simulink to simulate the model
make_animation(simulation_time, x, y, th, ast_x, ast_y, ast_th, ast_dx, ast_dy, 10)

% 'simulation_time' is an array of time values, exported from the simulink
% simulation

%% Make a video animation and save it to rocket-animation.avi
% this is more useful if the animation script above gives you problems
% or if you want to share a video with anyone else
% record_animation(simulation_time, x, y, th, ast_x, ast_y, ast_th, ast_dx, ast_dy)

%% check whether you succeeded
Scenario = 2;
mission_complete(x, y, ast_x, ast_y, ast_th, Scenario)

%% think about other ways of visualizing this system!

% % ENSURE IT WORKS FOR ALL SEEDS
% win_count = 0;
% fail_count = 0;
% no_of_attempts = 0;
% for i = 111:999
%     set_param(['AsteroidImpact', '/rngSeed'], 'Value', sprintf('%d',i));
%     sim('AsteroidImpact');  % tell simulink to simulate the model
%     Scenario = 2;
%     if mission_complete(x, y, ast_x, ast_y, ast_th, Scenario)
%         win_count = win_count + 1;
%     else
%         fail_count = fail_count + 1;
%     end
%     no_of_attempts = no_of_attempts + 1;
% end
% 
% no_of_attempts
% win_count
% fail_count
