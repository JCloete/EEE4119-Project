clear c_x;

for i = 1:20
    sim('AsteroidImpact.slx');
    ddx = diff(ast_dx)./diff(simulation_time);
    tv = (simulation_time(1:end-1)+simulation_time(2:end))/2;
    %plot(tv,ddx);

    c = ((ddx)./(-ast_dx(2:end)));
    c_x(i) = sum(c)/length(c)
end

c_final = sum(c_x)/length(c_x)


% [tout, xout] = ode45(@eom_x, simulation_time, [-3000, 182]);
% 
% plot(tout, xout(:,1))
% hold
% plot(simulation_time, ast_x)
% hold

function x = eom_x(time,ddx)
    c = 0.0093;
    x(1) = ddx(2);
    x(2) = -ddx(2)*c;
    x = x';
end
%plot(tv,c_x);