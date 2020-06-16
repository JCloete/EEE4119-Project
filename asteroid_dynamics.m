X0_ast = [-3000, 182];
Y0_ast = [5000, 0];
T0_ast = 0 ;
TFINAL_ast = 35;

[t_ast, x_ast] = ode45(@eomAstX, [T0_ast TFINAL_ast], X0_ast)
x_ast(:,1)
plot(t_ast,x_ast(:,1))
hold;
[t_ast, y_ast] = ode45(@eomAstY, [T0_ast TFINAL_ast], Y0_ast)
y_ast(:,1)
plot(t_ast,y_ast(:,1))

function [xdot] = eomAstX(t,x)
    CM = 0.0093;
    xdot(1) = x(2);
    xdot(2) = -CM*x(2);
    xdot = xdot';
end

function [ydot] = eomAstY(t,y)
    CM = 0.0093;
    ydot(1) = y(2);
    ydot(2) = CM*y(2) - 9.81;
    ydot = ydot';
end