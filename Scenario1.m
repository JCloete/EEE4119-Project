% Find time when x_ast = 0;
t_index = find(x_ast >= 0);
x_ast(t_index(2));

time = t_index(1);

num = 2000*y_ast(t_index(1))+9810*t_index(1)^2;
den = t_index(1)^2;

Ft = num/den