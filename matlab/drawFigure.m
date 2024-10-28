function drawFigure(params, state)

% unpack params
r = params(4);
L = 2*params(1);

x = state(1, :);
theta = state(2, :);

phi = -x/r;

figure;

for i = 1:length(theta)
  
    circle(x(i),r);
    axis equal;
    axis([-5, 5, -r, 0.3]);

    plot([x(i), x(i)-r*sin(phi(i))], [0, r*cos(phi(i))], 'r');

    plot([x(i), x(i)-L*sin(theta(i))], [0, L*cos(theta(i))], 'k');
    pause(0.1);
    hold off;

end
end

