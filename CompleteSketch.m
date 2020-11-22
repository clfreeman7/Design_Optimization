function [x_full , y_full] = CompleteSketch(x , y, n)
    v = [x';y'];
    x_full = x';
    y_full = y';
    for i = 2:n
        theta = 2*pi/n*(i-1);
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        vnew = R*v;
        x_full = [x_full vnew(1,:)];
        y_full = [y_full vnew(2,:)];
    end
end