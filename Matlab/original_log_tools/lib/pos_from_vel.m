function [t, pos] = pos_from_vel(t_in, vx, vy, yaw, pitch, roll)

t = t_in;

pos = zeros(3, size(t, 2));

for i = 2:size(t, 2),
    inc_x = vx(i - 1) * (t(i) - t(i - 1)) + (vx(i) - vx(i - 1)) * (t(i) - t(i - 1)) / 2;
    inc_y = vy(i - 1) * (t(i) - t(i - 1)) + (vy(i) - vy(i - 1)) * (t(i) - t(i - 1)) / 2;
%    m = rotk([1 0 0], pi) * rotk([0 0 1], yaw(i) * pi / 180) * rotk([0 1 0], pitch(i) * pi / 180) * rotk([1 0 0], roll(i) * pi / 180); 
    m = rotk([1 0 0], pi) * rotk([0 0 1], yaw(i) * pi / 180);
    pos(:, i) = pos(:, i - 1) + m * [inc_x; inc_y; 0];
end;

end

