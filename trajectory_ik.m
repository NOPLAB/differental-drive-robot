clear;
clc;

% 画像を読み込み
img = imread("github.png");
% imshow(img);

points = img2points(img);

path = points2path(points, 0.005);
path = path';

% figure
% plot(path(:,1), path(:,2),'k--d')
% xlim([0 1])
% ylim([0 1])

R = 0.025;
L = 0.146;

x0 = path(1, 1);
y0 = path(1, 2);

theta = pi;

t = 0;

vl_history = [0];
vr_history = [0];

path_size = size(path)

Ts = 0.05;

last_points = [x0, y0];
for p = path'
    vx = (p(1) - last_points(1)) / Ts;
    vy = (p(2) - last_points(2)) / Ts;
    
    v = sqrt(vx^2 + vy^2);

    omega = (atan2(vy, vx) - theta) / Ts;

    disp([vx, vy, v, omega]);

    vl = (2*v - L * omega) / 2;
    vr = (2*v + L * omega) / 2;

    % disp([vl, vr]);

    vl_history(end+1) = vl;
    vr_history(end+1) = vr;

    theta = theta + omega * Ts;
    last_points = p;

    t = t + 1;
end

t

figure;
plot(vl_history);
hold on;
plot(vr_history);
hold off;

% CSVに書き出すデータを準備
output_data = table(vr_history', vl_history');

% ファイル名を指定
output_filename = 'Data.csv';

% CSVに書き出し
writetable(output_data, output_filename);