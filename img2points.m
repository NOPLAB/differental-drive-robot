function [points] = img2points(img)
%IMG2POINTS この関数の概要をここに記述
%   詳細説明をここに記述

% グレースケール変換
img_gray = im2gray(img);
% imshow(img_gray);

% バイナリ化
img_bin = imbinarize(img_gray);
% imshow(img_bin);

% エッジ検出
img_edge = edge(img_gray, "canny");
% imshow(img_edge);

% 点の座標を取得（黒いピクセルのみ）
[x, y] = ind2sub(size(img_edge), find(img_edge == 1));
points = [x'; y']; % 行列に変換（注意: 転置している）

img_size = size(img);
large_size = 0;
if img_size(1) > img_size(2)
    large_size = img_size(1);
else 
    large_size = img_size(2);
end

% 縮小
points = points / large_size;

% 点をプロット
%{
figure;
plot(y, x, '*'); % 軸が入れ替わるため y, x の順にプロット
title('点');
%}

end

