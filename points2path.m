function path = points2path(points, len)
%IMG2PATH この関数の概要をここに記述
%   詳細説明をここに記述

% 点の数
num_points = size(points, 2);

% 距離を計算し、条件を満たす点を選択
keep_indices = true(1, num_points); % 初期状態では全ての点を保持

for i = 1:num_points
    if keep_indices(i) % この点がまだ有効であればチェック
        % 現在の点との距離を計算
        distances = sqrt(sum((points - points(:, i)).^2, 1));
        
        % 自分自身以外で、距離がlen未満の点を除外
        close_indices = (distances < len) & (1:num_points ~= i);
        keep_indices(close_indices) = false;
    end
end

% 抽出された点
filtered_points = points(:, keep_indices);

% 結果をプロット
%{
figure;
scatter(points(2, :), points(1, :), 'r', 'filled'); % 元の点 (赤)
hold on;
scatter(filtered_points(2, :), filtered_points(1, :), 'b', 'filled'); % 抽出された点 (青)
legend('元の点', '抽出された点');
xlabel('y');
ylabel('x');
title('点の抽出結果');
%}


% ソート済みの点のリスト（入力としては filtered_points）
num_points = size(filtered_points, 2); % 点の数

% 一筆書き用のパス初期化
path = zeros(2, num_points); % 結果を格納する配列
visited = false(1, num_points); % 訪問済みを記録する配列

% 初期点を右上の点として選択 (yが最大、同じ場合はxが最小)
[~, start_idx] = max(filtered_points(2, :) + 1e-6 * filtered_points(1, :)); % 右上を見つける
path(:, 1) = filtered_points(:, start_idx); % 最初の点を記録
visited(start_idx) = true; % 訪問済みに設定

% 一筆書きのパスを計算
current_idx = start_idx; % 現在位置のインデックス
for i = 2:num_points
    % 現在の点から他の未訪問の点への距離を計算
    distances = sqrt(sum((filtered_points - filtered_points(:, current_idx)).^2, 1));
    distances(visited) = inf; % 訪問済みの点は除外
    
    % 最も近い未訪問点を探す
    [~, next_idx] = min(distances);
    
    % 次の点をパスに追加
    path(:, i) = filtered_points(:, next_idx);
    visited(next_idx) = true; % 訪問済みに設定
    current_idx = next_idx; % 次の点を現在の点とする
end

% 結果をプロット
% figure;
% scatter(filtered_points(2, :), filtered_points(1, :), 'r', 'filled'); % 元の点
% hold on;
% plot(path(2, :), path(1, :), '-b', 'LineWidth', 1.5); % パス
% legend('抽出された点', 'ロボットのパス');
% xlabel('y');
% ylabel('x');
% title('ロボットの一筆書きパス');
end

