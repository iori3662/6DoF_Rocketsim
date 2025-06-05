clear all;
close all;

% 変数宣言
M = 5.0;     % ボールの重さ（使っていないので削除してもOK）
v0 = 5.0;    % 初速度
y0 = 0.0;    % 初期位置
g = 9.81;    % 重力加速度（G → gに統一）
dt = 0.01;   % 刻み幅
T_end = 2 * v0 / g;
N = round(T_end / dt);  % サンプル数（小数を避ける）

% 初期化（インデックスは1から）
y = zeros(1, N+1);
v = zeros(1, N+1);
T = zeros(1, N+1);

y(1) = y0;
v(1) = v0;
T(1) = 0;

for i = 1:N
    v(i+1) = v(i) - g * dt;
    y(i+1) = y(i) + v(i+1) * dt;
    T(i+1) = T(i) + dt;
end

% プロット
plot(T, y)
xlabel('Time [s]')
ylabel('Height [m]')
title('鉛直投げ上げ運動のシミュレーション')
grid on
