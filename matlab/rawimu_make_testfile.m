function truth = rawimu_make_testfile(outFile, seconds, varargin)
%RAWIMU_MAKETESTFILE  生成一段“已知姿态”的 RAWIMU 二进制流，用于无硬件自检/回放
%
%   truth = rawimu_make_testfile('rawimu_test.dat', 20);
%   rawimu_live('Source','file','File','rawimu_test.dat');     % 应能画出同样的角运动
%
%   truth 列： [t(s) roll(deg) pitch(deg) yaw(deg)]
%
%   可选 Name-Value：
%     'Fs'        100      采样率 Hz（RAWIMU 一般 100 Hz）
%     'Week'      2435     GPS 周
%     'Sow0'      400000   起始周内秒
%     'ImuModel'  'M1633'  与 rawimu_live 的预设一致
%     'InvertY'   true     与 rawimu_live 的约定一致
%     'YawRate'   30       偏航角速度 deg/s（匀速）
%     'RollAmp'   20       横滚正弦幅度 deg
%     'RollFreq'  0.15     Hz
%     'PitchAmp'  10       俯仰正弦幅度 deg
%     'PitchFreq' 0.08     Hz
%     'Temp'      25       温度 °C
%     'Noise'     false    是否夹杂 ASCII 行，用于验证重同步
%     'SaveTruth' ''       非空则把真值写 CSV
%
%   注意：本函数只验证“帧解析 + 单位换算 + 姿态积分 + 显示”这条链路，
%   不能替代真实设备的 LSB 标定。

p = struct('Fs', 100, 'Week', 2435, 'Sow0', 400000, 'ImuModel', 'M1633', ...
    'InvertY', true, 'YawRate', 30, 'RollAmp', 20, 'RollFreq', 0.15, ...
    'PitchAmp', 10, 'PitchFreq', 0.08, 'Temp', 25, 'Noise', false, 'SaveTruth', '');
if mod(numel(varargin), 2) ~= 0
    error('参数必须成对出现');
end
fn = fieldnames(p);
for k = 1:2:numel(varargin)
    idx = find(strcmpi(fn, varargin{k}), 1);
    if isempty(idx)
        error('未知参数：%s', varargin{k});
    end
    p.(fn{idx}) = varargin{k+1};
end

sc = preset_scale(p.ImuModel);

fs = p.Fs;
N = round(seconds * fs);
t = (0:N-1).' / fs;

% ---- 真值姿态（deg）----
roll  = p.RollAmp  * sin(2*pi*p.RollFreq  * t);
pitch = p.PitchAmp * sin(2*pi*p.PitchFreq * t);
yaw   = p.YawRate * t;

% ---- 姿态 -> 四元数 -> 机体角速度 ----
q = zeros(N, 4);
Rall = zeros(3, 3, N);
for k = 1:N
    R = eul2R(roll(k), pitch(k), yaw(k));
    Rall(:, :, k) = R;
    q(k, :) = R2quat(R).';
end

w = zeros(N, 3);                      % rad/s，机体轴
dt = 1/fs;
for k = 1:N-1
    dq = quat_mul(quat_conj(q(k, :)), q(k+1, :));
    if dq(1) < 0, dq = -dq; end
    ang = 2 * atan2(norm(dq(2:4)), dq(1));
    if norm(dq(2:4)) > 1e-12
        w(k, :) = (dq(2:4) / norm(dq(2:4)) * ang / dt);
    end
end
w(N, :) = w(max(1, N-1), :);

g = 9.80665;
% ---- 写帧 ----
fid = fopen(outFile, 'w');
if fid < 0
    error('无法写文件：%s', outFile);
end
cleanupObj = onCleanup(@() fclose(fid));

week = p.Week;
sow = p.Sow0 + t;
seq = 0;
for k = 1:N
    % 角速度 -> 计数
    gyroDeg = w(k, :) * 180/pi;
    gCount = round(gyroDeg / sc.GyroLsbDeg);
    % 比力（静止时 f = R^T * [0;0;g]）
    fBody = Rall(:, :, k).' * [0; 0; g];
    aCount = round(fBody.' / sc.AccelLsbMps2);
    if p.InvertY
        gCount(2) = -gCount(2);
        aCount(2) = -aCount(2);
    end
    % IMU_Status：低 8 位采样计数、8~15 位状态、高 16 位温度
    counter = mod(k - 1, 256);
    traw = int16(round((p.Temp - sc.TempOffset) / sc.TempLsb));
    st = uint32(counter) + bitshift(uint32(0), 8) + ...
         bitshift(uint32(typecast(traw, 'uint16')), 16);

    hdr = [uint8([170 68 18 28]) ...
           typecast(uint16(268), 'uint8') ...
           uint8([0 1]) ...
           typecast(uint16(40), 'uint8') ...
           typecast(uint16(mod(seq, 65536)), 'uint8') ...
           uint8([0 32]) ...
           typecast(uint16(week), 'uint8') ...
           typecast(uint32(mod(round(sow(k) * 1000), 604800000)), 'uint8') ...
           typecast(uint32(0), 'uint8') ...
           typecast(uint16(0), 'uint8') ...
           typecast(uint16(803), 'uint8')];
    body = [typecast(uint32(week), 'uint8') ...
            typecast(sow(k), 'uint8') ...
            typecast(st, 'uint8') ...
            typecast(int32([aCount(3) aCount(2) aCount(1) ...
                            gCount(3) gCount(2) gCount(1)]), 'uint8')];
    hb = [hdr body];
    fwrite(fid, hb, 'uint8');
    fwrite(fid, typecast(novatel_crc32(hb), 'uint8'), 'uint8');
    seq = seq + 1;

    if p.Noise && mod(k, 300) == 0
        fwrite(fid, sprintf('$GPGSV,1,1,00,1,1,1,1*79\r\n'), 'char');
    end
end

truth = [t, roll, pitch, yaw];
if ~isempty(p.SaveTruth)
    fid2 = fopen(p.SaveTruth, 'w');
    fprintf(fid2, 't_s,roll_deg,pitch_deg,yaw_deg\n');
    fprintf(fid2, '%.6f,%.5f,%.5f,%.5f\n', truth.');
    fclose(fid2);
end
fprintf('[make_testfile] %s: %.1f s, %d 帧, 真值 roll±%.0f° pitch±%.0f° yaw %.0f°/s\n', ...
    outFile, seconds, N, p.RollAmp, p.PitchAmp, p.YawRate);
end

% -------------------------------------------------------------------
function sc = preset_scale(name)
G = 9.80665;
switch upper(name)
    case 'M1633'
        sc = struct('GyroLsbDeg', 1/65536, 'AccelLsbMps2', 0.5/65536, ...
            'TempLsb', 1/100, 'TempOffset', 0);
    case 'B240'
        sc = struct('GyroLsbDeg', 1/65536, 'AccelLsbMps2', 0.5/65536, ...
            'TempLsb', 1/256, 'TempOffset', 25);
    case 'B240_16BIT'
        sc = struct('GyroLsbDeg', 1/100, 'AccelLsbMps2', G/2000, ...
            'TempLsb', 1/256, 'TempOffset', 25);
    case 'B200'
        sc = struct('GyroLsbDeg', 1/65536, 'AccelLsbMps2', G*100/5000/65536, ...
            'TempLsb', 1/200, 'TempOffset', 50);
    otherwise
        error('未知 IMU 预设：%s', name);
end
end

function R = eul2R(rollDeg, pitchDeg, yawDeg)
r = rollDeg*pi/180; p = pitchDeg*pi/180; y = yawDeg*pi/180;
Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
R = Rz * Ry * Rx;
end

function q = R2quat(R)
tr = trace(R);
if tr > 0
    s = sqrt(tr + 1) * 2;
    q = [0.25*s; (R(3,2)-R(2,3))/s; (R(1,3)-R(3,1))/s; (R(2,1)-R(1,2))/s];
else
    [~, i] = max([R(1,1), R(2,2), R(3,3)]);
    if i == 1
        s = sqrt(1 + R(1,1) - R(2,2) - R(3,3)) * 2;
        q = [(R(3,2)-R(2,3))/s; 0.25*s; (R(1,2)+R(2,1))/s; (R(1,3)+R(3,1))/s];
    elseif i == 2
        s = sqrt(1 + R(2,2) - R(1,1) - R(3,3)) * 2;
        q = [(R(1,3)-R(3,1))/s; (R(1,2)+R(2,1))/s; 0.25*s; (R(2,3)+R(3,2))/s];
    else
        s = sqrt(1 + R(3,3) - R(1,1) - R(2,2)) * 2;
        q = [(R(2,1)-R(1,2))/s; (R(1,3)+R(3,1))/s; (R(2,3)+R(3,2))/s; 0.25*s];
    end
end
q = q / norm(q);
end

function q = quat_conj(q)
q = [q(1); -q(2); -q(3); -q(4)];
end

function q = quat_mul(a, b)
q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
     a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
     a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
     a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end
