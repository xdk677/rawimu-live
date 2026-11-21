function rawimu_live(varargin)
%RAWIMU_LIVE  RAWIMU(0x10C) 串口实时解析 + 姿态三维/曲线可视化
%
%   与 M22p_gpimu.m 那种“先存 log 再离线分析”不同，本脚本直接从串口字节流里
%   边收边解 RAWIMU，实时积分出姿态（roll/pitch/yaw）并显示。
%
%   用法（在 MATLAB 里直接运行）：
%       rawimu_live                                        % 用下面 default_config 里的默认值
%       rawimu_live('Port','COM49','Baud',921600)
%       rawimu_live('ImuModel','M1633','InvertY',true)
%       rawimu_live('Source','file','File','COM49_8.03.dat')          % 回放已录的原始 log
%       rawimu_live('Source','file','File','raw.dat','Snapshot','s.png','SaveCsv','imu.csv')
%
%   无硬件自检（先用生成的已知姿态流验证解析/积分是否正确）：
%       rawimu_make_testfile('rawimu_test.dat', 20);
%       rawimu_live('Source','file','File','rawimu_test.dat');
%
%   运行中按键：ESC 退出；r 姿态归零；z 重新估计陀螺零偏（设备静止时按）。
%
%   可配置字段见 default_config；单位与换算式见 rawimu_live_README.md。
%
%   数据帧格式（NovAtel/Bynav 二进制长头，与仓库 notify_analysis/log_matrix.pkl
%   中 RAWIMU 定义一致）：
%     头 28B : AA 44 12 | len=28 | id(0x010C) | type | port | data_len=40 |
%              seq | idle | 'sync' | week(u16) | sow_ms(u32) | recv_status(u32) |
%              reserved(u16) | version(u16)
%     体 40B : Week(u32) | Seconds_into_Week(double, s) | IMU_Status(u32) |
%              Z/Y/X 加速度(i32 x3) | Z/Y/X 角速度(i32 x3)
%     尾 4B  : CRC32（覆盖头+体；初值 0、末尾不取反，见 novatel_crc32.m）
%
%   详见 rawimu_live_README.md。

% ============================== 配置 ==============================
cfg = default_config();
cfg = apply_overrides(cfg, varargin);
cfg = resolve_scales(cfg);

S = init_state(cfg);

if S.cfg.Verbose
    fprintf('=================================================================\n');
    fprintf(' RAWIMU 实时姿态监视   源=%s   IMU模型=%s   速率=%.0f Hz\n', ...
        S.cfg.Source, S.cfg.ImuModel, S.cfg.Freq);
    fprintf(' 陀螺 LSB=%.6g deg/s/LSB   加计 LSB=%.6g m/s^2/LSB   温度 LSB=%.6g\n', ...
        S.cfg.GyroLsbDeg, S.cfg.AccelLsbMps2, S.cfg.TempLsb);
    fprintf(' 提示：静止时 |a| 应≈9.81 m/s^2、温度为室温；若 |a| 差 100 倍请改 AccelLsbMps2。\n');
    fprintf(' 按键：ESC 退出, r 姿态归零, z 重新估计零偏\n');
    fprintf('=================================================================\n');
end

try
    if strcmpi(S.cfg.Source, 'file')
        run_file();
    else
        run_serial();
    end
catch ME
    finishup();
    rethrow(ME);
end

% 正常结束也刷新一次，保证截图/曲线是最终状态
if S.nRawimu > 0
    refresh_ui(true);
end
finishup();

% ======================== 嵌套函数（可访问 S） ========================

    function run_serial()
        fprintf('[RAWIMU] 打开串口 %s @ %d bps ...\n', S.cfg.Port, S.cfg.Baud);
        try
            S.port = serialport(S.cfg.Port, S.cfg.Baud, 'Timeout', S.cfg.TimeoutS);
        catch ME
            error(['打开串口失败：%s\n' ...
                   '请先确认：1) 端口号正确 serialportlist("available")；' ...
                   '2) 端口未被其它程序占用；3) 已安装 Instrument Control Toolbox。'], ME.message);
        end
        try
            S.port.InputBufferSize = 65536;   % 默认 512 B 在绘图卡顿时容易溢出
        catch
        end
        if ~isempty(S.cfg.InitCmd)
            writeline(S.port, S.cfg.InitCmd);
            fprintf('[RAWIMU] 已发送初始化命令：%s\n', S.cfg.InitCmd);
        end
        refresh_ui(true);                     % 先把窗口显示出来，避免“看不到界面”
        fprintf('[RAWIMU] 等待数据…（若无反应：查波特率、端口是否输出 RAWIMU）\n');

        lastRefresh = tic;
        while ~S.quit
            n = S.port.NumBytesAvailable;
            if n > 0
                feed(read(S.port, min(n, 262144), 'uint8'));
            else
                pause(0.002);
            end
            if toc(lastRefresh) >= S.cfg.RefreshS
                lastRefresh = tic;
                refresh_ui(false);
            end
            if check_stop()
                S.quit = true;
            end
        end
    end

    function run_file()
        fpath = S.cfg.File;
        if isempty(fpath) || ~isfile(fpath)
            error('回放文件不存在：%s', fpath);
        end
        fid = fopen(fpath, 'r');
        if fid < 0
            error('无法打开文件：%s', fpath);
        end
        fprintf('[RAWIMU] 回放文件 %s\n', fpath);

        % 节流：按 RAWIMU 每帧 72 B、Freq Hz 估算“实时”字节率
        % （ReplaySpeed=1 即接近实时回放；Inf 则全速解析）
        bytesPerSec = max(1, 72 * S.cfg.Freq);
        lastRefresh = tic;
        while ~feof(fid) && ~S.quit
            [chunk, cnt] = fread(fid, 262144, 'uint8=>uint8');
            if cnt == 0
                break;
            end
            feed(chunk);
            if isfinite(S.cfg.ReplaySpeed) && S.cfg.ReplaySpeed > 0
                pause(cnt / (bytesPerSec * S.cfg.ReplaySpeed));
            end
            if toc(lastRefresh) >= S.cfg.RefreshS
                lastRefresh = tic;
                refresh_ui(false);
            end
            if check_stop()
                break;
            end
        end
        fclose(fid);
        pause(0.01);
    end

    function tf = check_stop()
        tf = S.quit;
        if ~tf && S.cfg.MaxFrames > 0 && S.nRawimu >= S.cfg.MaxFrames
            tf = true;
        end
        if ~tf && S.cfg.DurationS > 0 && toc(S.tStart) >= S.cfg.DurationS
            tf = true;
        end
    end

    function feed(bytes)
        if isempty(bytes)
            return;
        end
        S.buf = [S.buf; uint8(bytes(:))];
        S.nBytes = S.nBytes + numel(bytes);
        parse_buffer();
    end

    % ---------------------------------------------------------------
    % 字节流 -> 帧：找同步头、取长度、校 CRC、按消息 ID 分发
    % ---------------------------------------------------------------
    function parse_buffer()
        buf = S.buf;
        n = numel(buf);
        i = 1;
        while i <= n - 15            % 至少够一个短帧头
            if buf(i) ~= 170 || buf(i+1) ~= 68          % 0xAA 0x44
                i = i + 1;
                S.nDrop = S.nDrop + 1;
                continue;
            end
            s3 = double(buf(i+2));
            if s3 == 18                 % 0x12 -> 28 字节长头
                hlen = 28;
            elseif s3 == 19 || s3 == 16 || s3 == 17     % 0x13/0x10/0x11 -> 12 字节短头
                hlen = 12;
            else
                i = i + 1;
                S.nDrop = S.nDrop + 1;
                continue;
            end
            if (n - i + 1) < (hlen + 4)
                break;                  % 连头+CRC 都还没收全
            end
            if hlen == 28
                len = double(buf(i+8)) + 256 * double(buf(i+9));
            else
                len = double(buf(i+3));
            end
            total = hlen + len + 4;
            if (n - i + 1) < total
                break;                  % 帧未收全，等下一批字节
            end

            if S.cfg.CheckCrc
                crc = double(novatel_crc32(buf(i:i+hlen+len-1)));
                k = i + total - 4;
                crcRx = double(buf(k)) + 256 * double(buf(k+1)) + ...
                        65536 * double(buf(k+2)) + 16777216 * double(buf(k+3));
                if crc ~= crcRx
                    S.nCrcErr = S.nCrcErr + 1;
                    i = i + 1;          % 从下一个字节重新找同步，避免整段丢帧
                    S.nDrop = S.nDrop + 1;
                    continue;
                end
            end

            S.nFrame = S.nFrame + 1;
            id = double(buf(i+4)) + 256 * double(buf(i+5));
            S.idCounts(id + 1) = S.idCounts(id + 1) + 1;   % 消息 ID 直方图（排错用）
            if hlen == 28 && id == 268 && len == 40          % 0x010C RAWIMU
                handle_rawimu(buf(i+28 : i+27+40), 'RAWIMU');
            elseif hlen == 12 && id == 1462 && len == 40     % 0x05B6 RAWIMUSX
                handle_rawimu(buf(i+12 : i+11+40), 'RAWIMUSX');
            elseif hlen == 28
                S.nOther = S.nOther + 1;
            else
                S.nOtherShort = S.nOtherShort + 1;
            end
            i = i + total;
        end

        if i > 1
            S.buf = buf(i:end);
        end
        % 长时间无同步头时防止缓冲区无限增长
        if numel(S.buf) > 1048576
            S.buf = S.buf(end-4095:end);
            S.nDrop = S.nDrop + 1;
        end
    end

    % ---------------------------------------------------------------
    % RAWIMU / RAWIMUSX 解包：物理量换算 + 姿态积分
    % ---------------------------------------------------------------
    function handle_rawimu(body, kind)
        if numel(body) < 40
            return;
        end
        if strcmp(kind, 'RAWIMUSX')
            % 体: imu_info(u8) | imu_type(u8) | Week(u16) | sec(double) | status(u32) | 6x i32
            S.imuInfo = double(body(1));
            S.imuType = double(body(2));
            weekB = double(typecast(body(3:4), 'uint16'));
        else
            % 体: Week(u32) | sec(double) | status(u32) | 6x i32
            weekB = double(typecast(body(1:4), 'uint32'));
        end
        S.imuKind = kind;
        sec   = typecast(body(5:12), 'double');
        st    = typecast(body(13:16), 'uint32');
        rawv  = typecast(body(17:40), 'int32');       % [Zacc -Yacc Xacc Zgyr -Ygyr Xgyr]

        % 轴向：仓库约定（python_mat_read.m / nav_imu_data.m）为 X、-Y、Z
        accRaw = [double(rawv(3)); double(rawv(2)); double(rawv(1))];
        gyrRaw = [double(rawv(6)); double(rawv(5)); double(rawv(4))];
        if S.cfg.InvertY
            accRaw(2) = -accRaw(2);
            gyrRaw(2) = -gyrRaw(2);
        end
        % 安装方向可再映射：AxisMap 指定“显示轴 i 取哪个原始轴”，AxisSign 定符号
        m = S.cfg.AxisMap;  sgn = S.cfg.AxisSign(:);
        accRaw = accRaw(m) .* sgn;
        gyrRaw = gyrRaw(m) .* sgn;
        S.rawG = gyrRaw;                     % 留原始计数，面板/CSV 用
        S.rawA = accRaw;

        % ---- 加计标度自校准：把开头 BiasSec 秒当作静止，用 |a|≡g 归一 ----
        if S.cfg.AutoAccelScale && ~S.accCalDone
            S.accCalSum = S.accCalSum + norm(accRaw);
            S.accCalN = S.accCalN + 1;
            if S.accCalN >= max(10, round(S.cfg.BiasSec / S.cfg.DtNominal))
                lsbNew = 9.80665 / (S.accCalSum / S.accCalN);
                if lsbNew > 0.5 * S.cfg.AccelLsbMps2 && lsbNew < 2 * S.cfg.AccelLsbMps2
                    S.cfg.AccelLsbMps2 = lsbNew;
                    if S.cfg.Verbose
                        fprintf(['[RAWIMU] 加计标度自校准：AccelLsbMps2 = %.6g m/s²/LSB' ...
                            '（静止 |a| 归一到 9.80665）\n'], S.cfg.AccelLsbMps2);
                    end
                elseif S.cfg.Verbose
                    fprintf('[RAWIMU] 加计自校准跳过：估计 %.3g 与预设 %.3g 相差过大（开头可能不是静止）\n', ...
                        lsbNew, S.cfg.AccelLsbMps2);
                end
                S.accCalDone = true;
            end
        end

        gyrDeg  = gyrRaw * S.cfg.GyroLsbDeg;
        accMps2 = accRaw * S.cfg.AccelLsbMps2;

        traw   = double(typecast(uint16(bitand(bitshift(st, -16), uint32(65535))), 'int16'));
        tempC  = traw * S.cfg.TempLsb + S.cfg.TempOffset;
        statLo = bitand(st, uint32(65535));
        statB  = bitand(bitshift(st, -8), uint32(255));
        cnt    = bitand(st, uint32(255));

        % ---- 时间与 dt ----
        tAbs = weekB * 604800 + sec;
        if isfinite(S.tAbsPrev)
            dt = tAbs - S.tAbsPrev;
            if ~(dt > 0 && dt < 0.5)
                dt = S.cfg.DtNominal;
                S.nDtFix = S.nDtFix + 1;
            end
        else
            dt = S.cfg.DtNominal;
        end
        S.tAbsPrev = tAbs;
        if ~isfinite(S.tRef)
            S.tRef = tAbs;
        end
        tRel = tAbs - S.tRef;
        S.tLast = tRel;
        S.dtSum = S.dtSum + dt;
        S.dtN = S.dtN + 1;

        % ---- 温度 / 加速度统计 ----
        S.tempMin = min(S.tempMin, tempC);
        S.tempMax = max(S.tempMax, tempC);
        an = norm(accMps2);
        S.accNormSum = S.accNormSum + an;
        S.accNormN = S.accNormN + 1;
        S.statusByteSeen(double(statB) + 1) = S.statusByteSeen(double(statB) + 1) + 1;
        if isfinite(S.counterPrev)
            step = mod(double(cnt) - S.counterPrev, 256);
            S.counterSteps(step + 1) = S.counterSteps(step + 1) + 1;
        end
        S.counterPrev = double(cnt);

        % ---- 陀螺：增量制(delta)还是速率制(rate) ----
        if S.cfg.DeltaMode
            gyrRate = gyrDeg / dt;
        else
            gyrRate = gyrDeg;
        end

        % ---- 静止零偏 ----
        if S.cfg.AutoBias && ~S.biasDone
            S.biasAcc = S.biasAcc + gyrRate;
            S.biasN = S.biasN + 1;
            if S.biasN >= max(10, round(S.cfg.BiasSec / S.cfg.DtNominal))
                S.bias = S.biasAcc / S.biasN;
                S.biasDone = true;
                if S.cfg.Verbose
                    fprintf('[RAWIMU] 零偏估计完成 [%.4f %.4f %.4f] deg/s（静止首 %.1fs）\n', ...
                        S.bias(1), S.bias(2), S.bias(3), S.cfg.BiasSec);
                    if norm(S.bias) > 0.5
                        fprintf(['[RAWIMU] 注意：零偏估计值偏大 (%.2f deg/s)。若设备在这 %.1f s 内\n' ...
                                 '         本来就在转动，请改用 ''AutoBias'', false。\n'], ...
                                 norm(S.bias), S.cfg.BiasSec);
                    end
                end
            end
        end
        wDeg = gyrRate - S.bias;
        S.angAcc = S.angAcc + wDeg(:).' * dt;    % 累计转角，用于核对陀螺量程/单位

        % 由重力方向算出的 roll/pitch：只作参考与慢修正，绝不参与主姿态
        rollA  = atan2(accMps2(2), accMps2(3));
        pitchA = atan2(-accMps2(1), hypot(accMps2(2), accMps2(3)));

        % ---- 1) 姿态主体：陀螺角速度四元数积分 ----
        S.q = quat_propagate(S.q, wDeg * pi/180, dt);
        [roll, pitch, yaw] = quat_to_euler(S.q);

        % ---- 2) 可选：加速度慢修正（互补滤波；只修 roll/pitch，永不修 yaw） ----
        if S.cfg.AccelLeveling && strcmpi(S.cfg.AttitudeMode, 'complementary')
            if abs(an - 9.80665) < 0.6 && norm(wDeg) < 20
                k = min(0.2, dt / max(0.05, S.cfg.LevelTau));
                roll  = roll  + k * wrap_pi(rollA  - roll);
                pitch = pitch + k * wrap_pi(pitchA - pitch);
                S.q = euler_to_quat(roll, pitch, yaw);
                [roll, pitch, yaw] = quat_to_euler(S.q);
            end
        end

        % ---- 入环形缓冲 + 落盘 ----
        push_sample(tRel, [rad2deg(roll), rad2deg(pitch), rad2deg(yaw)], ...
                    wDeg, accMps2, [rad2deg(rollA), rad2deg(pitchA)]);
        S.nRawimu = S.nRawimu + 1;
        S.tempNow = tempC;
        S.statusNow = statLo;
        S.statusByteNow = statB;
        S.counterNow = cnt;

        if S.csvFid > 0
            fprintf(S.csvFid, ['%.6f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,' ...
                '%.2f,%u,%u,%u,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n'], ...
                tRel, rad2deg(roll), rad2deg(pitch), rad2deg(yaw), ...
                wDeg(1), wDeg(2), wDeg(3), accMps2(1), accMps2(2), accMps2(3), ...
                tempC, statLo, statB, cnt, ...
                gyrRaw(1), gyrRaw(2), gyrRaw(3), accRaw(1), accRaw(2), accRaw(3));
        end
    end

    function push_sample(t, eul, w, a, eulA)
        idx = mod(S.idx, S.N) + 1;
        S.data(idx, :) = [t, eul(1), eul(2), eul(3), w(1), w(2), w(3), ...
                          a(1), a(2), a(3), eulA(1), eulA(2)];
        S.idx = idx;
        S.count = min(S.count + 1, S.N);
    end

    function D = ordered_data()
        if S.count < S.N
            D = S.data(1:S.count, :);
        else
            D = [S.data(S.idx+1:S.N, :); S.data(1:S.idx, :)];
        end
    end

    function on_key(evt)
        switch evt.Key
            case 'escape'
                S.quit = true;
            case 'r'
                S.q = [1; 0; 0; 0];
                fprintf('[RAWIMU] 姿态已归零\n');
            case 'z'
                S.bias = [0; 0; 0];
                S.biasAcc = [0; 0; 0];
                S.biasN = 0;
                S.biasDone = false;
                fprintf('[RAWIMU] 重新估计零偏，请保持静止 %.1f s\n', S.cfg.BiasSec);
        end
    end

    % ---------------------------------------------------------------
    % 界面
    % ---------------------------------------------------------------
    function init_ui()
        fn = S.cfg.FontName;
        S.fig = figure('Name', 'RAWIMU 实时姿态', 'Color', 'w', ...
            'Position', [60 50 1480 840], 'Visible', S.cfg.Visible, ...
            'MenuBar', 'none', 'ToolBar', 'none', 'NumberTitle', 'off', 'Renderer', 'opengl');
        S.fig.WindowKeyPressFcn = @(~, evt) on_key(evt);
        tl = tiledlayout(S.fig, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

        % ---- 三维姿态（天空 + 地面 + 罗盘 + 飞机 + 投影） ----
        ax = nexttile(tl, [2 1]);
        hold(ax, 'on'); grid(ax, 'off'); box(ax, 'off');
        al = S.cfg.AxisLimit;
        axis(ax, [-al al -al al -0.70*al 1.15*al]);
        daspect(ax, [1 1 1]);
        view(ax, S.cfg.ViewAz, S.cfg.ViewEl);
        set(ax, 'CameraUpVector', [0 0 -1], 'Projection', 'orthographic', ...
            'FontName', fn, 'FontSize', 9, 'XTick', [], 'YTick', [], 'ZTick', [], ...
            'Color', [0.78 0.87 0.99]);          % 天空底色
        S.h.ax3d = ax;
        S.h.att = text(ax, 0.02, 0.97, '', 'Units', 'normalized', ...
            'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
            'FontName', fn, 'FontSize', 11, 'FontWeight', 'bold', ...
            'BackgroundColor', [1 1 1], 'Margin', 3, 'Interpreter', 'none');

        zg = 0.90 * al;                           % 姿态台高度（Z 向下 = 机体下方）
        S.zGround = zg;
        rp = 0.90 * al;                           % 圆形“姿态台”半径
        S.rc = 0.80 * rp;
        th = linspace(0, 2*pi, 241);
        patch(ax, 'XData', rp*cos(th), 'YData', rp*sin(th), ...
            'ZData', zg * ones(size(th)), 'FaceColor', [0.89 0.94 0.85], ...
            'EdgeColor', [0.62 0.72 0.58], 'LineWidth', 1.5);
        for fr = [0.40 0.62 0.80]
            plot3(ax, rp*fr*cos(th), rp*fr*sin(th), (zg - 0.008)*ones(size(th)), ...
                'Color', [0.80 0.87 0.78]);
        end
        for a = 0:30:330
            r1 = rp * 0.93;  r2 = rp * 0.99;
            plot3(ax, [r1 r2]*cosd(a), [r1 r2]*sind(a), [zg zg] - 0.008, ...
                'Color', [0.55 0.65 0.52], 'LineWidth', 1.0);
        end
        dname = {'N', 'E', 'S', 'W'};
        dang = [0 90 180 270];
        for k = 1:4
            text(ax, 0.88*rp*cosd(dang(k)), 0.88*rp*sind(dang(k)), zg - 0.012, ...
                dname{k}, 'FontName', fn, 'FontSize', 12, 'FontWeight', 'bold', ...
                'Color', [0.25 0.45 0.25], 'HorizontalAlignment', 'center');
        end

        % 飞机：多色部件 + 地面投影阴影
        S.parts = aircraft_parts();
        for k = 1:numel(S.parts)
            S.h.shadow(k) = patch(ax, 'Vertices', S.parts(k).V, 'Faces', S.parts(k).F, ...
                'FaceColor', [0.18 0.24 0.18], 'EdgeColor', 'none', 'FaceAlpha', 0.35);
            S.h.part(k) = patch(ax, 'Vertices', S.parts(k).V, 'Faces', S.parts(k).F, ...
                'FaceColor', S.parts(k).Color, 'EdgeColor', [0.20 0.22 0.26], ...
                'LineWidth', 0.5, 'FaceLighting', 'none');
        end

        L = 1.55;
        S.h.tri(1) = plot3(ax, [0 L], [0 0], [0 0], 'r', 'LineWidth', 2.0);
        S.h.tri(2) = plot3(ax, [0 0], [0 L], [0 0], 'g', 'LineWidth', 2.0);
        S.h.tri(3) = plot3(ax, [0 0], [0 0], [0 L], 'b', 'LineWidth', 2.0);
        S.h.tlab(1) = text(ax, L, 0, 0, 'X', 'Color', 'r', 'FontWeight', 'bold', 'FontName', fn);
        S.h.tlab(2) = text(ax, 0, L, 0, 'Y', 'Color', [0 0.6 0], 'FontWeight', 'bold', 'FontName', fn);
        S.h.tlab(3) = text(ax, 0, 0, L, 'Z', 'Color', 'b', 'FontWeight', 'bold', 'FontName', fn);

        % 航向指针：机头方向投影到地面姿态台上
        S.h.headline = plot3(ax, [0 S.rc], [0 0], [zg zg] - 0.02, 'Color', [0.80 0.15 0.15], ...
            'LineWidth', 2.5);

        % ---- 姿态角 ----
        ax = nexttile(tl);
        hold(ax, 'on'); grid(ax, 'on');
        S.h.eul(1) = plot(ax, nan, nan, 'r', 'LineWidth', 1.2);
        S.h.eul(2) = plot(ax, nan, nan, 'g', 'LineWidth', 1.2);
        S.h.eul(3) = plot(ax, nan, nan, 'b', 'LineWidth', 1.2);
        % 由重力反算的 roll/pitch（虚线参考，只用于对照，不参与姿态）
        S.h.eul(4) = plot(ax, nan, nan, '--', 'Color', [0.75 0.35 0.35], 'LineWidth', 0.9);
        S.h.eul(5) = plot(ax, nan, nan, '--', 'Color', [0.35 0.65 0.35], 'LineWidth', 0.9);
        ylim(ax, [-200 200]); ylabel(ax, 'deg');
        title(ax, '姿态角（实线=陀螺积分  虚线=重力反算  红roll 绿pitch 蓝yaw）', ...
            'FontName', fn, 'FontSize', 9);
        set(ax, 'FontName', fn, 'FontSize', 9);

        % ---- 陀螺 ----
        ax = nexttile(tl);
        hold(ax, 'on'); grid(ax, 'on');
        S.h.gyr(1) = plot(ax, nan, nan, 'r', 'LineWidth', 1.0);
        S.h.gyr(2) = plot(ax, nan, nan, 'g', 'LineWidth', 1.0);
        S.h.gyr(3) = plot(ax, nan, nan, 'b', 'LineWidth', 1.0);
        ylabel(ax, 'deg/s');
        title(ax, '陀螺角速度（红 X / 绿 Y / 蓝 Z，已去零偏）', 'FontName', fn);
        set(ax, 'FontName', fn, 'FontSize', 9);

        % ---- 加速度 ----
        ax = nexttile(tl);
        hold(ax, 'on'); grid(ax, 'on');
        S.h.acc(1) = plot(ax, nan, nan, 'r', 'LineWidth', 1.0);
        S.h.acc(2) = plot(ax, nan, nan, 'g', 'LineWidth', 1.0);
        S.h.acc(3) = plot(ax, nan, nan, 'b', 'LineWidth', 1.0);
        ylabel(ax, 'm/s^2');
        title(ax, '加速度（红 X / 绿 Y / 蓝 Z）', 'FontName', fn);
        set(ax, 'FontName', fn, 'FontSize', 9);

        % ---- 状态面板 ----
        ax = nexttile(tl);
        axis(ax, 'off');
        title(ax, '状态', 'FontName', fn, 'FontSize', 10);
        S.h.info = text(ax, 0.02, 0.94, '', 'VerticalAlignment', 'top', ...
            'HorizontalAlignment', 'left', 'FontName', 'Consolas', ...
            'FontSize', 9, 'Interpreter', 'none');
        S.h.infoAx = ax;
    end

    function refresh_ui(force)
        if S.count == 0 && ~force
            return;
        end
        if isempty(S.fig)
            init_ui();
        elseif ~ishghandle(S.fig)
            S.quit = true;      % 用户关掉了窗口
            return;
        end
        D = ordered_data();
        if isempty(D)
            return;
        end

        t = D(:, 1);
        sel = t >= (t(end) - S.cfg.WindowS);
        ts = t(sel);

        yawW = mod(D(sel, 4) + 180, 360) - 180;
        set(S.h.eul(1), 'XData', ts, 'YData', D(sel, 2));
        set(S.h.eul(2), 'XData', ts, 'YData', D(sel, 3));
        set(S.h.eul(3), 'XData', ts, 'YData', yawW);
        set(S.h.eul(4), 'XData', ts, 'YData', D(sel, 11));
        set(S.h.eul(5), 'XData', ts, 'YData', D(sel, 12));
        set(S.h.gyr(1), 'XData', ts, 'YData', D(sel, 5));
        set(S.h.gyr(2), 'XData', ts, 'YData', D(sel, 6));
        set(S.h.gyr(3), 'XData', ts, 'YData', D(sel, 7));
        set(S.h.acc(1), 'XData', ts, 'YData', D(sel, 8));
        set(S.h.acc(2), 'XData', ts, 'YData', D(sel, 9));
        set(S.h.acc(3), 'XData', ts, 'YData', D(sel, 10));
        if numel(ts) > 1
            xlim(S.h.eul(1).Parent, [ts(1) max(ts(1) + 1, ts(end))]);
        end

        % 三维：机体坐标系 -> 导航系（飞机旋转，地面/罗盘不动，投影当阴影）
        R = quat_to_dcm(S.q);
        for k = 1:numel(S.parts)
            Vi = S.parts(k).V * R.';
            set(S.h.part(k), 'Vertices', Vi);
            % 阴影：沿固定“阳光”方向平移到地面，便于判断离地高度与姿态
            Vs = Vi;
            Vs(:, 1) = Vs(:, 1) + 0.30;
            Vs(:, 2) = Vs(:, 2) + 0.22;
            Vs(:, 3) = S.zGround - 0.03;
            set(S.h.shadow(k), 'Vertices', Vs);
        end
        L = 1.55;
        for k = 1:3
            e = R(:, k) * L;
            set(S.h.tri(k), 'XData', [0 e(1)], 'YData', [0 e(2)], 'ZData', [0 e(3)]);
            set(S.h.tlab(k), 'Position', 1.12 * e);
        end
        % 机头方向在罗盘上的投影（航向指针）
        set(S.h.headline, 'XData', [0 R(1, 1) * S.rc], ...
            'YData', [0 R(2, 1) * S.rc], ...
            'ZData', [S.zGround S.zGround] - 0.02);

        % 帧率由数据时间戳算（回放/串口都适用）
        nWin = min(size(D, 1), max(2, round(1 / S.cfg.DtNominal) + 1));
        if nWin > 1
            span = D(end, 1) - D(end - nWin + 1, 1);
            S.rateHz = (nWin - 1) / max(1e-6, span);
        end

        [roll, pitch, yaw] = quat_to_euler(S.q);
        roll = rad2deg(roll); pitch = rad2deg(pitch); yaw = rad2deg(yaw);
        set(S.h.att, 'String', sprintf('姿态（机体轴 X前 / Y右 / Z下）\nroll %+7.2f°\npitch %+7.2f°\nyaw %+7.2f°', ...
            roll, pitch, yaw));

        if S.nRawimu > 0
            aMean = S.accNormSum / max(1, S.accNormN);
            dtMean = S.dtSum / max(1, S.dtN);
            lines = {
                sprintf('%-9s: %7.1f Hz', 'Rate', S.rateHz)
                sprintf('%-9s: %7.2f ms', 'Interval', dtMean * 1000)
                sprintf('%-9s: %7.2f C', 'Temp', S.tempNow)
                sprintf('%-9s: %7.3f m/s2', '|a|', aMean)
                sprintf('%-9s: 0x%04X', 'Status', double(S.statusNow))
                sprintf('%-9s: 0x%02X', 'StatByte', double(S.statusByteNow))
                sprintf('%-9s: %7d', 'Counter', double(S.counterNow))
                sprintf('%-9s: %s', 'Bias dps', bias_text())
                sprintf('%-9s: %s', 'Mode', mode_text())
                sprintf('%-9s: %5.1f %5.1f %5.1f', 'IntAngle', S.angAcc(1), S.angAcc(2), S.angAcc(3))
                sprintf('%-9s: %.3g %.3g %.3g', 'RawGyro', S.rawG(1), S.rawG(2), S.rawG(3))
                ''
                sprintf('%-9s: %7d', 'RAWIMU', S.nRawimu)
                sprintf('%-9s: %7d', 'Frames', S.nFrame)
                sprintf('%-9s: %7d', 'ShortHdr', S.nOtherShort)
                sprintf('%-9s: %7d', 'CRC err', S.nCrcErr)
                sprintf('%-9s: %7d', 'SkipBytes', S.nDrop)
                sprintf('%-9s: %7.1f kB', 'Bytes', S.nBytes / 1024)
                };
            set(S.h.info, 'String', lines);
        end

        drawnow limitrate;
    end

    function s = bias_text()
        if ~S.cfg.AutoBias
            s = 'off';
        elseif ~S.biasDone
            s = sprintf('est %3d%%', round(100 * S.biasN * S.cfg.DtNominal / S.cfg.BiasSec));
        else
            s = sprintf('%.3f %.3f %.3f', S.bias(1), S.bias(2), S.bias(3));
        end
    end

    function s = mode_text()
        if strcmpi(S.cfg.AttitudeMode, 'complementary') && S.cfg.AccelLeveling
            s = sprintf('gyro+accel(tau=%.1fs)', S.cfg.LevelTau);
        else
            s = 'gyro only';
        end
    end

    % ---------------------------------------------------------------
    function finishup()
        if S.finished
            return;
        end
        S.finished = true;
        if isfield(S, 'port') && ~isempty(S.port)
            try
                delete(S.port);
            catch
            end
        end
        if S.csvFid > 0
            try
                fclose(S.csvFid);
                fprintf('[RAWIMU] 已保存 CSV：%s\n', S.cfg.SaveCsv);
            catch
            end
        end

        if ~isempty(S.fig) && ishghandle(S.fig) && ~isempty(S.cfg.Snapshot)
            try
                exportgraphics(S.fig, S.cfg.Snapshot, 'Resolution', 140);
            catch
                try
                    saveas(S.fig, S.cfg.Snapshot);
                catch
                end
            end
            fprintf('[RAWIMU] 已保存界面截图：%s\n', S.cfg.Snapshot);
        end

        fprintf('\n================ RAWIMU 统计 ================\n');
        fprintf('字节 %d, 帧 %d (RAWIMU %d, 其它 %d, 短头 %d)\n', ...
            S.nBytes, S.nFrame, S.nRawimu, S.nOther, S.nOtherShort);
        fprintf('CRC 错 %d, 重同步丢弃字节 %d, dt 修正 %d 次\n', ...
            S.nCrcErr, S.nDrop, S.nDtFix);
        if S.nRawimu > 0
            fprintf('解析的 IMU 报文：%s', S.imuKind);
            if isfinite(S.imuInfo)
                fprintf('（imu_info=%d, imu_type=0x%02X）', S.imuInfo, S.imuType);
            end
            fprintf('\n');
            fprintf('平均帧间隔 %.3f ms (%.2f Hz), 温度 %.2f..%.2f °C, 平均|a| %.3f m/s²\n', ...
                1000 * S.dtSum / max(1, S.dtN), S.nRawimu / max(1e-6, S.dtSum), ...
                S.tempMin, S.tempMax, S.accNormSum / max(1, S.accNormN));
            fprintf('姿态算法：%s\n', mode_text());
            fprintf('累计转角（陀螺积分）：X %+8.1f  Y %+8.1f  Z %+8.1f deg\n', S.angAcc);
            fprintf(['  核对陀螺量纲/零偏的方法：把设备绕某个轴慢慢转 90°，上面的累计转角应≈90°；\n' ...
                     '  若只走了 ~1°，说明 GyroLsbDeg 小了两个数量级（RAWIMUSX 的陀螺 LSB 需按型号改）。\n']);
            used = find(S.statusByteSeen > 0);
            fprintf('出现过的状态字节: %s\n', mat2str(used - 1));
            used = find(S.counterSteps > 0);
            fprintf('采样计数步长(模256)分布: %s\n', mat2str([used - 1; S.counterSteps(used)]));
            if S.accNormSum / max(1, S.accNormN) < 5
                fprintf(['注意：平均|a| 远小于 9.81，加速度换算是这个型号最不确定的一项，\n' ...
                         '      可按比例调整 AccelLsbMps2（当前 %.6g）。\n'], S.cfg.AccelLsbMps2);
            end
        else
            fprintf('没有解到任何 RAWIMU 帧：检查波特率、端口是否输出 RAWIMU、CRC/同步头设置。\n');
        end
        if S.nFrame > 0
            [c, ord] = sort(S.idCounts, 'descend');
            nShow = min(8, sum(c > 0));
            fprintf('实际收到的消息 ID（Top%d）:\n', nShow);
            for k = 1:nShow
                fprintf('  0x%04X : %d 帧\n', ord(k) - 1, c(k));
            end
        end
        fprintf('=============================================\n');
    end
end

% ============================ 局部函数 ============================

function cfg = default_config()
cfg = struct();
cfg.Source      = 'serial';     % 'serial' | 'file'
cfg.Port        = 'COM49';      % 串口名：serialportlist("available")
cfg.Baud        = 921600;       % 波特率，按实际设备设置
cfg.TimeoutS    = 2;
cfg.InitCmd     = '';           % 打开后可发的配置命令，如 'LOG COM1 RAWIMUA ONTIME 0.01'
cfg.ImuModel    = 'M1633';      % 预设：M1633 / B240 / B240_16BIT / B200 / X2_2
cfg.GyroLsbDeg  = NaN;          % 覆写：deg/s per LSB（NaN=用预设）
cfg.AccelLsbMps2= NaN;          % 覆写：m/s^2 per LSB
cfg.TempLsb     = NaN;          % 覆写：°C per LSB
cfg.TempOffset  = NaN;
cfg.Freq        = NaN;          % 覆写：标称采样率 Hz
cfg.InvertY     = true;         % 仓库约定：Y 轴取反（python_mat_read.m）
cfg.AxisMap     = [1 2 3];      % 安装方向映射：显示轴 i 取哪个原始轴
cfg.AxisSign    = [1 1 1];      % 对应轴符号
cfg.DeltaMode   = false;        % true = 报文里是“每帧增量”而不是速率
cfg.CheckCrc    = true;
cfg.AttitudeMode = 'complementary'; % 'gyro'=纯陀螺积分 | 'complementary'=陀螺积分+加速度慢修正
cfg.AccelLeveling = true;       % complementary 时是否用加速度修 roll/pitch
cfg.LevelTau    = 2;            % 调平时间常数 s（越大越依赖陀螺）
cfg.AutoBias    = true;         % 开头静止若干秒自动估计零偏
cfg.AutoAccelScale = true;      % 同一段静止数据里把 |a| 归一到 9.80665，自校准加计标度
cfg.BiasSec     = 3;
cfg.WindowS     = 30;           % 曲线显示时间窗 s
cfg.RefreshS    = 0.1;          % 界面刷新周期 s
cfg.ViewAz      = -125;         % 三维视角方位角
cfg.ViewEl      = -26;          % 三维视角俯仰角（负值=相机在机身上方看下来）
cfg.AxisLimit   = 2.20;         % 三维场景范围 ±AxisLimit
cfg.MaxSamples  = 20000;        % 环形缓冲长度（20 s @100Hz）
cfg.SaveCsv     = '';           % 非空则逐帧写 CSV
cfg.Snapshot    = '';           % 非空则退出时保存界面截图
cfg.Visible     = 'on';
cfg.File        = '';           % Source='file' 时的原始 log
cfg.ReplaySpeed = Inf;          % 回放倍速（Inf=全速，1=按波特率节流）
cfg.MaxFrames   = 0;            % >0 则收到这么多 RAWIMU 帧后退出
cfg.DurationS   = 0;            % >0 则运行这么多秒后退出
cfg.Verbose     = 1;
cfg.FontName    = 'Microsoft YaHei';
end

function cfg = apply_overrides(cfg, args)
if mod(numel(args), 2) ~= 0
    error('参数必须成对出现：''Name'', Value');
end
f = fieldnames(cfg);
for k = 1:2:numel(args)
    idx = find(strcmpi(f, args{k}), 1);
    if isempty(idx)
        error('未知参数 "%s"。可用参数：%s', args{k}, strjoin(f(:).', ', '));
    end
    cfg.(f{idx}) = args{k+1};
end
end

function cfg = resolve_scales(cfg)
p = imu_preset(cfg.ImuModel);
if isnan(cfg.GyroLsbDeg),   cfg.GyroLsbDeg   = p.GyroLsbDeg;   end
if isnan(cfg.AccelLsbMps2), cfg.AccelLsbMps2 = p.AccelLsbMps2; end
if isnan(cfg.TempLsb),      cfg.TempLsb      = p.TempLsb;      end
if isnan(cfg.TempOffset),   cfg.TempOffset   = p.TempOffset;   end
if isnan(cfg.Freq),         cfg.Freq         = p.Freq;         end
cfg.DtNominal = 1 / cfg.Freq;
end

function p = imu_preset(name)
% 换算系数来源：
%   陀螺/加速度 与 GNSS_NAV/src/processing/load_imu_model.m 中同一型号的“单位”一致，
%   但整体再乘 100：实测这台设备(RAWIMUSX, 100 Hz)的陀螺是
%   “0.01 deg/s 为单位、16 位小数”的 32 位定标值，即 1 LSB = 1/65536 deg/s；
%   load_imu_model.m 里写的是 1/(100*2^16)，小了整 100 倍（用 0.01deg/s 单位又除了 100）。
%
%   验证方法（2026-10-20 实测，COM49@460800）：
%     手转 95°，∫ωx·dt 用 1/65536 积分得 95.2°，用 1/(100*2^16) 只得 0.95°；
%     静止噪声 0.005 deg/s（1/65536）vs 0.00006 deg/s（旧值，明显不像 MEMS）。
%   加速度同样乘过 100：用 G*100/(2000*2^16) 时静止 |a|=9.6~9.8，用原值是 0.098。
G = 9.80665;
switch upper(name)
    case 'M1633'
        p = struct('Freq', 100, 'GyroLsbDeg', 1/65536, ...
            'AccelLsbMps2', 0.5/65536, 'TempLsb', 1/100, 'TempOffset', 0);
    case 'B240'
        p = struct('Freq', 100, 'GyroLsbDeg', 1/65536, ...
            'AccelLsbMps2', 0.5/65536, 'TempLsb', 1/256, 'TempOffset', 25);
    case 'B240_16BIT'
        p = struct('Freq', 100, 'GyroLsbDeg', 1/100, ...
            'AccelLsbMps2', G/2000, 'TempLsb', 1/256, 'TempOffset', 25);
    case 'B200'
        p = struct('Freq', 100, 'GyroLsbDeg', 1/65536, ...
            'AccelLsbMps2', G*100/5000/65536, 'TempLsb', 1/200, 'TempOffset', 50);
    case 'X2_2'
        p = struct('Freq', 100, 'GyroLsbDeg', 100/80/65536, ...
            'AccelLsbMps2', G/4905/65536, 'TempLsb', 1/30, 'TempOffset', 25);
    otherwise
        error('未知 IMU 预设 "%s"，可用：M1633 / B240 / B240_16BIT / B200 / X2_2\n' , name);
end
end

function S = init_state(cfg)
S = struct();
S.cfg = cfg;
S.buf = zeros(0, 1, 'uint8');
S.nBytes = 0; S.nFrame = 0; S.nRawimu = 0; S.nOther = 0; S.nOtherShort = 0;
S.nCrcErr = 0; S.nDrop = 0; S.nDtFix = 0;
S.idCounts = zeros(1, 65536);
S.imuInfo = NaN; S.imuType = NaN; S.imuKind = '';

S.tAbsPrev = NaN;
S.tRef = NaN; S.tLast = NaN;
S.dtSum = 0; S.dtN = 0;

S.q = [1; 0; 0; 0];
S.bias = [0; 0; 0]; S.biasAcc = [0; 0; 0]; S.biasN = 0; S.biasDone = false;

S.tempMin = Inf; S.tempMax = -Inf; S.tempNow = NaN;
S.angAcc = [0 0 0];
S.accCalSum = 0; S.accCalN = 0; S.accCalDone = false;
S.accNormSum = 0; S.accNormN = 0;
S.statusNow = uint32(0); S.statusByteNow = uint32(0); S.counterNow = uint32(0);
S.statusByteSeen = zeros(1, 256);
S.counterSteps = zeros(1, 256);
S.counterPrev = NaN;

S.N = max(100, round(cfg.MaxSamples));
S.data = nan(S.N, 12);
S.idx = 0; S.count = 0;

S.fig = []; S.h = struct();
S.port = [];
S.quit = false; S.finished = false;
S.tStart = tic;
S.lastRateT = 0; S.lastRateN = 0; S.rateHz = 0;

S.csvFid = -1;
if ~isempty(cfg.SaveCsv)
    S.csvFid = fopen(cfg.SaveCsv, 'w');
    if S.csvFid < 0
        warning('无法写 CSV：%s', cfg.SaveCsv);
        S.csvFid = -1;
    else
        fprintf(S.csvFid, ['t_s,roll_deg,pitch_deg,yaw_deg,wx_dps,wy_dps,wz_dps,' ...
            'ax_mps2,ay_mps2,az_mps2,temp_C,status,status_byte,counter,' ...
            'wx_raw,wy_raw,wz_raw,ax_raw,ay_raw,az_raw\n']);
    end
end
end

% ---------------------------- 姿态数学 ----------------------------

function q = quat_propagate(q, wRad, dt)
th = norm(wRad) * dt;
if th < 1e-12
    return;
end
axis = wRad / norm(wRad);
dq = [cos(th/2); sin(th/2) * axis];
q = quat_mul(q, dq);
q = q / norm(q);
end

function q = quat_mul(a, b)
q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
     a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
     a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
     a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end

function R = quat_to_dcm(q)
w = q(1); x = q(2); y = q(3); z = q(4);
R = [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y);
     2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x);
     2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)];
end

function [roll, pitch, yaw] = quat_to_euler(q)
R = quat_to_dcm(q);
pitch = asin(max(-1, min(1, -R(3, 1))));
yaw = atan2(R(2, 1), R(1, 1));
roll = atan2(R(3, 2), R(3, 3));
end

function q = euler_to_quat(roll, pitch, yaw)
cr = cos(roll/2);  sr = sin(roll/2);
cp = cos(pitch/2); sp = sin(pitch/2);
cy = cos(yaw/2);   sy = sin(yaw/2);
q = [cy*cp*cr + sy*sp*sr;      % w
     cy*cp*sr - sy*sp*cr;      % x
     cy*sp*cr + sy*cp*sr;      % y
     sy*cp*cr - cy*sp*sr];     % z
q = q / norm(q);
end

function a = wrap_pi(a)
a = mod(a + pi, 2*pi) - pi;
end

% --------------------------- 机体模型 ---------------------------
% 机体轴 X 前、Y 右、Z 下；返回多个彩色部件，刷新时统一乘旋转矩阵。

function P = aircraft_parts()
P = struct('V', {}, 'F', {}, 'Color', {});

% 机身（白色，尾 -> 前）
[V, F] = tube3([-1.15 -0.85 -0.35 0.55 1.00], [0.015 0.060 0.095 0.095 0.055], 0, 0, 10);
P(end+1) = struct('V', V, 'F', F, 'Color', [0.96 0.96 0.98]);

% 机头（红色，一眼看出朝向）
[V, F] = tube3([1.00 1.34], [0.055 0.010], 0, 0, 10);
P(end+1) = struct('V', V, 'F', F, 'Color', [0.86 0.18 0.18]);

% 主翼（后掠 + 上反角；-Z 为上）
V = [ 0.35 0.10 -0.02; -0.30 0.10 -0.02; -0.55 1.05 -0.11
     -0.10 1.05 -0.11 ];
P(end+1) = struct('V', V, 'F', [1 2 3 4], 'Color', [0.74 0.78 0.84]);
V = [ 0.35 -0.10 -0.02; -0.30 -0.10 -0.02; -0.55 -1.05 -0.11
     -0.10 -1.05 -0.11 ];
P(end+1) = struct('V', V, 'F', [1 2 3 4], 'Color', [0.74 0.78 0.84]);

% 平尾
V = [-0.80 0.09 -0.01; -1.02 0.09 -0.01; -1.16 0.42 -0.04
     -1.00 0.42 -0.04];
P(end+1) = struct('V', V, 'F', [1 2 3 4], 'Color', [0.78 0.81 0.86]);
V = [-0.80 -0.09 -0.01; -1.02 -0.09 -0.01; -1.16 -0.42 -0.04
     -1.00 -0.42 -0.04];
P(end+1) = struct('V', V, 'F', [1 2 3 4], 'Color', [0.78 0.81 0.86]);

% 垂尾（蓝色）
V = [-0.62 0 -0.04; -1.10 0 -0.04; -1.14 0 -0.60; -0.86 0 -0.60];
P(end+1) = struct('V', V, 'F', [1 2 3 4], 'Color', [0.24 0.38 0.62]);

% 两台发动机（机翼下）
[V, F] = tube3([0.30 -0.34], [0.075 0.075], 0.42, 0.16, 8);
P(end+1) = struct('V', V, 'F', F, 'Color', [0.42 0.44 0.48]);
[V, F] = tube3([0.30 -0.34], [0.075 0.075], -0.42, 0.16, 8);
P(end+1) = struct('V', V, 'F', F, 'Color', [0.42 0.44 0.48]);

% 整体放大一点，远看更清楚
sc = 1.25;
for k = 1:numel(P)
    P(k).V = P(k).V * sc;
end
end

function [V, F] = tube3(xs, rs, cy, cz, n)
% 沿 X 轴的圆管：xs 是各截面 x 坐标，rs 是半径，中心在 (0,cy,cz)
V = zeros(numel(xs) * n, 3);
for i = 1:numel(xs)
    for j = 0:n-1
        a = 2*pi*j/n + pi/n;
        V((i-1)*n + j + 1, :) = [xs(i), cy + rs(i)*cos(a), cz + rs(i)*sin(a)];
    end
end
F = zeros((numel(xs) - 1) * n, 4);
row = 0;
for i = 1:numel(xs) - 1
    for j = 1:n
        j2 = mod(j, n) + 1;
        a1 = (i-1)*n + j;  a2 = (i-1)*n + j2;
        b1 = i*n + j;      b2 = i*n + j2;
        row = row + 1;
        F(row, :) = [a1 b1 b2 a2];
    end
end
end