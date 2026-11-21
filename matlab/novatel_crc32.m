function crc = novatel_crc32(bytes)
%NOVATEL_CRC32  Bynav/NovAtel 二进制帧使用的 CRC32（初值 0、末尾不取反）
%
%   crc = novatel_crc32(uint8([...]))
%
%   注意：这不是标准 zlib/iEEE 802.3 CRC32。设备固件使用的是
%   “反射多项式 0xEDB88320 + 寄存器初值 0 + 结果不再异或 0xFFFFFFFF”的变体，
%   等价于仓库 tools/analyze_dpx_pair.py 里的
%       (zlib.crc32(frame[:-4], 0xffffffff) ^ 0xffffffff) & 0xffffffff
%   （已用 reports/dpx_803_804_* 的真实 log 15169 帧全部校验通过）。
%
%   校验范围：帧头 + 数据体（不含末尾 4 字节 CRC 本身）。
%
%   见 rawimu_live.m / rawimu_make_testfile.m。

bytes = uint8(bytes(:));
crc = uint32(0);
tbl = crc_table();
for k = 1:numel(bytes)
    idx = double(bitand(bitxor(crc, uint32(bytes(k))), uint32(255)));
    crc = bitxor(tbl(idx + 1), bitshift(crc, -8));
end
end

% -------------------------------------------------------------------------
function tbl = crc_table()
persistent T
if isempty(T)
    T = zeros(256, 1, 'uint32');
    for k = 0:255
        c = uint32(k);
        for b = 1:8
            if bitand(c, uint32(1)) ~= 0
                c = bitxor(uint32(3988292384), bitshift(c, -1)); % 0xEDB88320
            else
                c = bitshift(c, -1);
            end
        end
        T(k + 1) = c;
    end
end
tbl = T;
end
