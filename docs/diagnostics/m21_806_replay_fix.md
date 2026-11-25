# Mixed-stream recovery, 2026-11-24

Input: Desktop/20261124-170748-地平线8.06~1/CONN1-COM48-115200.dat,
515917 bytes. MATLAB nav_data.imu_raw contains 1394 samples with consecutive
counter increments and 10 ms header intervals.

Old parser decoded 1350 valid RAWIMU frames and reported 44 counter jumps plus
44 interval anomalies, CRC errors 0. Independent scanning of binary sync,
length and CRC found all 1394 valid RAWIMU frames. Missing frame offsets start
728, 7944, 15160; preceding binary diagnostics contain '#'/'$' bytes. The ASCII
branch consumed everything through the next LF, including complete valid frames.
This was a parser recovery defect, not a device-data continuity failure.

Fix: validate the ASCII candidate bytes and 8192-byte bound before consuming a
line or waiting for its terminator. Reject candidates containing binary/control
bytes and resume scanning. Preserve valid fragmented ASCII, CRC checking and
normal binary framing. Replay index and actual replay share the corrected parser.

Reproduced with failing synthetic tests for both markers and 1/7/16/65536-byte
read chunks, and unterminated printable prefixes before split binary sync.
After fix the actual replay index and Session both contain 1394 frames, with
CRC errors 0, counter jumps 0, duplicates 0 and interval anomalies 0.
All six physical-value arrays match existing MATLAB conversion exactly (max
error 0); temperature, counters and header intervals checked too. See
m21_806_matlab_comparison.json. Original input and MATLAB files unchanged.
