# M21 comparison, 2026-11-24

Input: CONN1-COM48-115200.dat, 256222 bytes, 709 RAWIMU frames, CRC errors 0.
User's desktop folder resolves to `20261124-145744-地平线8.05~1`.
Reference: existing `.mat_nav.mat` from main_of_dpx -> all_analysis_dpx ->
nav_imu_data; plotted reference `.dat_imu_data.png` inspected directly.

Causes and changes:

1. M21 incorrectly inherited M22 empirical acceleration scaling and .01 C/count.
   Use IMU_ID_M21 alias B240: gyro 1/65536 deg/s, acceleration
   9.80665/(20*65536) m/s² (increment conversion at 100 Hz), temperature /256+25.
2. Live chart used compensated gyro and adaptive gravity scaling. Retain separate
   fixed-profile physical histories; default chart to original physical quantities,
   with explicit compensated view. Attitude calibration and original CSV semantics
   remain; analysis archive already uses original physical quantities.
3. M21 nine-panel smoothing uses MATLAB centered movmean(500) by default; retain
   selectable trailing five-second average. Shrinking edges match MATLAB.
4. MATLAB time interval uses binary header timestamps, while integrity uses body
   measurement time. Cache both; label M21 original nine-panel interval as header
   time, preserve body intervals for integrity and quality checks.

Actual data comparison: all 709 six-axis values match MATLAB conversion exactly
(max error 0); temperature, counts, header intervals and SOW also checked.
Gyro means -0.0491364645, -0.1219757037, -0.2212362141 deg/s.
Accel means 0.1621212106, -0.5001227355, 9.7565172450 m/s².
Temperature 35.4140625–35.515625 C.
Centered 5s peak-to-peak values match all six MATLAB plot legends.
Machine-readable report: m21_matlab_comparison.json.
