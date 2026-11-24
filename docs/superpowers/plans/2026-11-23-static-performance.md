# Static IMU performance analysis implementation plan

**Goal:** Add an independent static IMU analysis window for live/recorded/replayed data, manual segment selection, quality checks, 10s smoothing and overlapping Allan variance/deviation.

**User choice:** Manually select start/end and confirm stationary; report quality rather than silently auto-selecting a segment. Continue direct implementation in the current authorized checkout.

**Architecture:** `performance_data.py` keeps full-session physical samples in a lazy temporary binary archive, separate from65s display history and DAT/CSV. Rows preserve configured model scales before automatic calibration, actual device interval/counter/status and model revision. Archive time is valid-sample duration at configured rate, explicitly labeled. Offline DAT/ASCII/CSV parsing and range loading run in background. `imu_performance.py` implements pure NumPy statistics, full-window trailing smoothing and overlapping Allan calculations. `performance_window.py` owns Tk controls, result tables, six time plots and two log-log Allan charts; workers only post queue messages, never call Tk.

**Compatibility:** Python3.7/Tk, existing numpy/Pillow/OpenGL requirements; no new dependencies. Existing formats, calibration, CSV schema and record behavior preserved. Background archive failure must never stop sampling. Current source is whole session, not rolling screen history. File import independent of live data source.

**Numerical decisions:** Original mapped gyro/acc counts multiplied by frozen initial model factors, no dynamic bias removal or automatic accelerometer renormalization. Explicit model changes mark a revision boundary. Raw-file/CSV import uses selected IMU model; CSV raw count columns take precedence, physical-only columns retain a source note. Six axes reported in deg/s andm/s²; gyro means contain Earth rotation and accelerometer means contain gravity. Do not call these absolute sensor biases. 10s mean requires a full selected10s history, with initial incomplete samples omitted. Allan uses raw unsmoothed data, mean centering for numerical stability, adjacent overlapping m-sample means, denominatorN-2m+1; log-spaced m<=N/10 to retain at least10clusters. Both variance and its square root export. No forced bias-instability estimate from the minimum of a curve.

**Quality gate:** Nonfinite values, invalid/nonuniform device intervals, duplicate/skipped counters and model changes disable Allan for the selected range; no interpolation or concatenation across gaps. First selected interval excluded. Static confirmation required for Allan, with gyro/acc standard deviations and gravity/norm checks displayed as motion hints, not a static certification. CRC shown as source cumulative count. A short segment still provides statistics; Allan/presentation explains missing support.

## Tasks

- [x] Write failing tests for independent overlapping-Allan reference, invariance to constant offsets, white-noise behavior, full-window smoothing, quality gates, long archives/model scale independence, raw/CSV import, cancellation and exports.
- [x] Implement archive, range snapshot/file imports, Session integration and cleanup. Full data survives ring rollover; current recording files stay unchanged.
- [x] Implement background analysis, quality/statistics and display reduction preserving extrema. No Tk work in workers; bound long calculations by cooperative cancellation.
- [x] Implement independent window, menu/button, segment inputs, confirmation, tabs for stats/smoothing/Allan, CSV result export, source/session lifecycle and previews.
- [x] Run whole suite and sample selftests, inspect actual normal/compact screenshots, fresh independent review, document scope/formulas/units and update plan.

## Final rulings and evidence

- Analysis selection time accumulates measured positive device intervals up to10s; invalid/huge clock jumps advance one nominal period solely to keep selection usable, retaining original intervals for rejection. This supersedes the initial nominal-index-time design and avoids false10s smoothing when actual/configured frequencies differ.
- Missing/invalid status blocks Allan while still checking all known adjacent counters and known fault samples. Unknown or discontinuous partial status resets smoothing; all-missing status still permits time-only smoothing/statistics, with Allan unavailable.
- Source switches reset static confirmation. All missing/finite boundaries survive plot reduction; no interpolated bridge across later gaps in a bucket. Tab Map events redraw charts after Notebook visibility changes.
- All89 tests passed, including13 numerical/archive and5 actual Tk window tests. Binary2000frames/1999truth comparisons andASCII797frames selftests passed; py_compile andgit diff --check passed. Actual1320×920 and1100×760 screenshots for stats/smoothing/Allan inspected. Tests cover recording isolation, cancellation/session change, range, complete10s warmup, no smoothing over device gaps and independent Allan reference. No hardware connected; executable packaging not built.
- Independent reviewer found4 P2 issues above; each reproduced with a failing regression, fixed, and rechecked. Final review found no remaining important issues. Temporary archives explicitly close on source switch/exit and have cleanup finalizers for non-App consumers.

## Verification focus

Window close while loading/analyzing; new session during a job; file import while live acquisition continues; incomplete10s windows; repeated timestamps andweek rollback; use of precalibration scales; time range outside source; last long tau support; bad input CSV/NaNs/cancellation; axis/variance units in exported data.

## References

- https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html
- https://www.nist.gov/publications/handbook-frequency-stability-analysis
