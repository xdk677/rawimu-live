# GPIMU analysis view implementation plan

**Goal:** Add GPIMU view to RAWIMU Workbench following live/replay, with independent file loading, six axes, samples/sentence, temperature, device interval, and timestamp-aligned RAWIMU comparison. User explicitly selected both live/replay and independent files.

**References:** `../M22p_gpimu.m`, `../GNSS_NAV/ins/src/io/read_gpimu.m`, `../GNSS_NAV/ins/src/conversion/load_gpimu2imudata.m`, `../GNSS_NAV/ins/src/conversion/print_gpimu.m`, `../GNSS_NAV/ins/src/processing/load_imu_model.m`, and IMU_ID_M21/M22 aliases. Current script active modelM1633; do not change MATLAB files. Referenced old desktop recording has moved/is absent, so use reproducible protocol fixtures and any available local actual records only if found.

**Protocol:** Supported new $GPIMU comma layout contains IMUtime,count,GXYZhex32,AXYZhex32,week,sow,5reserved,type/flag,temperaturehex32,optionalextension then2hex XOR checksum. Signed32 decoding (never guess24bit), separate2hex XOR validation, report checksum/format failures. GPcount is samples aggregated per sentence, not an incrementing modulo256 frame counter. No synthetic IMU_Status or fault names from type/flag fields.

**Models:** Independently select GPmodel. M1633 gyro1/(100*65536)deg/s, accel1/(200*65536)m/s²,T0.01x; M21 aliasB240 gyro same, accel9.80665/(2000*65536),T/256+25; M22 aliasB240_E7111 gyroXYsame,Z−1/(16896*256),accelM21,T/128. Axis identity except M22 gyroZ reversal, as MATLAB. GP conversion never uses RAWinvert_y/auto calibration. RAWcomparison uses original mapped counts and frozen RAWmodel scales before dynamicbias/renormalization.

**Architecture:** `gpimu_data.py` strict decoder, profiles, bounded live raw store, full-file background loader and nearestGPS-time one-to-one comparison. `gpimu_view.py` third Notebook page:9compactplots(row1count/T/dt,row2GXYZ,row3AXYZ), comparison6plots/statistics, originalfields, independentfile position and visible-windowCSV export. No new dependencies. CaptureGP auxiliary records in FrameParser/Session without changing RAWn_imu or attitude. Current frozen snapshot includes GPandRAWcomparison. GP-only replay uses fallbackGPindex and independentvalidGPcount; mixed replay keeps RAW timeline and never doubles integration. Explicitly show clock fallback toIMUtime whenGPweek unavailable; compare only valid GPS epochs, no interpolation/forcedfitting. MixedfilesGP data follows rawfilebyteprefix; dedicatedfilemode can inspect the wholeGP history independent ofRAWtimeline.

**Lifecycle:** Main10/30/60s window and display freeze shared for current source; newSession resets only following-current source, standalone file retained. UIworkers communicate viaqueue and cancellation; noTk in background. Store rawcounts so switching GPprofile reinterprets all retainedGPsamples consistently. Cancel loaders onviewdestruction, weakAppreference avoids TkGC worker cycles. Rawsource/CSV/DAT unchanged; GPvisible-window export separate.

- [x] Add failing protocol/sign/XOR/model/count semantics, no double integration, nearest-timecomparison, GP-only/mixedreplay and malformedGP tests.
- [x] Implement core parser/data/capture/replay fallback; preserve all99prior regressions.
- [x] Implement view, background file source, cursor/freeze/reset/export and ordinary/compact actual screenshots.
- [x] Full checks, independent review, documentation and final verification.

## Final evidence and rulings

All113 tests passed, including10 GPprotocol/data and4 GPUI regressions plus all99prior checks. Binary2000frames/1999truth, ASCII797frames and mixedGPfixture1200RAW+1200GP selftests passed; py_compile andgit diff --check passed. Mixedfixture replay retains RAWprimarytime and all1200pairs match exactly under the independently applied model factors. PureGP replay, noGPSfallback, aggregatecount semantics, freeze, independentfile/session isolation, export and cursorreset covered.

Independent review identified M22 temperature low16signed reinterpretation (the MATLAB combined-model step before /128) and ambiguous duplicateGP epochs; failing tests reproduced both, fixes rechecked, no important findings remain. Originalraw32temperature remains available in field view. Matching uses batchedsearchsorted and excludes duplicateepochs onboth sides. ActualScreenshot inspection found firsthiddenmonitor mapping could remain blank untilnewdata; explicit Map renders cacheddata, with a failing→passing GUIregression. HiddenGPpages skipcomparison/renderwork. Fileposition changes clear an out-of-range cursor; rawIMU/GPS time fields retain9decimalplaces.

Normal1600×960 andcompact1100×720 monitor/comparison/field screenshots generated and inspected. Fixture is syntheticGPderived from bundledRAW trajectory, not a hardwarecapture. No hardware connected, no user windows stopped, MATLABscript unchanged andno executablepackage built. Independentfileloader retains wholeloadedfile; live data remains bounded by displayhistory. Main RAW CSV schema unchanged, GPview exports visibleGPwindow separately.
