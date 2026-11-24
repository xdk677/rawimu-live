# Allan timestamp-jitter false rejection

**Problem reproduced on user's original recording:** \\192.168.82.29\共享\地平线8.05-6\20261116-215337-德赛~1\CONN3-COM51-921600_filecopy2.dat.1261246samples,12612.4501386s, counter increments all1, CRC0, no invalid/nonpositive intervals. Timing range9.4220638..10.5810165ms,18712intervals outside old2%gate, none outside10%. Mean period10.0000001099ms (99.999998901Hz); maximum accumulated timing-phase residual after fittingmeanperiod0.488842ms (4.8884% of one period). Six physical means match user's screenshot exactly underM1633default scale. Thus reproduced, not inferred from a screenshot alone.

**Decision:** Separate modest timestamp jitter warnings from discontinuity errors. Default continuous-sample mode computes ordinary overlapping Allan on original ordered values using observed mean period, explicitly labeled an equally spaced approximation. Require complete counter continuity, finite samples, same configuration, no nonpositive intervals, no interval deviating>10% from median (absolute floor2µs), and accumulated phase departure from fittedmeanperiod<=25% of one period. These are explicit engineering acceptance gates, not claims of a measurement standard. Keep strictdevice-time mode2% for users needing it; both modes reject clock nonlinearity, gaps, resets, unknown counters and configuration changes. No interpolation, no dropping/rejoining fragments, no automatic stationary confirmation.

**Smoothing:**10s remains actual time-windowed means, resets only on actualdiscontinuities/severe timing changes/counter/config breaks, not harmless2%-10% jitter. Do not reset real smoothing at everysoftjitterevent.

**UI/report:** Expose two timingmodes, label approximation and warnings, separate jittercount/minmax/phase from errorcount, preserve units and usedperiod in report. Stationarycheckbox remainsrequired; user screenshot currently has it unchecked. When startinganalysis, progressbanner explicitly distinguishes previous result table from in-progress analysis.

- [x] Add failing balanced-jitter/strict-mode/drift/gap/counter and GUI mode regressions.
- [x] Implement qualityclassification, usedperiod, smoothingboundary and reportchanges; keep raw samples/times unmodified.
- [x] Verify correctedAllan on full realrecording, numericreference and whole suite; inspect actualdata GUI and requestindependentreview.
- [x] Update docs with thresholds, approximation and actualdiagnostic evidence, remove temporaryrawcache, finalverification.

## Final evidence

All118 tests passed. Original recording full1261246samples successfully produced six-axisAllan with period0.010000000109866741s, interval_anomalies0,counter_anomalies0,jitter_count18712,phase_max0.4888416345ms; ordinaryAllan equally-spaced approximation explicitly labeled. Sourcephysicalmeans exactlymatch screenshot. Realdata qualityreport/summary/AllanCSV and stats/Allan screenshots saved under docs/diagnostics, no full rawrecording copied into the repository. Actualnormal andsyntheticcompact UI inspected; newmodecontrol andmanualstaticconfirmation fit1100×760.

Independent review found no important issues with the quality-gate change and all16then-current numerical+6GUItests passed. Subsequent realdata verification revealed ceil(logspace) could round the last default m one sample above its supported limit; added a failing regression and clamped factors to maximum. Full118suite and final realrecordingAllan rerun passed. Lasttau1261.2400138568s retains10clusters asintended. No sensorvalues/timestamps edited, no imputation/drop/rejoining performed, originalNASfile unmodified, existingsoftware windows preserved. A temporary131MB decoded .npy cache used to avoid repeatednetworkparsing was removed afterverification. GUIprogress now labels previousresulttable while newanalysis runs, rather than displaying oldblockingmessage as the currentcomputation result.

References: https://www.mathworks.com/help/fusion/ref/allanvar.html ; https://www.nist.gov/publications/characterizing-frequency-stability-measurements-having-multiple-data-gaps . OrdinaryAllan assumes equally spaced sequence; gaps must not be hidden by concatenation.
