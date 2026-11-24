# Replay timeline implementation plan

**Goal:** Show replay current/total elapsed time and progress, with pause/resume and drag/keyboard seeking in the desktop viewer.

**Architecture:** Extend FrameParser with the absolute end offset of its current valid frame; preserve all existing decoding behavior. New `replay_timeline.py` scans an immutable recording in a worker and records accepted IMU end offsets and normalized device elapsed times in compact arrays. Playback feeds original source bytes through the existing Session parser, pacing before every IMU frame. The index scans in background, reports byte progress and can be stopped. Clock regressions/nonfinite or>10s jumps use nominal periods only for playback navigation, retaining actual device intervals for integrity/performance checks.

**Seek:** Stop old worker asynchronously, create a new session after it exits, reuse a validated index, and fast-forward original bytes from the beginning to the selected timestamp. Thus attitude/state/history/calibration reflect the full prefix, not a fresh orientation at the target. Preserve paused/running state; dragging after EOF lands paused. Source-switch lifecycle clears existing live plots and cancels current-session analysis through existing hooks. The full prefix remains available to static analysis.

**UI:** Independent replay row below existing tools, only shown for replay, pause/resume button, slider, elapsed/total duration, scanning/seeking status. Preview while dragging; commit once on release or keyboard selection. Keep display freeze separate from replay pause. Source/stop/exit cancels any pending seek. CSV recording disables seeking (pause remains possible), preventing replay-prefix duplicates in saved CSV. No hardware connection or termination of user windows.

**Compatibility:** Python3.7/Tkinter, no new dependencies; serial parsing, CSV andDAT schemas unchanged. Compact arrays use16bytes/sample. Existing speed0 and interruptible stop contracts preserved. Index signature checks path/size/mtime; changing files must be loaded again.

- [x] Add failing offset/index/time normalization, pause/stop/seek consistency, original replay timing and GUI timeline/recording guard tests.
- [x] Implement scanner, per-frame pacing/control and asynchronous seek lifecycle; update Session replay display timestamps without altering raw integrity time.
- [x] Implement timeline row, preview/pause/seek controls and actual screenshots; ensure compact layout and source cleanup.
- [x] Full regression/selftest, independent review, docs and final verification.

## Final verification and rulings

All99 tests passed, including10 timeline regressions and the original19 backend tests. Binary2000frames/1999truth comparisons andASCII797frames selftests passed; py_compile andgit diff --check passed. Actual1600×960 and1100×720 screenshots inspected. No hardware connected; existing user windows preserved; executable package not built.

Independent review found malformed recognizedASCII consuming callback-based indices, mutation during playback being ignored, and CSV starting during seek reconstruction. Each reproduced with a failing regression, then fixed. Playback now reads to the accepted frame-end offsets rather than mapping callback ordinals, retains original malformed/unknown bytes for parser statistics, validates IMU counts, and checks file descriptor signatures before each frame plus path signature atEOF. CSV start checks state before/after file selection and Session identity. Final review found no remaining important issues.

Sequential-window testing exposed an App→ReplayTimeline→App Python cycle causing Tk variable finalizers to run during worker-thread garbage collection while the UI waited on Session.lock. Faulthandler identified both stacks; the component now holds a weak App reference. Fresh10 timeline tests and full99 suite complete cleanly without Tk thread/finalizer warnings. Seek GUI assertions explicitly refresh the final observed snapshot to avoid racing the UI's next scheduled tick.
