# Network sources and GPS seconds-of-week implementation plan

> Execute inline using executing-plans; retain the user's existing uncommitted work.

**Goal:** Preserve serial acquisition, add TCP client/server and UDP reception,
and switch time-series tick labels and inspection between elapsed time and GPS SOW.

**Architecture:** A socket worker feeds the existing Session.feed byte path, so
parsing, recording and IMU/GPIMU analysis remain common. A focused time-axis mapper
labels the existing elapsed plotting coordinates using measured GPS week/SOW.
This preserves seek, smoothing, freeze navigation and synchronized cursor indices.
Do not reinterpret elapsed-only CSV timestamps as GPS. Allan remains on tau.

**Tech stack:** Python 3.7 stdlib sockets/threading, Tkinter Canvas, NumPy.

- [x] Add failing loopback tests for TCP client/server, UDP, exact DAT bytes,
  disconnect/stop while waiting and invalid endpoints; implement network_source.py.
  TCP server accepts one peer at a time and clears partial framing between peers.
  UDP accepts one sender per connection to avoid interleaving receiver streams.
- [x] Add source selector and address/port controls to rawimu_live.py, retain
  serial controls and connection actions, gate DAT for all live workers, show
  listening/receiving/source/error states, and expose optional CLI configuration.
- [x] Add failing tests for measured SOW histories, wrapping, missing GPS,
  clock jumps and cursor accuracy; implement time_axis.py and sample histories.
  Apply the mapper to main plots, integrity children, status lanes, six-axis,
  GPIMU and static smooth charts. Keep data/recording and Allan tau unchanged.
- [x] Add UI integration tests for live mode gating, shared axis switching,
  freeze zoom/inspection and no-GPS fallback; update README and inspect rendered UI.
- [x] Run full regression suite, compile and diff checks; request focused review.

Focused review found no important issues; reviewer independently ran all 18 new
source/axis/UI tests. The loopback preview exposed default Windows UDP receive
buffer loss (1250/2000 burst frames); added a failing regression and increased
SO_RCVBUF to 1 MiB, after which the test and preview received all 2000 frames.
Rendered UI: docs/images/ui_python_network_sow.png. No hardware was connected.
Final verification: 144 tests passed (55.492 s), touched Python files compiled,
and git diff --check passed (existing CRLF conversion warnings only).
