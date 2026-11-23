# Command view implementation plan

Implement inline on the existing workbench; retain all earlier uncommitted work.

Add a command tab with manual text/HEX send, explicit text line endings, history,
TX completion/failure logs and bounded response/HEX previews. Reuse the active
serial/TCP/UDP connection; never open a second serial handle. UDP can reply to its
pinned sender or an explicitly entered IPv4 target. Replay cannot transmit.
No commands or automatic repetitions are issued by opening/changing the view.

- [x] Test byte encoding and a bounded command queue with connection tokens;
  commands queued for a closed connection must never reach its replacement.
- [x] Implement command_transport.py with worker-side writes and bounded RX
  observation. Add serial/TCP/UDP integration tests and hook existing workers.
- [x] Implement command_view.py with weak App ownership and UI polling through
  the existing refresh loop. Test validation, send action, mode gating and logs.
- [x] Update documentation, render the view, run full tests and focused review.

Actual device commands are never sent during implementation; use mocks/loopback.

Focused review identified mixed binary/text response loss, UDP target reply
isolation (including a reply before the first telemetry packet), and slow writes
blocking acquisition. Added failing regressions for each and fixed them with a
framed response observer, per-source RX buffers, explicit reply routing, and a
connection-owned independent TX thread. Serial writes use 256-byte chunks; RX
never waits for queued writes. Review confirmed the final UDP boundary fix, with
no new important findings. All 18 targeted command tests pass.
Final verification: all 162 tests passed in 49.087 s; touched Python files compile
and git diff --check passes (existing CRLF conversion warnings only). Inspected
docs/images/ui_python_command_view.png from the local UDP echo preview.
