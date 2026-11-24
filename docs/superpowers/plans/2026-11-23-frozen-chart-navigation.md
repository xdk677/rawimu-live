# Frozen chart navigation

Implement independently for each chart: wheel zoom, Shift wheel X, Ctrl wheel Y,
left drag pan, double click reset. Gate live charts on shared display freeze;
offline GPIMU and computed performance results are static and navigable directly.
Digital fault lanes keep their categorical Y axis. Preserve cached samples,
cursor mapping, gaps, peak decimation, and clip traces to the plot rectangle.
Reset navigation when resuming live or changing the displayed time range.
Use a shared weak-owner navigation controller for linear and logarithmic charts.

Validation: real Tk gesture tests, application integration tests, full suite,
compile and whitespace checks, and focused review.

Implemented all chart families. Seven navigation tests cover frozen gating,
bounded zoom, horizontal/vertical modifiers, panning, cursor mapping, clipping,
digital lanes, integrity child independence, Allan logarithmic navigation, static
GP files, and unfreezing without new GP data. Focused review found the latter
cached-axis issue; reproduced with a failing test and fixed by cached redraw.
The full suite has 125 tests. A frozen zoom screenshot was rendered and inspected
at docs/images/ui_python_frozen_zoom.png.
