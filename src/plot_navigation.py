"""Canvas navigation in data coordinates; owners are weak to avoid Tk GC cycles."""
import weakref


class PlotNavigation(object):
    def __init__(self, canvas, redraw, enabled=True, horizontal_only=False):
        self.canvas = canvas
        self.redraw = weakref.WeakMethod(redraw)
        self.enabled = enabled
        self.owner = None
        self.base = self.view = self.rect = self.drag = None
        self.horizontal_only = horizontal_only
        canvas.bind('<MouseWheel>', self.wheel, add='+')
        canvas.bind('<ButtonPress-1>', self.press, add='+')
        canvas.bind('<B1-Motion>', self.motion, add='+')
        canvas.bind('<ButtonRelease-1>', self.release, add='+')
        canvas.bind('<Double-Button-1>', self.double_click, add='+')

    def allow(self):
        owner = self.owner() if self.owner else None
        return (bool(owner and (getattr(owner, 'frozen', False) or
                               getattr(owner, 'file_data', None) is not None))
                if self.owner else self.enabled)

    def configure(self, base, rect):
        if self.base != base or not self.allow():
            self.view = self.drag = None
        self.base, self.rect = base, rect
        return self.view or base

    def reset(self):
        self.view = self.drag = None
        self.draw()

    def draw(self):
        callback = self.redraw()
        if callback:
            callback()

    def inside(self, event):
        if not self.allow() or not self.base or not self.rect:
            return False
        left, top, width, height = self.rect
        return left <= event.x <= left+width and top <= event.y <= top+height

    @staticmethod
    def bounded(lo, hi, minimum, maximum):
        span = min(maximum-minimum, max((maximum-minimum)*1e-5, hi-lo))
        lo = max(minimum, min(maximum-span, lo))
        return lo, lo+span

    def wheel(self, event):
        if not self.inside(event) or not event.delta:
            return
        left, top, width, height = self.rect
        x0, x1, y0, y1 = self.view or self.base
        factor = 1.25 ** (-max(-4., min(4., event.delta/120.)))
        def zoom(lo, hi, fraction, minimum, maximum):
            anchor = lo + fraction*(hi-lo)
            return self.bounded(anchor+(lo-anchor)*factor, anchor+(hi-anchor)*factor, minimum, maximum)
        if not event.state & 4:  # Ctrl: Y only.
            x0, x1 = zoom(x0, x1, (event.x-left)/width, *self.base[:2])
        if not self.horizontal_only and not event.state & 1:  # Shift: X only.
            y0, y1 = zoom(y0, y1, 1-(event.y-top)/height, *self.base[2:])
        self.view = x0, x1, y0, y1
        self.draw()
        return 'break'

    def press(self, event):
        if self.inside(event):
            self.drag = event.x, event.y, self.view or self.base

    def motion(self, event):
        if self.drag and self.allow():
            x, y, (x0, x1, y0, y1) = self.drag
            dx = (event.x-x)*(x1-x0)/self.rect[2]
            dy = (event.y-y)*(y1-y0)/self.rect[3]
            xx = self.bounded(x0-dx, x1-dx, *self.base[:2])
            yy = (y0, y1) if self.horizontal_only else self.bounded(y0+dy, y1+dy, *self.base[2:])
            self.view = xx + yy
            self.draw()
            return 'break'

    def release(self, event):
        self.drag = None

    def double_click(self, event):
        if self.inside(event):
            self.reset()
            return 'break'


def clip_polyline(points, xmin, xmax, ymin, ymax):
    """Liang–Barsky clipping, preserving separate excursions and gaps."""
    if not points:
        return []
    # Auto-scaled plots normally fit completely inside the viewport.  Avoid
    # rebuilding every point pair through the clipping loop in that case.
    if all(xmin <= x <= xmax and ymin <= y <= ymax for x, y in points):
        return [points]
    result, current = [], []
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        low, high = 0., 1.
        for p, q in ((-dx, a[0]-xmin), (dx, xmax-a[0]),
                     (-dy, a[1]-ymin), (dy, ymax-a[1])):
            if p == 0:
                if q < 0:
                    low, high = 1., 0.
                    break
            elif p < 0:
                low = max(low, q/p)
            else:
                high = min(high, q/p)
        if low <= high:
            start = (a[0]+low*dx, a[1]+low*dy)
            end = (a[0]+high*dx, a[1]+high*dy)
            if current and current[-1] != start:
                result.append(current)
                current = []
            if not current:
                current.append(start)
            current.append(end)
        elif current:
            result.append(current)
            current = []
    if current:
        result.append(current)
    return result
