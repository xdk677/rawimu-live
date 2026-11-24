"""Textured, depth-buffered F-22 display embedded in Tk; no acquisition threads here."""
import ctypes
import math
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from OpenGL import GL, GLU

try:
    from pyopengltk import OpenGLFrame
except ImportError:
    # pyopengltk 0.0.4 does not implement a Darwin frame.  Keep the textured
    # renderer importable on macOS so MacF22View can reuse its GL drawing code.
    class OpenGLFrame(object):
        pass

from aircraft_mesh import load_f22

_FLIP_TOP_BOTTOM = getattr(Image, "Transpose", Image).FLIP_TOP_BOTTOM


class DetailedF22View(OpenGLFrame):
    def __init__(self, parent, cfg, on_error=None, **kwargs):
        self.cfg, self.on_error = cfg, on_error
        self.rotation = np.eye(3, dtype=np.float32)
        self.failed = False
        self.renderer = ''
        self._buffers, self._textures, self._batches = [], {}, []
        self._glyphs = {}
        self.last_render_ms = 0.
        OpenGLFrame.__init__(self, parent, **kwargs)
        self.bind('<Destroy>', self._cleanup, add='+')

    def _fail(self, error):
        if not self.failed:
            self.failed = True
            self.place_forget()
            if self.on_error:
                self.on_error(str(error))

    def tkMap(self, event):
        if self.failed:
            return
        try:
            OpenGLFrame.tkMap(self, event)
            self._display()
        except Exception as error:
            self._fail(error)

    def tkCreateContext(self):
        # One stencil bit per shadow pixel prevents overlapping flattened faces
        # from producing a noisy, repeatedly blended ground projection.
        from pyopengltk.win32 import pfd
        pfd.cStencilBits = 8
        OpenGLFrame.tkCreateContext(self)

    def tkResize(self, event):
        self.width, self.height = event.width, event.height
        if self.context_created:
            self._display()

    def _display(self):
        if not self.context_created or self.failed or not self.winfo_ismapped():
            return
        try:
            self.tkMakeCurrent()
            self.redraw()
            self.tkSwapBuffers()
        except Exception as error:
            self._fail(error)

    def initgl(self):
        if self._batches:
            return
        version = GL.glGetString(GL.GL_VERSION)
        if not version:
            raise RuntimeError('OpenGL context unavailable')
        self.renderer = GL.glGetString(GL.GL_RENDERER).decode('utf-8', 'replace')
        self.width, self.height = self.winfo_width(), self.winfo_height()
        model = load_f22()
        self.triangles = model['triangles']
        for batch in sorted(model['batches'], key=lambda item: item['alpha'] < 1):
            buffer = int(GL.glGenBuffers(1))
            self._buffers.append(buffer)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, buffer)
            array = np.ascontiguousarray(batch['array'], dtype=np.float32)
            GL.glBufferData(GL.GL_ARRAY_BUFFER, array.nbytes, array, GL.GL_STATIC_DRAW)
            texture = 0
            if batch['texture']:
                path = batch['texture']
                if path not in self._textures:
                    with Image.open(path) as image:
                        pixels = image.convert('RGBA').transpose(_FLIP_TOP_BOTTOM)
                        handle = int(GL.glGenTextures(1))
                        self._textures[path] = handle
                        GL.glBindTexture(GL.GL_TEXTURE_2D, handle)
                        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR_MIPMAP_LINEAR)
                        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
                        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGBA, pixels.width, pixels.height,
                                        0, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, pixels.tobytes())
                        GL.glGenerateMipmap(GL.GL_TEXTURE_2D)
                texture = self._textures[path]
            self._batches.append(dict(batch, buffer=buffer, texture_id=texture, count=len(array)))
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDepthFunc(GL.GL_LESS)
        GL.glEnable(GL.GL_NORMALIZE)
        GL.glShadeModel(GL.GL_SMOOTH)
        GL.glLightModeli(GL.GL_LIGHT_MODEL_TWO_SIDE, GL.GL_TRUE)
        GL.glLightModelfv(GL.GL_LIGHT_MODEL_AMBIENT, (.24, .25, .28, 1.))
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
        GL.glClearColor(.925, .945, .968, 1.)
        self._has_stencil = GL.glGetIntegerv(GL.GL_STENCIL_BITS) > 0

    def set_attitude(self, rotation):
        self.rotation = np.asarray(rotation, dtype=np.float32).reshape(3, 3).copy()
        self._display()

    def _matrix(self):
        value = np.eye(4, dtype=np.float32)
        value[:3, :3] = self.rotation
        return value.T

    def _geometry(self, shaded=True):
        GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
        if shaded:
            GL.glEnableClientState(GL.GL_NORMAL_ARRAY)
            GL.glEnableClientState(GL.GL_TEXTURE_COORD_ARRAY)
        for batch in self._batches:
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, batch['buffer'])
            GL.glVertexPointer(3, GL.GL_FLOAT, 32, ctypes.c_void_p(0))
            if shaded:
                GL.glNormalPointer(GL.GL_FLOAT, 32, ctypes.c_void_p(12))
                GL.glTexCoordPointer(2, GL.GL_FLOAT, 32, ctypes.c_void_p(24))
                if batch['texture_id']:
                    GL.glEnable(GL.GL_TEXTURE_2D)
                    GL.glBindTexture(GL.GL_TEXTURE_2D, batch['texture_id'])
                else:
                    GL.glDisable(GL.GL_TEXTURE_2D)
                color = tuple(batch['color']) + (batch['alpha'],)
                GL.glMaterialfv(GL.GL_FRONT_AND_BACK, GL.GL_AMBIENT_AND_DIFFUSE, color)
                GL.glMaterialfv(GL.GL_FRONT_AND_BACK, GL.GL_SPECULAR,
                                (.55, .50, .36, 1.) if batch['alpha'] < 1 else (.20, .21, .22, 1.))
                GL.glMaterialf(GL.GL_FRONT_AND_BACK, GL.GL_SHININESS, batch['shininess'])
                GL.glDepthMask(batch['alpha'] >= 1)
            GL.glDrawArrays(GL.GL_TRIANGLES, 0, batch['count'])
        GL.glDepthMask(True)
        GL.glDisableClientState(GL.GL_VERTEX_ARRAY)
        if shaded:
            GL.glDisableClientState(GL.GL_NORMAL_ARRAY)
            GL.glDisableClientState(GL.GL_TEXTURE_COORD_ARRAY)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        GL.glDisable(GL.GL_TEXTURE_2D)

    def _label(self, text, position, color):
        key = text, color
        if key not in self._glyphs:
            image = Image.new('RGBA', (32, 28), (0, 0, 0, 0))
            try:
                font = ImageFont.truetype('consolab.ttf', 18)
            except OSError:
                font = ImageFont.load_default()
            ImageDraw.Draw(image).text((2, 0), text, font=font, fill=tuple(color) + (255,))
            self._glyphs[key] = image.transpose(_FLIP_TOP_BOTTOM).tobytes()
        GL.glRasterPos3f(*position)
        GL.glDrawPixels(32, 28, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, self._glyphs[key])

    def redraw(self):
        start = time.perf_counter()
        width, height = max(1, self.winfo_width()), max(1, self.winfo_height())
        GL.glViewport(0, 0, width, height)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GLU.gluPerspective(35., width / float(height), .1, 40.)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        az, el = math.radians(self.cfg['view_az']), math.radians(self.cfg['view_el'])
        distance = 8.5
        GLU.gluLookAt(distance * math.cos(el) * math.cos(az), distance * math.cos(el) * math.sin(az),
                      distance * math.sin(el), 0., 0., .30, 0., 0., -1.)
        GL.glDisable(GL.GL_LIGHTING)
        GL.glDisable(GL.GL_TEXTURE_2D)
        GL.glDisable(GL.GL_BLEND)
        GL.glColor3f(.86, .90, .86)
        GL.glBegin(GL.GL_TRIANGLE_FAN)
        GL.glVertex3f(0., 0., .95)
        for index in range(97):
            a = index * 2 * math.pi / 96
            GL.glVertex3f(2.20 * math.cos(a), 2.20 * math.sin(a), .95)
        GL.glEnd()
        GL.glColor3f(.58, .66, .61)
        GL.glLineWidth(1.)
        for radius in (2.20, 1.75, .88):
            GL.glBegin(GL.GL_LINE_LOOP)
            for index in range(96):
                a = index * 2 * math.pi / 96
                GL.glVertex3f(radius * math.cos(a), radius * math.sin(a), .945)
            GL.glEnd()
        GL.glEnable(GL.GL_BLEND)
        if self._has_stencil:
            GL.glClearStencil(0)
            GL.glClear(GL.GL_STENCIL_BUFFER_BIT)
            GL.glEnable(GL.GL_STENCIL_TEST)
            GL.glStencilFunc(GL.GL_NOTEQUAL, 1, 0xff)
            GL.glStencilOp(GL.GL_KEEP, GL.GL_KEEP, GL.GL_REPLACE)
            GL.glDisable(GL.GL_DEPTH_TEST)
            GL.glDepthMask(False)
            GL.glColor4f(.18, .25, .22, .18)
            shadow = np.array(((1, 0, -.35, .3325), (0, 1, -.25, .2375),
                               (0, 0, 0, .94), (0, 0, 0, 1)), dtype=np.float32)
            GL.glPushMatrix()
            GL.glMultMatrixf(shadow.T)
            GL.glMultMatrixf(self._matrix())
            self._geometry(shaded=False)
            GL.glPopMatrix()
            GL.glDisable(GL.GL_STENCIL_TEST)
            GL.glDepthMask(True)
            GL.glEnable(GL.GL_DEPTH_TEST)
        # The compass is a reference backdrop, never a clipping plane at steep attitudes.
        GL.glClear(GL.GL_DEPTH_BUFFER_BIT)
        GL.glEnable(GL.GL_LIGHTING)
        for light, position, strength in ((GL.GL_LIGHT0, (-3., -4., -6., 0.), .85),
                                           (GL.GL_LIGHT1, (4., 2., -3., 0.), .35)):
            GL.glEnable(light)
            GL.glLightfv(light, GL.GL_POSITION, position)
            GL.glLightfv(light, GL.GL_DIFFUSE, (strength, strength, strength, 1.))
            GL.glLightfv(light, GL.GL_SPECULAR, (.65, .65, .65, 1.))
        GL.glPushMatrix()
        GL.glMultMatrixf(self._matrix())
        self._geometry()
        GL.glPopMatrix()
        GL.glDisable(GL.GL_LIGHTING)
        GL.glDisable(GL.GL_DEPTH_TEST)
        for text, x, y in (('N', 2.0, 0.), ('E', 0., 2.0), ('S', -2.0, 0.), ('W', 0., -2.0)):
            self._label(text, (x, y, .93), (67, 94, 79))
        for k, color in enumerate(((196, 54, 54), (33, 151, 89), (48, 101, 197))):
            endpoint = self.rotation[:, k] * 1.65
            GL.glColor3ub(*color)
            GL.glBegin(GL.GL_LINES)
            GL.glVertex3f(0, 0, 0)
            GL.glVertex3f(*endpoint)
            GL.glEnd()
            self._label('XYZ'[k], endpoint, color)
        GL.glColor3f(.75, .15, .15)
        GL.glBegin(GL.GL_LINES)
        GL.glVertex3f(0., 0., .93)
        GL.glVertex3f(float(self.rotation[0, 0]) * 1.9, float(self.rotation[1, 0]) * 1.9, .93)
        GL.glEnd()
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glFlush()
        self.last_render_ms = (time.perf_counter() - start) * 1000.

    def capture(self):
        if not self.context_created or self.failed:
            raise RuntimeError('Detailed model has no active OpenGL context')
        self.tkMakeCurrent()
        self.redraw()
        width, height = self.winfo_width(), self.winfo_height()
        GL.glReadBuffer(GL.GL_BACK)
        GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1)
        data = GL.glReadPixels(0, 0, width, height, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
        return Image.frombytes('RGB', (width, height), data).transpose(_FLIP_TOP_BOTTOM)

    def _cleanup(self, event):
        if event.widget is not self:
            return
        context = getattr(self, '_OpenGLFrame__context', None)
        window = getattr(self, '_OpenGLFrame__window', None)
        if context:
            from OpenGL.WGL import wglMakeCurrent, wglDeleteContext
            user32 = ctypes.windll.user32
            user32.ReleaseDC.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
            # Each teardown operation must run even if a driver rejects an earlier one.
            operations = [lambda: wglMakeCurrent(window, context),
                          lambda: GL.glDeleteBuffers(len(self._buffers), self._buffers) if self._buffers else None,
                          lambda: GL.glDeleteTextures(list(self._textures.values())) if self._textures else None,
                          lambda: wglMakeCurrent(None, None),
                          lambda: wglDeleteContext(context),
                          lambda: user32.ReleaseDC(self._wid, window)]
            try:
                for operation in operations:
                    try:
                        operation()
                    except Exception:
                        pass  # Context deletion also releases any remaining GPU objects.
            finally:
                self._OpenGLFrame__context = None
                self._OpenGLFrame__window = None
                self.context_created = False
