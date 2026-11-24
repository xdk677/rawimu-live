"""Native macOS F-22 renderer using an offscreen CGL OpenGL context."""
import ctypes
import ctypes.util
import tkinter as tk

import numpy as np
from PIL import Image, ImageTk
from OpenGL import GL

from aircraft_gl import DetailedF22View


_CGL_VOID = ctypes.c_void_p
_CGL_ATTR_ACCELERATED = 73
_CGL_ATTR_COLOR_SIZE = 8
_CGL_ATTR_DEPTH_SIZE = 12
_CGL_ATTR_STENCIL_SIZE = 13
_FRAMEBUFFER_COMPLETE = 0x8CD5
_FLIP_TOP_BOTTOM = getattr(Image, "Transpose", Image).FLIP_TOP_BOTTOM


class _CGLContext(object):
    """Small ctypes bridge for the deprecated but still supported macOS CGL API."""

    def __init__(self):
        path = ctypes.util.find_library("OpenGL")
        if not path:
            raise RuntimeError("macOS OpenGL.framework is unavailable")
        self.api = ctypes.CDLL(path)
        self._bind_api()
        attrs = (ctypes.c_uint32 * 8)(
            _CGL_ATTR_ACCELERATED,
            _CGL_ATTR_COLOR_SIZE, 24,
            _CGL_ATTR_DEPTH_SIZE, 24,
            _CGL_ATTR_STENCIL_SIZE, 8,
            0,
        )
        pixel = _CGL_VOID()
        count = ctypes.c_int()
        error = self.api.CGLChoosePixelFormat(
            attrs, ctypes.byref(pixel), ctypes.byref(count))
        if error or not pixel.value:
            raise RuntimeError("CGLChoosePixelFormat failed (%s)" % error)
        context = _CGL_VOID()
        error = self.api.CGLCreateContext(pixel, None, ctypes.byref(context))
        self.api.CGLDestroyPixelFormat(pixel)
        if error or not context.value:
            raise RuntimeError("CGLCreateContext failed (%s)" % error)
        self.context = context
        self.closed = False

    def _bind_api(self):
        api = self.api
        api.CGLChoosePixelFormat.argtypes = [
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(_CGL_VOID),
            ctypes.POINTER(ctypes.c_int),
        ]
        api.CGLChoosePixelFormat.restype = ctypes.c_int
        api.CGLCreateContext.argtypes = [
            _CGL_VOID, _CGL_VOID, ctypes.POINTER(_CGL_VOID)]
        api.CGLCreateContext.restype = ctypes.c_int
        api.CGLDestroyPixelFormat.argtypes = [_CGL_VOID]
        api.CGLDestroyPixelFormat.restype = ctypes.c_int
        api.CGLDestroyContext.argtypes = [_CGL_VOID]
        api.CGLDestroyContext.restype = ctypes.c_int
        api.CGLSetCurrentContext.argtypes = [_CGL_VOID]
        api.CGLSetCurrentContext.restype = ctypes.c_int
        api.CGLGetCurrentContext.argtypes = []
        api.CGLGetCurrentContext.restype = ctypes.c_void_p

    def activate(self):
        previous = self.api.CGLGetCurrentContext()
        error = self.api.CGLSetCurrentContext(self.context)
        if error:
            raise RuntimeError("CGLSetCurrentContext failed (%s)" % error)
        return previous

    def restore(self, previous):
        self.api.CGLSetCurrentContext(previous)

    def close(self):
        if self.closed:
            return
        self.closed = True
        current = self.api.CGLGetCurrentContext()
        if current == self.context.value:
            self.api.CGLSetCurrentContext(None)
        self.api.CGLDestroyContext(self.context)
        self.context = _CGL_VOID()


class MacF22View(tk.Canvas):
    """Textured F-22 rendered by CGL into a Tk PhotoImage.

    Reuses the established mesh, material, lighting and compass drawing code
    from DetailedF22View while replacing its unsupported Darwin window frame
    with an explicit framebuffer object.
    """

    initgl = DetailedF22View.initgl
    redraw = DetailedF22View.redraw
    _geometry = DetailedF22View._geometry
    _label = DetailedF22View._label
    _matrix = DetailedF22View._matrix

    def __init__(self, parent, cfg, on_error=None, **kwargs):
        kwargs.setdefault("highlightthickness", 0)
        kwargs.setdefault("borderwidth", 0)
        kwargs.setdefault("background", "#edf1f8")
        tk.Canvas.__init__(self, parent, **kwargs)
        self.cfg, self.on_error = cfg, on_error
        self.rotation = np.eye(3, dtype=np.float32)
        self.failed = False
        self.context_created = False
        self.renderer = ""
        self._buffers, self._textures, self._batches = [], {}, []
        self._glyphs = {}
        self.last_render_ms = 0.
        self._framebuffer = 0
        self._color_target = 0
        self._depth_target = 0
        self._target_size = None
        self._photo = None
        self._photo_size = None
        self._image_item = self.create_image(0, 0, anchor="nw")
        self._captured = None
        self._readback_buffer = None
        self._readback_size = None
        self._last_rendered_rotation = None
        self._resize_job = None
        self._closed = False
        try:
            self._cgl = _CGLContext()
            with self._current_context():
                self.initgl()
                self.renderer = GL.glGetString(
                    GL.GL_RENDERER).decode("utf-8", "replace")
            self.context_created = True
        except Exception as error:
            self._cgl = None
            self._fail(error)
            return
        self.bind("<Configure>", self._on_configure, add="+")
        self.bind("<Destroy>", self._cleanup, add="+")
        self.after(20, self._display)

    def _current_context(self):
        class _CurrentContext(object):
            def __init__(context_self):
                context_self.previous = None

            def __enter__(context_self):
                context_self.previous = self._cgl.activate()

            def __exit__(context_self, exc_type, exc_value, traceback):
                self._cgl.restore(context_self.previous)

        return _CurrentContext()

    def _fail(self, error):
        if self.failed:
            return
        self.failed = True
        self.place_forget()
        if self.on_error:
            self.on_error(str(error))

    def _on_configure(self, event):
        if event.widget is not self or event.width < 4 or event.height < 4:
            return
        self._target_size = None
        if self._resize_job is not None:
            try:
                self.after_cancel(self._resize_job)
            except tk.TclError:
                pass
        self._resize_job = self.after_idle(self._display)

    def _ensure_framebuffer(self, width, height):
        size = max(4, int(width)), max(4, int(height))
        if self._target_size == size:
            return
        if self._framebuffer:
            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
            GL.glDeleteFramebuffers(1, [self._framebuffer])
            GL.glDeleteTextures([self._color_target])
            GL.glDeleteRenderbuffers(1, [self._depth_target])
        self._framebuffer = int(GL.glGenFramebuffers(1))
        self._color_target = int(GL.glGenTextures(1))
        self._depth_target = int(GL.glGenRenderbuffers(1))
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self._framebuffer)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self._color_target)
        GL.glTexParameteri(
            GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(
            GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glTexImage2D(
            GL.GL_TEXTURE_2D, 0, GL.GL_RGBA8, size[0], size[1], 0,
            GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, None)
        GL.glFramebufferTexture2D(
            GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0,
            GL.GL_TEXTURE_2D, self._color_target, 0)
        GL.glBindRenderbuffer(GL.GL_RENDERBUFFER, self._depth_target)
        GL.glRenderbufferStorage(
            GL.GL_RENDERBUFFER, GL.GL_DEPTH24_STENCIL8, size[0], size[1])
        GL.glFramebufferRenderbuffer(
            GL.GL_FRAMEBUFFER, GL.GL_DEPTH_STENCIL_ATTACHMENT,
            GL.GL_RENDERBUFFER, self._depth_target)
        status = GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER)
        if status != _FRAMEBUFFER_COMPLETE:
            raise RuntimeError(
                "OpenGL framebuffer incomplete (0x%04X)" % status)
        self._target_size = size
        self._has_stencil = GL.glGetIntegerv(GL.GL_STENCIL_BITS) > 0

    def _read_image(self, width, height):
        size = width, height
        if self._readback_size != size:
            self._readback_buffer = ctypes.create_string_buffer(
                width * height * 4)
            self._readback_size = size
        GL.glReadBuffer(GL.GL_COLOR_ATTACHMENT0)
        GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1)
        GL.glReadPixels(
            0, 0, width, height, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE,
            self._readback_buffer)
        image = Image.frombuffer(
            "RGBA", size, self._readback_buffer, "raw", "RGBA", 0, 1)
        return image.transpose(_FLIP_TOP_BOTTOM)

    def set_attitude(self, rotation):
        value = np.asarray(rotation, dtype=np.float32).reshape(3, 3)
        if (self._last_rendered_rotation is not None and
                np.max(np.abs(value - self._last_rendered_rotation)) < 1e-6):
            return
        self.rotation = value.copy()
        self._display()

    def _display(self):
        self._resize_job = None
        if self.failed or not self.context_created or self._closed:
            return
        width, height = self.winfo_width(), self.winfo_height()
        if width < 4 or height < 4:
            return
        try:
            with self._current_context():
                self._ensure_framebuffer(width, height)
                self.redraw()
                image = self._read_image(width, height)
            self._captured = image
            if self._photo is None or self._photo_size != image.size:
                self._photo = ImageTk.PhotoImage(image, master=self)
                self._photo_size = image.size
                self.itemconfigure(self._image_item, image=self._photo)
            else:
                self._photo.paste(image)
            self._last_rendered_rotation = self.rotation.copy()
        except Exception as error:
            self._fail(error)

    def capture(self):
        self._display()
        if self.failed or self._captured is None:
            raise RuntimeError("Detailed model has no active framebuffer")
        return self._captured.copy()

    def _cleanup(self, event=None):
        if event is not None and event.widget is not self:
            return
        if self._closed:
            return
        self._closed = True
        if self._resize_job is not None:
            try:
                self.after_cancel(self._resize_job)
            except tk.TclError:
                pass
            self._resize_job = None
        # PhotoImage owns a Tcl command.  Drop it on the Tk thread before the
        # interpreter disappears so later GC on a worker cannot touch Tcl.
        self._captured = None
        self._readback_buffer = None
        self._readback_size = None
        self._photo = None
        if self._image_item is not None:
            try:
                self.delete(self._image_item)
            except tk.TclError:
                pass
            self._image_item = None
        if not self.context_created or self._cgl is None:
            return
        try:
            with self._current_context():
                if self._framebuffer:
                    GL.glBindFramebuffer(
                        GL.GL_FRAMEBUFFER, self._framebuffer)
                if self._buffers:
                    GL.glDeleteBuffers(len(self._buffers), self._buffers)
                if self._textures:
                    GL.glDeleteTextures(list(self._textures.values()))
                if self._framebuffer:
                    GL.glDeleteFramebuffers(1, [self._framebuffer])
                    GL.glDeleteTextures([self._color_target])
                    GL.glDeleteRenderbuffers(1, [self._depth_target])
                GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
        except Exception:
            pass
        finally:
            self._cgl.close()
            self.context_created = False
