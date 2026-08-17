"""
IMU 3D Attitude Viewer — quaternion-driven OpenGL rendering (no gimbal lock).

Usage:
  pip install pygame pyopengl
  python imu_3d_viewer.py COM11
  python imu_3d_viewer.py COM11 115200

Firmware output format (13 columns):
  Ax,Ay,Az,  Gx,Gy,Gz,  Roll,Pitch,Yaw,  q0(w),q1(x),q2(y),q3(z)

Rotation is built directly from the Madgwick quaternion (cols 9–12),
so the 3D view is gimbal-lock free.  Euler angles are shown in the HUD
for reference only.

Controls:
  R      — reset view
  F      — toggle fullscreen
  G      — toggle ground grid
  A      — toggle auto-rotate (when no data)
  ESC/Q  — quit
"""

import sys
import math
import threading
import time

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# ============================================================
WIDTH, HEIGHT = 900, 680
CAMERA_DISTANCE = 7.0
# ============================================================

def quat_to_matrix(q0, q1, q2, q3):
    """Convert quaternion (w,x,y,z) to 4x4 OpenGL rotation matrix.
    Assumes unit quaternion.  Column-major for glMultMatrixf."""
    x2 = q1 + q1;  y2 = q2 + q2;  z2 = q3 + q3
    xx = q1 * x2;  yy = q2 * y2;  zz = q3 * z2
    xy = q1 * y2;  xz = q1 * z2;  yz = q2 * z2
    wx = q0 * x2;  wy = q0 * y2;  wz = q0 * z2

    # Row-major matrix (for readability), then stored column-major
    # R = [[1-yy-zz,   xy-wz,    xz+wy,  0],
    #      [  xy+wz,  1-xx-zz,   yz-wx,  0],
    #      [  xz-wy,   yz+wx,   1-xx-yy, 0],
    #      [  0,       0,        0,       1]]
    return [
        1-yy-zz,  xy-wz,    xz+wy,    0.0,
        xy+wz,    1-xx-zz,  yz-wx,    0.0,
        xz-wy,    yz+wx,    1-xx-yy,  0.0,
        0.0,      0.0,      0.0,      1.0,
    ]


class IMU3DViewer:
    def __init__(self, port, baud):
        self.ser = None
        if port and is_serial_port(port):
            if not HAS_SERIAL:
                raise ImportError("pyserial not installed. Run: pip install pyserial")
            self.ser = serial.Serial(port, baud, timeout=0.005)
            print(f"Serial {port} opened @ {baud}, waiting for data...")
        elif port:
            print(f"File mode: {port}")
            self._file_iter = open(port, 'r')
            self._file_done = False

        # Current attitude
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.ax_val = self.ay_val = self.az_val = 0.0
        self.gx_val = self.gy_val = self.gz_val = 0.0

        # State
        self.running = True
        self.line_count = 0
        self.show_grid = True
        self.auto_spin = True      # auto-rotate when no data
        self.auto_angle = 0.0
        self.last_data_time = 0.0
        self.fps = 0.0
        self._t_last = time.time()
        self._frame_count = 0
        self._font = None

    # ── Serial reader thread ──
    def serial_thread(self):
        buf = b""
        while self.running:
            try:
                data = self.ser.read(256)
                if not data:
                    continue
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line or line.startswith(b'#'):
                        continue
                    try:
                        parts = [float(x) for x in line.split(b',')]
                        if len(parts) >= 13:
                            self._apply_data(parts)
                    except (ValueError, IndexError):
                        pass
            except Exception:
                break

    def _file_pump(self):
        """Pump a few lines from file (non-blocking)."""
        if not hasattr(self, '_file_iter'):
            return
        for _ in range(5):
            try:
                line = next(self._file_iter)
            except StopIteration:
                self._file_done = True
                return
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            try:
                parts = [float(x) for x in line.split(',')]
                if len(parts) >= 13:
                    self._apply_data(parts)
            except (ValueError, IndexError):
                pass

    def _apply_data(self, parts):
        """Parse a 13-column row and update attitude."""
        self.ax_val = parts[0]; self.ay_val = parts[1]; self.az_val = parts[2]
        self.gx_val = parts[3]; self.gy_val = parts[4]; self.gz_val = parts[5]
        self.roll  = parts[6]; self.pitch = parts[7]; self.yaw   = parts[8]
        self.q0 = parts[9]; self.q1 = parts[10]
        self.q2 = parts[11]; self.q3 = parts[12]

        # Normalize quaternion
        n = math.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
        if n > 1e-9:
            self.q0 /= n; self.q1 /= n; self.q2 /= n; self.q3 /= n

        self.line_count += 1
        self.last_data_time = time.time()

    # ── Drawing ──
    def draw_cube(self, size=1.2):
        """Draw a colored cube with labeled axes (APM/Pixhawk style)."""
        s = size / 2
        verts = [
            [-s, -s,  s], [ s, -s,  s], [ s,  s,  s], [-s,  s,  s],  # front
            [-s, -s, -s], [ s, -s, -s], [ s,  s, -s], [-s,  s, -s],  # back
        ]
        faces = [
            (0,1,2,3), (4,5,6,7),  # front (+Z), back (-Z)
            (1,5,6,2), (0,4,7,3),  # right (+X), left (-X)
            (3,2,6,7), (0,1,5,4),  # top (+Y), bottom (-Y)
        ]
        face_colors = [
            (0.2, 0.3, 0.9),  # +Z → blue
            (0.15, 0.22, 0.7), # -Z → dark blue
            (0.9, 0.2, 0.2),  # +X → red
            (0.7, 0.15, 0.15),# -X → dark red
            (0.2, 0.8, 0.2),  # +Y → green
            (0.15, 0.6, 0.15),# -Y → dark green
        ]
        edges = [
            (0,1),(1,2),(2,3),(3,0),  # front
            (4,5),(5,6),(6,7),(7,4),  # back
            (0,4),(1,5),(2,6),(3,7),  # sides
        ]

        glBegin(GL_QUADS)
        for i, face in enumerate(faces):
            glColor3f(*face_colors[i])
            for v in face:
                glVertex3fv(verts[v])
        glEnd()

        glLineWidth(1.5)
        glColor3f(0.05, 0.05, 0.05)
        glBegin(GL_LINES)
        for edge in edges:
            for v in edge:
                glVertex3fv(verts[v])
        glEnd()

        # Axis arrows from center
        glLineWidth(3.0)
        glBegin(GL_LINES)
        glColor3f(1, 0.2, 0.2); glVertex3f(0, 0, 0); glVertex3f(2.0, 0, 0)   # +X red
        glColor3f(0.2, 1, 0.2); glVertex3f(0, 0, 0); glVertex3f(0, 2.0, 0)   # +Y green
        glColor3f(0.2, 0.4, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 2.0)   # +Z blue
        glEnd()
        glLineWidth(1.0)

    def draw_grid(self):
        """Reference ground grid."""
        glColor4f(0.3, 0.4, 0.5, 0.4)
        glLineWidth(0.5)
        glBegin(GL_LINES)
        for i in range(-10, 11):
            glVertex3f(i, -3, -10); glVertex3f(i, -3, 10)
            glVertex3f(-10, -3, i); glVertex3f(10, -3, i)
        glEnd()
        glLineWidth(1.0)

    def draw_hud_text(self):
        """Render overlay text via pygame surfaces (blended)."""
        if self._font is None:
            return

        lines = []
        # Data status
        if self.line_count == 0:
            lines.append("WAITING FOR DATA — auto-rotating...")
        else:
            lines.append(f"Samples: {self.line_count}  |  FPS: {self.fps:.0f}")

        # Accel
        lines.append(f"Accel:  X={self.ax_val:+.3f}  Y={self.ay_val:+.3f}  Z={self.az_val:+.3f} g")

        # Gyro
        lines.append(f"Gyro:   X={self.gx_val:+.1f}  Y={self.gy_val:+.1f}  Z={self.gz_val:+.1f} °/s")

        # Euler
        lines.append(f"Euler:  R={self.roll:+.1f}°  P={self.pitch:+.1f}°  Y={self.yaw:+.1f}°")

        # Quaternion
        lines.append(f"Quat:   w={self.q0:+.4f}  x={self.q1:+.4f}  y={self.q2:+.4f}  z={self.q3:+.4f}")

        # Keyboard hints
        lines.append("[R]eset  [F]ullscreen  [G]rid  [A]uto-spin  [ESC] Quit")

        screen = pygame.display.get_surface()
        y = 10
        for text in lines:
            surf = self._font.render(text, True, (220, 220, 240))
            screen.blit(surf, (12, y))
            y += 18

    def _draw_gl_scene(self):
        """Main 3D scene."""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluLookAt(0, 0, CAMERA_DISTANCE,  0, 0, 0,  0, 1, 0)

        # Apply rotation from quaternion
        rot_mat = quat_to_matrix(self.q0, self.q1, self.q2, self.q3)
        glMultMatrixf(rot_mat)

        # Grid (in world frame — rotated with cube, looks like ground)
        if self.show_grid:
            self.draw_grid()

        self.draw_cube()

    def run(self):
        pygame.init()
        flags = DOUBLEBUF | OPENGL
        pygame.display.set_mode((WIDTH, HEIGHT), flags)
        pygame.display.set_caption("IMU 3D Attitude Viewer  |  Quaternion-driven  |  [R]eset [A]uto-spin [G]rid")
        self._font = pygame.font.Font(None, 20)

        # OpenGL setup
        glViewport(0, 0, WIDTH, HEIGHT)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, WIDTH / HEIGHT, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glClearColor(0.08, 0.10, 0.16, 1.0)  # dark navy background

        clock = pygame.time.Clock()

        # Start reader
        if self.ser:
            threading.Thread(target=self.serial_thread, daemon=True).start()

        while self.running:
            dt = clock.tick(60) / 1000.0

            # ── Events ──
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False
                elif event.type == KEYDOWN:
                    if event.key in (K_ESCAPE, K_q):
                        self.running = False
                    elif event.key == K_r:
                        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
                        self.roll = self.pitch = self.yaw = 0.0
                    elif event.key == K_f:
                        pygame.display.toggle_fullscreen()
                    elif event.key == K_g:
                        self.show_grid = not self.show_grid
                    elif event.key == K_a:
                        self.auto_spin = not self.auto_spin
                        print(f"Auto-spin: {self.auto_spin}")

            # ── File pump (non-serial mode) ──
            if not self.ser:
                self._file_pump()

            # ── Auto-spin when idle ──
            if self.line_count == 0 or (self.auto_spin and time.time() - self.last_data_time > 2.0):
                self.auto_angle += 30.0 * dt
                # Gentle auto-rotation around Y (heading) + slight wobble
                cy = math.cos(math.radians(self.auto_angle * 0.5))
                sy = math.sin(math.radians(self.auto_angle * 0.5))
                cp = math.cos(math.radians(15.0 * math.sin(self.auto_angle * 0.3) * 0.5))
                sp = math.sin(math.radians(15.0 * math.sin(self.auto_angle * 0.3) * 0.5))
                # q_auto = q_yaw * q_pitch
                self.q0 = cy * cp
                self.q1 = 0.0
                self.q2 = sy * cp
                self.q3 = cy * sp

            # ── FPS counter ──
            self._frame_count += 1
            if self._frame_count % 15 == 0:
                t = time.time()
                self.fps = 15.0 / (t - self._t_last + 1e-9)
                self._t_last = t

            # ── Render ──
            self._draw_gl_scene()
            self.draw_hud_text()
            pygame.display.flip()

        pygame.quit()
        if self.ser:
            self.ser.close()
        if hasattr(self, '_file_iter'):
            self._file_iter.close()


def is_serial_port(s):
    return s.startswith('COM') or s.startswith('/dev/tty') or s.startswith('/dev/cu')


if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else None
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    if port is None:
        print(__doc__)
        print("Examples:")
        print("  python imu_3d_viewer.py COM11")
        print("  python imu_3d_viewer.py COM11 115200")
        print("  python imu_3d_viewer.py data.csv   (replay CSV file)")
        sys.exit(0)

    try:
        IMU3DViewer(port, baud).run()
    except ImportError as e:
        print(f"ERROR: {e}")
        sys.exit(1)
