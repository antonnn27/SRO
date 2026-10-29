import os

# Memaksa software rendering OpenGL agar stabil di Windows
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import numpy as np
import open3d as o3d
from pynput import keyboard

print("Menghubungkan ke CoppeliaSim ZMQ Remote API...")
client = RemoteAPIClient()
sim = client.require("sim")

# Memulai simulasi CoppeliaSim secara otomatis
try:
  sim.startSimulation()
  print("Simulation Started")
except Exception as e:
  print(f"Simulasi mungkin sudah berjalan: {e}")

# Mencari handle sensor pada berbagai kemungkinan path hierarki
sensor_handle = -1
possible_paths = [
    "/Quadcopter/fast3DLaserScanner",
    "/quadcopter/fast3DLaserScanner",
    "/Quadcopter/sensor",
    "/fast3DLaserScanner",
]

for path in possible_paths:
  try:
    sensor_handle = sim.getObject(path)
    if sensor_handle != -1:
      print(f"Sensor berhasil ditemukan pada path: {path}")
      break
  except Exception:
    pass

if sensor_handle == -1 or sensor_handle is None:
  print(
      "[ERROR]: Gagal menemukan objek sensor 'fast3DLaserScanner'. "
      "Periksa kembali nama objek tersebut di panel Hierarchy!"
  )
  try:
    sim.stopSimulation()
  except Exception:
    pass
  exit()

# Inisialisasi jendela visualizer Open3D
vis = o3d.visualization.Visualizer()
vis.create_window(
    window_name="3D Mapping + Trajectory & Axis Grid",
    width=1280,
    height=720
)

# Menambahkan sumbu koordinat referensi 3D (X=Merah, Y=Hijau, Z=Biru)
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=1.0,
    origin=[0, 0, 0]
)
vis.add_geometry(coordinate_frame)

pcd = o3d.geometry.PointCloud()
pcd_added = False

line_set = o3d.geometry.LineSet()
line_added = False

voxel_size = 0.05

print(
    "\n[INFO] Memulai live streaming mapping + Global Keyboard HUD."
    "\n[INFO] Tekan W, A, S, D, Q, E di keyboard untuk menggerakkan drone!"
    "\n[INFO] W/S : Maju/Mundur | A/D : Kiri/Kanan | Q/E : Naik/Turun"
    "\n[INFO] Tekan Ctrl+C di terminal untuk menyelesaikan mapping."
)

step_count = 0
all_raw_points = []
drone_trajectory = []

# Setup pynput listener untuk memantau tombol keyboard secara global
pressed_keys = set()


def on_press(key):
  try:
    if hasattr(key, "char") and key.char is not None:
      char = key.char.lower()
      if char in ["w", "a", "s", "d", "q", "e"]:
        pressed_keys.add(char.upper())
  except Exception:
    pass


def on_release(key):
  try:
    if hasattr(key, "char") and key.char is not None:
      char = key.char.lower()
      if char in ["w", "a", "s", "d", "q", "e"]:
        pressed_keys.discard(char.upper())
  except Exception:
    pass


# Menjalankan background thread untuk key listener
key_listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release
)
key_listener.start()


# Fungsi untuk merender jendela HUD OpenCV tombol WASDQE
def draw_keyboard_hud(active_keys):
  hud_img = np.zeros((220, 360, 3), dtype=np.uint8)

  key_positions = {
      "Q": (20, 20, 50, 50),
      "W": (85, 20, 50, 50),
      "E": (150, 20, 50, 50),
      "A": (20, 80, 50, 50),
      "S": (85, 80, 50, 50),
      "D": (150, 80, 50, 50),
  }

  for key, (x, y, w, h) in key_positions.items():
    is_active = key in active_keys

    # Warna tombol menyala hijau jika ditekan, abu-abu jika tidak
    color = (0, 255, 0) if is_active else (50, 50, 50)
    text_color = (0, 0, 0) if is_active else (200, 200, 200)

    cv2.rectangle(hud_img, (x, y), (x + w, y + h), color, -1)
    cv2.rectangle(hud_img, (x, y), (x + w, y + h), (200, 200, 200), 2)

    cv2.putText(
        hud_img,
        key,
        (x + 15, y + 33),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        text_color,
        2,
        cv2.LINE_AA
    )

  # Menulis teks keterangan pada panel samping HUD
  cv2.putText(
      hud_img, "LIVE KEY MONITOR", (220, 35),
      cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA
  )
  cv2.putText(
      hud_img, "W/S : Forward/Back", (220, 75),
      cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA
  )
  cv2.putText(
      hud_img, "A/D : Left/Right", (220, 105),
      cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA
  )
  cv2.putText(
      hud_img, "Q/E : Up/Down", (220, 135),
      cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA
  )

  return hud_img


try:
  while True:
    step_count += 1

    # Mengambil matriks posisi dan orientasi global sensor drone
    try:
      T_w_sens = sim.getObjectMatrix(sensor_handle, sim.handle_world)
      T_mat = np.array(T_w_sens).reshape(3, 4)
      T_homo = np.vstack([T_mat, [0, 0, 0, 1]])

      drone_pos = T_homo[:3, 3]

      if step_count % 3 == 0:
        if (
            len(drone_trajectory) == 0
            or np.linalg.norm(drone_trajectory[-1] - drone_pos) > 0.05
        ):
          drone_trajectory.append(drone_pos.copy())

    except Exception:
      time.sleep(0.03)
      continue

    # Mengambil data string titik dari signal sensor Lua CoppeliaSim
    try:
      packed_data = sim.getStringSignal("scannerPoints")

      if packed_data:
        if isinstance(packed_data, bytes):
          packed_data = packed_data.decode("utf-8")

        values = packed_data.split(",")

        if len(values) > 1:
          point_data = [float(val) for val in values if val.strip() != ""]

          if len(point_data) > 0:
            pts = np.array(point_data).reshape(-1, 3)
            pts_reduced = pts[::3]  # Mengambil sampel setiap titik ketiga

            for pt in pts_reduced:
              x_loc, y_loc, z_loc = pt[0], pt[1], pt[2]
              p_local = np.array([x_loc, y_loc, z_loc, 1.0])

              # Transformasi koordinat lokal sensor ke sistem koordinat dunia
              p_world = T_homo @ p_local
              all_raw_points.append(p_world[:3].copy())

    except Exception:
      pass

    # Memperbarui visualisasi Open3D secara berkala
    if step_count % 5 == 0:
      if len(all_raw_points) > 0:
        points_arr = np.asarray(all_raw_points, dtype=np.float64)

        temp_pcd = o3d.geometry.PointCloud()
        temp_pcd.points = o3d.utility.Vector3dVector(points_arr)
        temp_pcd_down = temp_pcd.voxel_down_sample(voxel_size=voxel_size)

        pcd.points = temp_pcd_down.points

        if not pcd_added:
          vis.add_geometry(pcd)
          pcd_added = True
        else:
          vis.update_geometry(pcd)

      if len(drone_trajectory) > 1:
        points_traj = np.asarray(drone_trajectory)
        lines = [[i, i + 1] for i in range(len(points_traj) - 1)]

        line_set.points = o3d.utility.Vector3dVector(points_traj)
        line_set.lines = o3d.utility.Vector2iVector(lines)

        colors = [[0.0, 0.8, 1.0] for _ in range(len(lines))]
        line_set.colors = o3d.utility.Vector3dVector(colors)

        if not line_added:
          vis.add_geometry(line_set)
          line_added = True
        else:
          vis.update_geometry(line_set)

    vis.poll_events()
    vis.update_renderer()

    # Memperbarui dan menampilkan jendela monitor keyboard OpenCV
    hud_frame = draw_keyboard_hud(pressed_keys)
    cv2.imshow("Live Keyboard Monitor (WASDQE)", hud_frame)

    # Keluar dari loop jika tombol ESC ditekan atau jendela ditutup
    if cv2.waitKey(1) & 0xFF == 27:
      break
    if cv2.getWindowProperty("Live Keyboard Monitor (WASDQE)", cv2.WND_PROP_VISIBLE) < 1:
      break

    time.sleep(0.01)

except KeyboardInterrupt:
  print("\n[INFO] Scanning dihentikan oleh pengguna.")

finally:
  # Menghentikan key listener dan membersihkan jendela OpenCV
  key_listener.stop()
  cv2.destroyAllWindows()


# Proses perhitungan jumlah titik akhir pemetaan
total_raw_points = len(all_raw_points)
total_mapped_points = 0

if total_raw_points > 0:
  points_arr = np.asarray(all_raw_points, dtype=np.float64)

  final_pcd = o3d.geometry.PointCloud()
  final_pcd.points = o3d.utility.Vector3dVector(points_arr)

  final_pcd_down = final_pcd.voxel_down_sample(voxel_size=voxel_size)
  total_mapped_points = len(final_pcd_down.points)

  pcd.points = final_pcd_down.points

  if not pcd_added:
    vis.add_geometry(pcd)
    pcd_added = True
  else:
    vis.update_geometry(pcd)

# Menampilkan ringkasan statistik pemindaian ke terminal
print("\n" + "=" * 48)
print("              HASIL AKHIR 3D MAPPING")
print("=" * 48)
print(f"Total titik hasil pemindaian : {total_raw_points:,}")
print(f"Total titik mapping akhir   : {total_mapped_points:,}")
print(f"Voxel size                  : {voxel_size} m")
print(f"Total titik lintasan drone  : {len(drone_trajectory):,}")
print("=" * 48 + "\n")


# Menambahkan bola penanda START (Hijau) dan STOP (Merah) di akhir sesi
if len(drone_trajectory) > 0:
  start_pt = drone_trajectory[0]
  stop_pt = drone_trajectory[-1]

  start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
  start_sphere.translate(start_pt)
  start_sphere.paint_uniform_color([0.0, 1.0, 0.0])

  stop_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
  stop_sphere.translate(stop_pt)
  stop_sphere.paint_uniform_color([1.0, 0.0, 0.0])

  vis.add_geometry(start_sphere)
  vis.add_geometry(stop_sphere)

  vis.update_geometry(pcd)
  vis.update_renderer()

  print("[INFO] Titik START (Hijau) dan STOP (Merah) ditambahkan.")

print(
    "[INFO] Jendela Open3D aktif. "
    "Tutup jendela manual atau tekan Ctrl+C untuk keluar."
)

# Mempertahankan jendela Open3D agar hasil akhir dapat diperiksa interaktif
try:
  while vis.poll_events():
    vis.update_renderer()
    time.sleep(0.01)

except KeyboardInterrupt:
  pass

finally:
  vis.destroy_window()

  try:
    sim.stopSimulation()
    print("[INFO] Simulasi selesai.")
  except Exception:
    pass