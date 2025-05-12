import cv2
import os
import numpy as np
from glob import glob

# === SLOŽKY ===
rgb_folder = "camera_frames"
depth_folder = "camera_frames"
voxel_folder = "voxel_framesss"
output_folder = "output_frames"

# === ROZMĚRY OBRAZŮ ===
w, h = 640, 480  # Rozměry jednotlivých snímků RGB a Depth
target_voxel_width = 1707
target_height = 960  # Výška výstupního obrázku

# === VYTVOŘ ČISTOU SLOŽKU PRO VÝSTUP ===
if os.path.exists(output_folder):
    for f in os.listdir(output_folder):
        os.remove(os.path.join(output_folder, f))
else:
    os.makedirs(output_folder)

# === ZÍSKÁNÍ TIMESTAMPU ZE JMÉNA SOUBORU ===
def get_timestamp(path):
    name = os.path.basename(path)
    name_no_ext = os.path.splitext(name)[0]
    parts = name_no_ext.split("_", 1)
    if len(parts) == 2:
        try:
            return float(parts[1])
        except:
            return -1
    return -1

# === NAČTENÍ A SEŘAZENÍ SOUBORŮ ===
rgb_paths = sorted(glob(os.path.join(rgb_folder, "rgb_*.jpg")), key=get_timestamp)
depth_paths = sorted(glob(os.path.join(depth_folder, "depth_*.png")), key=get_timestamp)
voxel_paths = sorted(glob(os.path.join(voxel_folder, "voxel_*.png")), key=get_timestamp)

rgb_times = [get_timestamp(p) for p in rgb_paths]
depth_times = [get_timestamp(p) for p in depth_paths]
voxel_times = [get_timestamp(p) for p in voxel_paths]

# === VOXEL FRONTY ===
voxel_queue = list(zip(voxel_times, voxel_paths))
voxel_index = 0
last_voxel_img = None
voxel_loaded = False

# === HLAVNÍ SMYČKA ===
for i in range(len(rgb_times) - 1):
    t1 = rgb_times[i]
    t2 = rgb_times[i + 1]

    # Najdi voxel mezi t1 a t2
    while voxel_index < len(voxel_queue):
        vt, vp = voxel_queue[voxel_index]
        if vt <= t1:
            voxel_index += 1
            continue
        if vt <= t2:
            last_voxel_img = cv2.imread(vp)
            voxel_loaded = True
            voxel_index += 1
        else:
            break

    if not voxel_loaded:
        continue

    # Najdi nejnovější depth snímek <= t2
    depth_candidates = [j for j, t in enumerate(depth_times) if t <= t2]
    if not depth_candidates:
        continue
    idx_depth = depth_candidates[-1]

    # Načti RGB a Depth
    rgb_img = cv2.imread(rgb_paths[i + 1])
    depth_img = cv2.imread(depth_paths[idx_depth])
    voxel_img = last_voxel_img.copy()

    # Změň velikost RGB a Depth
    rgb_img = cv2.resize(rgb_img, (w, h))
    depth_img = cv2.resize(depth_img, (w, h))
    left = np.vstack((rgb_img, depth_img))  # 640 × 960

    # Přizpůsob voxel obrázek výšce 960
    voxel_img = cv2.resize(voxel_img, (target_voxel_width, target_height))

    # Spojíme horizontálně: [RGB+Depth | MATLAB Plot]
    combined = np.hstack((left, voxel_img))

    # Ulož
    timestamp = rgb_times[i + 1]
    output_path = os.path.join(output_folder, f"frame_{timestamp:.6f}.jpg")
    cv2.imwrite(output_path, combined)
    print(f"✅ Uloženo: {output_path}")

print("🎉 Hotovo: všechny kombinované snímky jsou ve složce 'output_frames'")
