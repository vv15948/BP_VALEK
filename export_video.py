import os
import cv2
from glob import glob
import numpy as np

# === NASTAVENÍ ===
input_folder = "output_frames"
output_video = "video_from_dt.avi"
fps = 15  # pevná snímková frekvence (např. 30 fps = 1 frame = 33.33 ms)

# === FUNKCE: získej timestamp z názvu souboru ===
def get_timestamp(path):
    name = os.path.basename(path)
    base = os.path.splitext(name)[0]
    try:
        return float(base.replace("frame_", ""))
    except:
        return 0.0

# === SEŘAĎ SOUBORY PODLE ČASU ===
image_paths = sorted(glob(os.path.join(input_folder, "frame_*.jpg")), key=get_timestamp)
timestamps = [get_timestamp(p) for p in image_paths]

if len(image_paths) < 2:
    raise Exception("❌ Potřeba alespoň 2 snímky pro vytvoření videa.")

# === ZJISTI ROZMĚRY ===
first_frame = cv2.imread(image_paths[0])
h, w, _ = first_frame.shape

# === PŘIPRAV VIDEO WRITER ===
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
video_writer = cv2.VideoWriter(output_video, fourcc, fps, (w, h))

# === VYTVÁŘEJ VIDEO SNÍMEK PO SNÍMKU ===
total_frames = 0
for i in range(len(image_paths) - 1):
    frame = cv2.imread(image_paths[i])
    t1 = timestamps[i]
    t2 = timestamps[i + 1]
    dt = t2 - t1
    percent = int((i + 1) / (len(image_paths) - 1) * 100)
    print(f"\r🟦 {i + 1}/{len(image_paths) - 1} snímků ({percent} %)", end='', flush=True)

    # kolikrát zopakovat snímek, aby odpovídal dt při zvoleném fps
    repeat = max(1, round(dt * fps))
    for _ in range(repeat):
        video_writer.write(frame)
        total_frames += 1

# poslední snímek
last_frame = cv2.imread(image_paths[-1])
video_writer.write(last_frame)
total_frames += 1

video_writer.release()

print(f"🎬 Hotovo! Vygenerováno {total_frames} snímků do '{output_video}' podle reálných časů.")
