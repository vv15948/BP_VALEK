import socket
import pyrealsense2 as rs
import mediapipe as mp
import cv2
import numpy as np
import time
import json

# Inicializace MediaPipe Pose
mp_pose = mp.solutions.pose

# Inicializace RealSense kamery
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Funkce pro zarovnání hloubky na RGB obraz
align = rs.align(rs.stream.color)

# Získání intrinsických parametrů kamery
profile = pipeline.get_active_profile()
depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fx, fy = depth_intrinsics.fx, depth_intrinsics.fy
cx, cy = depth_intrinsics.ppx, depth_intrinsics.ppy

# Aplikace filtrů pro zlepšení hloubkových dat
depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)
spatial_filter = rs.spatial_filter()
temporal_filter = rs.temporal_filter()
hole_filling = rs.hole_filling_filter()

# Nastavení slabších filtrů
spatial_filter.set_option(rs.option.filter_magnitude, 1)
spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.3)
temporal_filter.set_option(rs.option.filter_smooth_delta, 10)

# Definování klíčových bodů
keypoints = {
    "head": 0, "shoulder_left": 11, "shoulder_right": 12, "hip_left": 23, "hip_right": 24,
    "elbow_left": 13, "elbow_right": 14, "wrist_left": 15, "wrist_right": 16,
    "knee_left": 25, "knee_right": 26
}

# Nastavení TCP/IP připojení k MATLABu
HOST = "127.0.0.1"
PORT = 5051
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
print("Připojeno k MATLAB serveru")

# Zpracování obrazu
try:
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)  # Zarovnání hloubky na RGB
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not color_frame or not depth_frame:
                continue

            # Aplikace filtrů na hloubkový obraz
            depth_frame = depth_to_disparity.process(depth_frame)
            depth_frame = spatial_filter.process(depth_frame)
            depth_frame = temporal_filter.process(depth_frame)
            depth_frame = disparity_to_depth.process(depth_frame)
            depth_frame = hole_filling.process(depth_frame)

            # Převod hloubkového snímku na numpy pole
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            frame = np.asanyarray(color_frame.get_data())
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            pose_data = {"time": time.time(), "landmarks": {}}

            if results.pose_landmarks:
                for name, idx in keypoints.items():
                    lm = results.pose_landmarks.landmark[idx]
                    x_pixel = int(lm.x * frame.shape[1])
                    y_pixel = int(lm.y * frame.shape[0])

                    # Získání hloubky z RealSense
                    if 0 <= x_pixel < frame.shape[1] and 0 <= y_pixel < frame.shape[0]:
                        D = depth_image[y_pixel, x_pixel] / 1000.0  # Převod na metry
                    else:
                        D = 10  # Pokud je mimo rozsah, nastavíme hloubku na 0

                    # Přepočet na kartézské souřadnice
                    X = (x_pixel - cx) * D / fx
                    Y = (-(y_pixel - cy) * D / fy)+0.7
                    Z = D-2.05  # Hloubka přímo jako vzdálenost od kamery

                    # Uložení dat pro MATLAB
                    pose_data["landmarks"][name] = {"x": X, "y": Z, "z": Y}
                    print(f"{name}: X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m (Depth={D:.3f} m)")
                    
                    # Vykreslení bodů do RGB obrazu
                    cv2.circle(frame, (x_pixel, y_pixel), 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"{name}: {D:.2f}m", (x_pixel + 5, y_pixel - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                    
                    # Vykreslení bodů do hloubkového obrazu
                    cv2.circle(depth_colormap, (x_pixel, y_pixel), 5, (0, 255, 0), -1)
                    cv2.putText(depth_colormap, f"{D:.2f}m", (x_pixel + 5, y_pixel - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Odeslání dat do MATLABu
            data_str = json.dumps(pose_data) + "\n"
            sock.sendall(data_str.encode())

            # Zobrazení OpenCV oken
            cv2.imshow("RGB Image", frame)
            cv2.imshow("Aligned Depth Map", depth_colormap)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    pipeline.stop()
    sock.close()
    cv2.destroyAllWindows()
    print("Připojení ukončeno")
