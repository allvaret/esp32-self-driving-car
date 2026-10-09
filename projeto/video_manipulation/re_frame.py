from pathlib import Path
import cv2

# Load the input video
input_path = "D:\\Projetos\\Programacao\\Python\\IA\\Esp32_Self_driver_car\\assets\simulation_Scar\\12sec.mp4"
cap = cv2.VideoCapture(input_path)

# create the output path
path_obj = Path(input_path)
output_path = f"{path_obj.stem}_reframed15{path_obj.suffix}"

#  Get original video properties
source_fps = cap.get(cv2.CAP_PROP_FPS)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

#  Define your target FPS and skip interval
target_fps = 15.0
skip_interval = max(1, round(source_fps / target_fps))


fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_path, fourcc, target_fps, (width, height))

frame_idx = 0
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    if frame_idx % skip_interval == 0:
        out.write(frame)

    frame_idx += 1

cap.release()
out.release()
cv2.destroyAllWindows()
print(f"Video saved as: {output_path}")
