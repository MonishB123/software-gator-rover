import cv2

# Input and output paths
video_path = '/workspaces/software-gator-rover/src/rover_core/rover_vision/test_images/Screen Recording 2026-09-22 at 8.32.11 PM.mp4'
output_path = '/workspaces/software-gator-rover/src/rover_core/rover_vision/test_images/output_video.mp4'

stream = cv2.VideoCapture(video_path)  

if not stream.isOpened():
    print(f"Error: Could not open video file at {video_path}")
    exit()

# Get the video's width, height, and frames-per-second (fps)
frame_width = int(stream.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(stream.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(stream.get(cv2.CAP_PROP_FPS))
if fps == 0: 
    fps = 30  # Fallback just in case

# Initialize the VideoWriter to save the video
fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))

print("Processing video... (No window will open)")
print(f"Output will be saved to: {output_path}")

frame_count = 0
while(True):
    ret, frame = stream.read()
    if not ret:
        print("\nEnd of video reached or cannot read frame.")
        break
    
    frame_count += 1
    
    # ---------------------------------------------------------
    # Put your AprilTag detection and drawing logic here!
    # For example: cv2.rectangle(frame, pt1, pt2, color, thick)
    # ---------------------------------------------------------

    # Write the modified frame to our output file
    out.write(frame)
    
    # Print progress so you know it hasn't frozen
    if frame_count % 30 == 0:
        print(f"Processed {frame_count} frames...", end='\r')

# Clean up
stream.release()
out.release()
print("\nProcessing complete! You can now open output_video.mp4 natively on your Mac.")