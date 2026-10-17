import cv2
import apriltag

# Input and output paths
video_path = '/workspaces/software-gator-rover/src/rover_core/rover_vision/test_images/Screen Recording 2026-09-22 at 8.32.11 PM.mp4'
output_path = '/workspaces/software-gator-rover/src/rover_core/rover_vision/test_images/output_video.mp4'

stream = cv2.VideoCapture(video_path)  

tags = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(tags)

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
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    results = detector.detect(gray)

    for r in results:
        # Draw a rectangle around the detected AprilTag
        (ptA, ptB, ptC, ptD) = r.corners
        ptA = (int(ptA[0]), int(ptA[1]))
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        
        # Draw the bounding box of the AprilTag detection (in green, with a thickness of 2) Reminder: BGR
        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
        
        # Draw the center of the AprilTag (in red, with a radius of 5) Color: Red
        (cx, cy) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # Draw the tag ID near the top-left corner of the bounding box (in green, with a font size of 0.6 and thickness of 2)
        tag_text = f"ID: {r.tag_id}"
        cv2.putText(frame, tag_text, (ptA[0], ptA[1] - 15), # (ptA[0], ptA[1] - 15) sets the text position above the top-left corner of the bounding box
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Write the modified frame to our output file
    out.write(frame)
    
    # Print progress so you know it hasn't frozen
    if frame_count % 30 == 0:
        print(f"Processed {frame_count} frames...", end='\r')

# Clean up
stream.release()
out.release()
print("\nProcessing complete! You can now open output_video.mp4 natively on your Mac.")