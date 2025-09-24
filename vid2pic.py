import cv2
import os

def vid2pic(video_path, output_folder):
    """
    Reads a video file and outputs all frames as PNG images.
    Args:
        video_path (str): The path to the video file (e.g., 'matlab/outputs/my_video.mp4').
        output_folder (str): The path to the output folder (e.g., 'frames').
    """

    # Check if the output folder exists, if not, create it.
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Create a VideoCapture object to read the video file.
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        raise IOError(f"Failed to open video file: {video_path}")

    # Initialize a frame counter.
    frame_num = 0

    # Loop through each frame of the video.
    while True:
        ret, frame = cap.read()

        # If 'ret' is False, it means we have reached the end of the video.
        if not ret:
            break

        frame_num += 1

        # Construct the output filename.
        output_filename = os.path.join(output_folder, f'frame_{frame_num:06d}.png')

        # Save the frame as a PNG image.
        cv2.imwrite(output_filename, frame)

        # Display progress (optional).
        if frame_num % 100 == 0:
            print(f'Processed {frame_num} frames...')

    cap.release()
    print(f'Finished processing video. Total frames: {frame_num}')

if __name__ == '__main__':
    import sys

    if len(sys.argv) != 3:
        print("Usage: python3 vid2pic.py <video_path> <output_folder>")
    else:
        video_path = sys.argv[1]
        output_folder = sys.argv[2]
        vid2pic(video_path, output_folder)
