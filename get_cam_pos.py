import pyrealsense2 as rs
import numpy as np

def get_pitch_angle():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    config = rs.config()
    config.enable_stream(rs.stream.pose)

    # Start streaming
    try:
        pipeline.start(config)
    except RuntimeError as e:
        print(f"Error starting the pipeline: {e}")
        return None

    try:
        while True:
            # Wait for a coherent set of frames
            frames = pipeline.wait_for_frames()
            pose_frame = frames.get_pose_frame()

            if pose_frame:
                # Get pose data
                pose_data = pose_frame.get_pose_data()

                # Extract rotation quaternion
                rotation = pose_data.rotation
                w, x, y, z = rotation.w, rotation.x, rotation.y, rotation.z

                # Convert quaternion to Euler angles
                pitch = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
                pitch_degrees = np.degrees(pitch)

                return pitch_degrees

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    pitch_angle = get_pitch_angle()
    if pitch_angle is not None:
        print(f"Pitch angle: {pitch_angle} degrees")
    else:
        print("Failed to get pitch angle.")