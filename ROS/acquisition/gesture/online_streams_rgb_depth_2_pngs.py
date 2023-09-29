
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import json
from colorama import Back
import keyboard

# CONSTANTS
NUMBER_OF_SAMPLES_FOR_GESTURE = 150
NUMBER_OF_GESTURES = 12
pose_png_path = "pose_pngs/"
PNG_MILLISECOND = 4000    # display a pose window for given milliseconds
DATASET_PATH = "D:\gesture_dataset"

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
cfg = pipeline.start(config)
dev = cfg.get_device()

colorizer = rs.colorizer()
colorizer.set_option(rs.option.color_scheme, 0)

depth_sensor = dev.first_depth_sensor()
depth_sensor.set_option(rs.option.visual_preset, 2)

# Skip 5 first frames to give the Auto-Exposure time to adjust
for x in range(5):
    pipeline.wait_for_frames()
print(Back.GREEN + "System Ready!" + Back.RESET)

record = None
try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        align = rs.align(rs.stream.color)
        frame_aligned = align.process(frames)
        color_image = np.asanyarray(frame_aligned.get_color_frame().get_data())
        depth_image = np.asanyarray(colorizer.colorize(frame_aligned.get_depth_frame()).get_data())

        images = np.hstack((color_image, depth_image))
        if keyboard.is_pressed('s'):
            record = True
        elif keyboard.is_pressed('q'):
            record = False
        # cv2.waitKey(1)
        # Press 's' to start saving iter
        if record==True:
            print("Inside the IF")
            pipeline.stop()
            res = input("Do you want to record a sample? (y/n) ")
            if res.lower() in ["y", "n"] and res.lower() == "y":
                cv2.destroyAllWindows()
                print("Preparing recording")
                parent_folder_name = input("Enter folder name: (min 4 chars) ")
                while len(parent_folder_name) < 4:
                    parent_folder_name = input("Enter folder name: (min 4 chars) ")
                parent_folder_name = os.path.join(DATASET_PATH, parent_folder_name)
                if not os.path.isdir(parent_folder_name):
                    os.mkdir(parent_folder_name)
                else:
                    print("FOLDER ERROR. ALREADY EXISTS")
                    confirm = input("Do you want proceed? (y/n) ")
                    if confirm.lower() == "n":
                        cv2.destroyAllWindows()
                        break
                gestures_captured = 0
                samples_captured = 0
                distance = input("Set distance of sensor from subject: (meters) [1, 2, 5] ")
                while distance != "1" and distance != "5" and distance != "2":
                    distance = input("Set distance of sensor from subject: (meters) [1, 2, 5] ")
                gender = input("Gender Input? (m/f) ")
                while gender.lower() not in ["m", "f"]:
                    gender = input("Gender Input? (m/f) ")
                if gender.lower() == "m":
                    gender = "Male"
                else:
                    gender = "Female"
                age = input("Age Input? ")
                lighting_lvl = input("Lighting Level? ((l)ow, (n)ormal, (h)igh) ")
                while lighting_lvl.lower() not in ["l", "n", "h"]:
                    lighting_lvl = input("Lighting Level? ((l)ow, (n)ormal, (h)igh) ")
                if lighting_lvl.lower() == "l":
                    lighting_lvl = "Low"
                elif lighting_lvl.lower() == "n":
                    lighting_lvl = "Normal"
                else:
                    lighting_lvl = "High"

                json_string = json.dumps({"gender": gender, "age": age, "lighting_lvl": lighting_lvl, "distance": distance})
                with open(os.path.join(parent_folder_name, 'info.json'), 'a') as outfile:
                    outfile.write(json_string+"\n")

                for index in range(NUMBER_OF_GESTURES):

                    rgb_folder = os.path.join(parent_folder_name,
                                              "Pose{:02d}_{}m_rgb".format(index + 1, distance))
                    depth_folder = os.path.join(parent_folder_name,
                                                "Pose{:02d}_{}m_depth".format(index + 1, distance))

                    if os.path.isdir(rgb_folder) or os.path.isdir(depth_folder):
                        print("Warning: FOLDER RGB/DEPTH POSE {:d} already exists.".format(index))
                        continue
                    os.mkdir(rgb_folder)
                    os.mkdir(depth_folder)

                for hand in ["sx", "dx"]:

                    while gestures_captured < NUMBER_OF_GESTURES:
                        print(("Ready to record {:d}th gesture with {} hand?".format(gestures_captured + 1, hand)))
                        pose = cv2.imread(os.path.join(pose_png_path, "pose_{:02d}_{}.png".format(gestures_captured + 1, hand)))
                        name_window = "{}th pose".format(gestures_captured + 1)
                        #cv2.imshow(name_window, pose)
                        #cv2.setWindowProperty(name_window, cv2.WND_PROP_TOPMOST, 1)
                        escape = cv2.waitKey(PNG_MILLISECOND)
                        if escape:
                            cv2.destroyWindow(name_window)
                        rdy = input("(y/n) : ")
                        while rdy.lower() not in ["y", "n"]:
                            rdy = input("Ready to record {:d}th gesture with {} hand? (y/n) ".format(gestures_captured + 1, hand))
                        if rdy.lower() == "n":
                            gestures_captured += 1
                            samples_captured += NUMBER_OF_SAMPLES_FOR_GESTURE
                            continue
                        rgb_folder = os.path.join(parent_folder_name,
                                                  "Pose{:02d}_{}m_rgb".format(gestures_captured + 1, distance))
                        depth_folder = os.path.join(parent_folder_name,
                                                    "Pose{:02d}_{}m_depth".format(gestures_captured + 1, distance))
                        path_save_rgb = os.path.join(rgb_folder, hand)
                        path_save_depth = os.path.join(depth_folder, hand)
                        if os.path.isdir(path_save_rgb) or os.path.isdir(path_save_depth):
                            print("ERROR FOLDER RGB/DEPTH {}th gesture {}. Already exists! SKIP ALL.".format(gestures_captured + 1, hand))
                            break
                        os.mkdir(path_save_rgb)
                        os.mkdir(path_save_depth)

                        pipeline.start(config)
                        print("Adjusting exposure...")
                        # Skip 20 (15+5) first frames to give the Auto-Exposure time to adjust
                        for x in range(15):
                            pipeline.wait_for_frames()
                        print("Done. Recording...")
                        for x in range(5):  # Handle delay of print
                            pipeline.wait_for_frames()
                        for x in range(NUMBER_OF_SAMPLES_FOR_GESTURE):
                            frames = pipeline.wait_for_frames()
                            align = rs.align(rs.stream.color)
                            frame_aligned = align.process(frames)
                            color_image = np.asanyarray(frame_aligned.get_color_frame().get_data())
                            depth_image = np.asanyarray(colorizer.colorize(frame_aligned.get_depth_frame()).get_data())

                            filename_image = '{:04d}.jpg'.format(x)
                            cv2.imwrite(os.path.join(path_save_rgb, filename_image), color_image)
                            cv2.imwrite(os.path.join(path_save_depth, filename_image), depth_image)
                            samples_captured += 1
                        gestures_captured += 1
                        if gestures_captured < NUMBER_OF_GESTURES:
                            print("Done, go to next gesture.")

                        pipeline.stop()
                    print("All gestures with {} hand recorded!".format(hand))
                    gestures_captured = 0
                    samples_captured = 0
            pipeline.start(config)

        elif record==False:
            cv2.destroyAllWindows()
            break

        # Show images
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', images)

finally:

    # Stop streaming
    pipeline.stop()
