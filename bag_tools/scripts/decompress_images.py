#!/usr/bin/env python3


import os
import rosbag
import numpy as np
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import logging
import sys
import argparse
import time

start_total = time.time()

def logging_configuration():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        handlers=[
            logging.FileHandler("procesamiento.log"),     # Save to file
            logging.StreamHandler(sys.stdout)             # Display on console
        ]
    )


# ............................................................................


def contains_compressed_images(bag_path):
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            for _, msg, _ in bag.read_messages():
                if msg._type == 'sensor_msgs/CompressedImage':
                    return True
    except Exception as e:
        logging.error(f"Error reading file {bag_path}: {e}")
    return False


def is_there_bags_with_compressed_images(dir_path):
    for root,_ , files in os.walk(dir_path):
        # we make it go to files containing certain words such as ‘camera’ or ‘stereo’ or ‘compressed’ to go faster
        files.sort(key = lambda name: 0 if 'camera' in name or 'stereo' in name  or 'compressed' in name else 1)
        
        for f in files:
            if not (f.endswith('.bag') or f.endswith('.bag.active')):
                continue

            bag_file = os.path.join(root,f)
            try:
                with rosbag.Bag(bag_file, 'r') as bag:
                    for _, msg, _ in bag.read_messages():
                        if msg._type == 'sensor_msgs/CompressedImage':
                            logging.info(f"There is at least one bag with compressed images in the directory {dir_path}: proceed to descompress it")
                            return True
            except Exception as e:
                logging.error(f"Error while reading {bag_file}: {e}")
                continue

    logging.info(f"There are no bags with images in the directory {dir_path}")
    return False


def decompress_bag(input_bag_path, output_bag_path):
    try:
        with rosbag.Bag(input_bag_path, 'r') as inbag, rosbag.Bag(output_bag_path, 'w') as outbag:
            for topic, msg, t in inbag.read_messages(): 
                # first we filter to check that the message type is the one we want (CompressedImage), if it is not, we rewrite it and move on to another message
                if msg._type != 'sensor_msgs/CompressedImage':
                    outbag.write(topic, msg, t)
                    continue
                try:
                    
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    # no ‘.reshape()’ is needed since imencode() returns the decoded image correctly
                    
                    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
                    if img is None:
                        logging.warning(f"Failed to decode image in {topic}")
                        continue

                    # process: bytes (ROS, PNG) -> numpy array (image) -> decompress image -> bytes (ROS, message type Image())
                    # we access the encoding that is stored in the ‘.format’ attribute
                    try:
                        if hasattr(msg, 'format') and msg.format:
                            encoding = msg.format.split(';')[0].strip()
                        else:
                            logging.warning(f"No format field or empty format in message from {topic}")
                            continue
                        
                    except Exception as e:
                        logging.warning(f"Unable to extract encoding from msg.format = '{msg.format}': {e}")
                        continue
                    
                    # we create a message of type Image and configure all its parameters according to the encoding
                    raw_msg = Image()
                    raw_msg.header = msg.header if hasattr(msg, 'header') else Header()
                    raw_msg.encoding = encoding

                    if len(img.shape) == 2:
                        # we have a 1 channel image
                        raw_msg.height, raw_msg.width = img.shape
                        raw_msg.step = raw_msg.width
                        # logging.debug(f"[{topic}] Decoding with encoding={encoding}, shape={img.shape}, step={raw_msg.step}")
                    elif len(img.shape) == 3:
                        # we have a 3 or + channel image
                        raw_msg.height, raw_msg.width, channels = img.shape
                        raw_msg.step = raw_msg.width * channels
                        # logging.debug(f"[{topic}] Decoding with encoding={encoding}, shape={img.shape}, step={raw_msg.step}")
                    else:
                        logging.warning(f"Image with unexpected shape in {topic}, shape = {img.shape}")
                        continue
                        

                    raw_msg.data = img.tobytes()
                    raw_msg.is_bigendian = 0

                    # rename the topic (delete /compressed)
                    raw_topic = topic.replace('/compressed', '')
                    outbag.write(raw_topic, raw_msg, t)

                except Exception as e:
                    logging.error(f"Error decompressing image in {topic}: {e}")
                    return False
        return True
    
    except Exception as e:
        logging.error(f"Error decompressing file {input_bag_path}: {e}")
        return False


def decompress_bag_in_directory(input_bag_path, output_dir_path):
    # we check that the output directory exists, if not, we create it
    os.makedirs(output_dir_path, exist_ok=True)

    input_filename = os.path.basename(input_bag_path)
    
    if input_filename.endswith(".bag.active"):
        base_name = input_filename[:-11]
    elif input_filename.endswith("_compressed.bag"):
        base_name = input_filename[:-15]
    elif input_filename.endswith(".bag"):
        base_name = input_filename[:-4]
    else:
        base_name = os.path.splitext(input_filename)[0]
    
    output_filename = base_name + "_raw.bag"
    output_bag_path = os.path.join(output_dir_path, output_filename)
    
    try:
        logging.info(f"Processing: {input_filename} -> {output_filename}")
        success = decompress_bag(input_bag_path, output_bag_path)

        if not success:
            logging.warning(f"Decompression function returned False for {input_bag_path}")
            return False, None
        
        if not os.path.exists(output_bag_path):
            logging.error(f"Expected output bag {output_bag_path} not found")
            return False, None
        
        with rosbag.Bag(output_bag_path, 'r') as bag:
            try:
                next(bag.read_messages())
            except StopIteration:
                logging.warning(f"Decompressed bag {output_bag_path} is empty")
                return False, None
            
        return True, output_bag_path
    
    except Exception as e:
        logging.error(f"Error decompressing {input_bag_path} in directory {output_dir_path}: {e}")
        return False, None

def decompress_all_directory(input_dir_path, output_dir_path):
    for root, _, files in os.walk(input_dir_path):
        # we calculate the relative subdirectory path to maintain the structure in the output directory
        rel_path = os.path.relpath(root, input_dir_path)
        out_dir = os.path.join(output_dir_path, rel_path)
        os.makedirs(out_dir, exist_ok=True)

        for f in files:
            if not (f.endswith('.bag') or f.endswith('.bag.active')):
                continue
            # if we put the same output directory as input, we don't want any problems
            if f.endswith('_raw.bag'):
                continue
            
            bag_path = os.path.join(root,f)
            if not contains_compressed_images(bag_path):
                logging.info(f"Omiting {bag_path} as it does not contain compressed images")
                continue
            
            logging.info(f"Decompressing {bag_path} in directory {out_dir}")
            
            start1 = time.time()
            decompress_bag_in_directory(bag_path, out_dir)
            end1 = time.time()
            print(f"Decompression process lasted {end1 - start1:.2f} seconds")

    logging.info(f"Decompression process completed for directory {input_dir_path} to {output_dir_path}")


# --------------------------------------------------------------------------------------------------------


if __name__ == "__main__":
    logging_configuration()
    parser = argparse.ArgumentParser(description="Decompress images according to encoding")
    parser.add_argument('--input_dir', type=str, required=True, help="Directory yyyy_mm_dd with compressed bags")
    parser.add_argument('--output_dir', type=str, required=True, help="Directory where we want to generate the decompressed bags")
    args = parser.parse_args()
    if is_there_bags_with_compressed_images(args.input_dir):
        decompress_all_directory(args.input_dir, args.output_dir)
    else:
        logging.info(f"In this directory there are no compressed images to decompress.")