#!/usr/bin/env python3


import os
import rosbag
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
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


# -------------------------------------------------------------------------------------------------------------------
# this first function will check all files looking for bagfiles with images and will tell us if there are any or not
# -------------------------------------------------------------------------------------------------------------------


def is_there_bags_with_images(dir_path):
    logging.debug(f"Entering this function FLAG 2: {dir_path}")
    for root,_ , files in os.walk(dir_path):
        # we make it go earlier to files containing certain words like ‘camera’ or ‘stereo’ to go faster
        files.sort(key = lambda name: 0 if 'camera' in name or 'stereo' in name else 1)
        
        logging.debug(f"Checking files names {files} in {root} FLAG 3")
        for f in files:
            logging.debug(f"Checking file {f} in {root} FLAG 4")
            if not (f.endswith('.bag') or f.endswith('.bag.active')):
                continue

            bag_file = os.path.join(root,f)
            logging.debug(f"Checking name: {bag_file} for directory {dir_path} FLAG 5")
            try:
                with rosbag.Bag(bag_file, 'r') as bag:
                    for topic, msg, _ in bag.read_messages():
                        if msg._type == 'sensor_msgs/Image' and 'image' in topic.lower():
                            logging.info(f"There is at least one bag with images in the {dir_path} directory: we proceed to compress it")
                            return True
            except Exception as e:
                logging.error(f"Error when reading {bag_file}: {e}")
                continue

    logging.info(f"There is no bag with images in the directory {dir_path}")
    return False


# --------------------------------------------------------------------------------------
# the next set of functions will allow use to compress all images of a desired directory
# --------------------------------------------------------------------------------------


def obtain_number_of_channels(encoding):
    if encoding.startswith('mono') or encoding.startswith('bayer'):
        return 1
    elif encoding in ['rgb8', 'bgr8', 'rgb16', 'bgr16']:
        return 3
    elif encoding in ['rgba8', 'bgra8', 'rgba16', 'bgra16']:
        return 4
    else:
        logging.warning(f"Encoding not recognized {encoding}: is assumed to be 1")
        return 1


def bag_contains_relevant_images(bag_path):
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, _ in bag.read_messages():
                if msg._type == 'sensor_msgs/Image' and 'image' in topic.lower():
                    return True
    except Exception as e:
        logging.error(f"Error when reading {bag_path}: {e}")
    return False


def similar_timestamps(bag, topic_raw, topic_comp, tolerance=1, num_samples=5):
    try:    
        times1 = [t.to_sec() for _, _, t in bag.read_messages(topic_raw)]
        times2 = [t.to_sec() for _, _, t in bag.read_messages(topic_comp)]

        if not times1 or not times2:
            logging.info(f"No timestamps found for {topic_raw} ({len(times1)} timestamps) or {topic_comp} ({len(times2)} timestamps)")
            return False
        for i in range(min(num_samples, len(times1), len(times2))):
            diff = abs(times1[i] - times2[i])
            logging.debug(f"Timestamp diff at index {i}: {diff:.6f}")
            if diff > tolerance:
                logging.info(f"Different timestamps {diff:.6f}s at index {i}")
                return False
        return True
    
    except Exception as e:
        logging.error(f"Error comparing timestamps for {topic_raw} and {topic_comp}: {e}")
        return False    


# Auxiliary function to normalize the format string
def normalize_format(fmt):
    fmt = fmt.lower()
    fmt = fmt.replace('jpeg', 'jpg')
    return fmt


# in this case we will delete the raw topic and keep the compressed one, as the focus of the script is to compress images
def bag_contains_raw_and_compressed(bag_path, desired_format):
    topics_to_delete = []
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            topic_info = bag.get_type_and_topic_info()[1]

            for topic, info in topic_info.items():
                logging.debug(f"Detected topic: {topic}, type: {info.msg_type}")
                if info.msg_type != 'sensor_msgs/Image' or 'image' not in topic.lower():
                    continue

                compressed_topic = topic + '/compressed'

                # we check if the compressed topic associated with the raw one exists
                if compressed_topic not in topic_info:
                    logging.debug(f"Compressed topic {compressed_topic} does not exist for {topic}, skipping")
                    continue

                comp_info = topic_info[compressed_topic]

                # we check that the compressed topic is of type CompressedImage
                if comp_info.msg_type != 'sensor_msgs/CompressedImage':
                    logging.warning(f"Compressed topic {compressed_topic} exists but has unexpetced type: {comp_info.msg_type}")
                    continue

                n_raw = info.message_count
                n_comp = comp_info.message_count

                # we check that the number of messages is similar and that the timestamps are similar
                # this is to avoid deleting topics that contain information that is not in the other one (like an empty topic)
                if abs(n_raw - n_comp) <= 1 and similar_timestamps(bag, topic, compressed_topic):
                    try:
                        msg = next(bag.read_messages(topics=[compressed_topic]))[1]
                        compressed_format = normalize_format(msg.format)

                        if desired_format.lower() in compressed_format:
                            logging.info(f"{compressed_topic} is already in desired format '{desired_format}', will delete {topic}")
                            topics_to_delete.append(topic)
                        else:
                            logging.info(f"{compressed_topic} is not in desired format '{desired_format}', will delete it")
                            topics_to_delete.append(compressed_topic)
                    except Exception as e:
                        logging.warning(f"Could not read compressed message for {compressed_topic}: {e}, so we assume it is not usable and will delete it")
                        # if there is an error, we assume the compressed topic is not valid
                        topics_to_delete.append(compressed_topic)
                    
                else:
                    if n_raw >= n_comp:
                        logging.info(f"{compressed_topic} has fewer messages ({n_comp}) than {topic} ({n_raw}), deleting it")
                        topics_to_delete.append(compressed_topic)
                    elif n_raw < n_comp:
                        logging.info(f"{topic} has fewer messages ({n_raw}) than {compressed_topic} ({n_comp}), deleting it")
                        topics_to_delete.append(topic)
     
    except Exception as e:
        logging.error(f"Error when checking if {bag_path} has both raw and compressed images: {e}")
    
    return topics_to_delete


# this function is like a 'cleaning' process of the bag, deleting redundant topics
def delete_repeated_topic(bag_path, topics_to_delete):
    try:
        if not topics_to_delete:
            logging.info(f"No topics to delete in {bag_path}, skipping cleanup")
            return bag_path
        
        tmp_path = bag_path.replace(".bag", "_temp.bag")

        with rosbag.Bag(bag_path, 'r') as inbag, rosbag.Bag(tmp_path, 'w') as outbag:
            for topic, msg, t in inbag.read_messages():
                if topic in topics_to_delete:
                    continue
                outbag.write(topic, msg, t)

        # we replace the original bag with the cleaned one, and delete the original
        os.replace(tmp_path, bag_path)
        logging.info(f"Bag {bag_path} cleaned, removed repeated topics: {topics_to_delete}")
    
    except Exception as e:
        logging.error(f"Error deleting repeated topics in {bag_path}: {e}")
        
    return bag_path


def compress_bag(input_bag_path, output_dir_path, format='png', quality=1):
    try:
        filename = os.path.basename(input_bag_path)
        if filename.endswith('.bag.active'):
            base_name = filename[:-11]
        elif filename.endswith('.bag'):
            base_name = filename[:-4]
        else:
            base_name = os.path.splitext(filename)[0]
            logging.warning(f"Extension not recognized in {filename}, base will be used: {base_name}")

        output_bag_path = os.path.join(output_dir_path, base_name + '_compressed.bag')

        logging.info(f"Compressing the file: {input_bag_path} into {output_bag_path} using format {format} with quality {quality}")
        
        # filter to ensure that we have compressed something
        modified = False
        # we are going to store in a variable which is the first timestamp with images, then it will be useful for us
        first_timestamp_img = None

        # we check if we have both raw and compressed images in the bag, and prepare the bag for compression
        topics_to_delete = bag_contains_raw_and_compressed(input_bag_path, format)
        
        if topics_to_delete:
            input_bag_path = delete_repeated_topic(input_bag_path, topics_to_delete)

        # we get a set of excluded topics to make a faster process
        excluded_topics = set()
        
        with rosbag.Bag(input_bag_path, 'r') as inbag, rosbag.Bag(output_bag_path, 'w') as outbag:
            for topic, msg, t in inbag.read_messages():
                if topic in excluded_topics:
                    continue

                # we only make the modification for messages of type Image, the rest we rewrite the same
                if msg._type != 'sensor_msgs/Image' or 'image' not in topic.lower():
                    outbag.write(topic, msg, t)
                    continue
                
                # we save the first timestamp with message of type Image
                if first_timestamp_img is None:
                    first_timestamp_img = t.to_sec()

                try:
                    # we have to differentiate 2 cases according to the number of channels.
                    # the process is: bytes(ROS, original message) -> image (with any reshape) -> compressed image -> bytes(ROS, compressed message keeping the original encoding)
                    num_channels = obtain_number_of_channels(msg.encoding)
                    image_np = np.frombuffer(msg.data, dtype=np.uint8)
                    if num_channels == 1:
                        image_np = image_np.reshape((msg.height, msg.width))
                    else:
                        image_np = image_np.reshape((msg.height, msg.width, num_channels))

                    # Note: rgb8, bgr8, rgba8, bgra8 are handled correctly by OpenCV, so there are no problems

                    # logging.debug(f"Image processing in topic {topic} with encoding {msg.encoding}, number of channels {num_channels}")
                    # logging.debug(f"Numpy shape before compression: {image_np.shape}, dtype: {image_np.dtype}")
                    # logging.debug(f"Min/Max pixel values: {image_np.min()}/{image_np.max()}")
                    
                    if format == 'png':
                        compression_param = [cv2.IMWRITE_PNG_COMPRESSION, max(0, min(9, quality))]
                        ext = '.png'
                    
                    elif format == 'jpg':
                        # JPG does not support some encodings, so we have to check them
                        valid_encodings_jpg_mono = ['mono8', '8uc1']

                        if msg.encoding.lower().startswith('bayer'):
                            logging.warning(f"Encoding is {msg.encoding}: Cannot compress Bayer images to JPG in {topic} without losing information. Skiping compression")
                            excluded_topics.add(topic)
                            continue
                        
                        if num_channels == 1 and msg.encoding.lower() not in valid_encodings_jpg_mono:
                            logging.warning(f"Unsuported mono encoding '{msg.encoding}' for JPG compression in {topic}. Skiping compression")
                            excluded_topics.add(topic)
                            continue

                        compression_param = [cv2.IMWRITE_JPEG_QUALITY, max(0, min(100, quality))]
                        ext = '.jpg'
                        
                    success, compressed_data = cv2.imencode(ext, image_np, compression_param)
                    if not success:
                        logging.warning(f"Failed to compress image to {format} in {topic}")
                        continue
                    
                    comp_msg = CompressedImage()
                    comp_msg.header = msg.header
                    # we save here the original encoding in this way
                    comp_msg.format = f"{msg.encoding}; {format} compressed {msg.encoding}"
                    comp_msg.data = compressed_data.tobytes()

                    outbag.write(topic + "/compressed", comp_msg, t)
                    modified = True

                except Exception as e:
                    logging.error(f"Error while processing {topic}: {e}")
        if modified:
            logging.info(f"Compressed file saved in:{output_bag_path}")
            return True, first_timestamp_img, output_bag_path
        # if we have not compressed/modified anything from the original file, we delete the ‘compressed.bag’ file that has been generated
        else:
            if os.path.exists(output_bag_path):
                os.remove(output_bag_path)
            logging.info(f"No relevant images were found in {input_bag_path}, deleted compressed file {output_bag_path}")
            return False, None, None
        
    except Exception as e:
        logging.error(f"Error compressing the file {input_bag_path}: {e}")
        return False, None, None
    

#------------------------------------------------------------------------------------------------------------
# we will create bagfiles with the same same name as the original ones, but with the suffix ‘_compressed.bag’
# and we will create them in the same directory as the original bagfiles
# ------------------------------------------------------------------------------------------------------------


def compress_all_directory(dir_path, output_dir_path, format='png', quality=1):
    for root, _, files in os.walk(dir_path):
        # we calculate the relative subdirectory path to maintain the structure in the output directory
        rel_path = os.path.relpath(root, dir_path)
        out_dir = os.path.join(output_dir_path, rel_path)
        os.makedirs(out_dir, exist_ok=True)

        for f in files:
            if not (f.endswith('.bag') or f.endswith('.bag.active')):
                continue
            # this is to avoid reading the bags that we have just compressed
            if f.endswith('_compressed.bag'):
                continue
            bag_path = os.path.join(root, f)
            if not bag_contains_relevant_images(bag_path):
                logging.info(f"Omiting {bag_path} as it does not contain relevant images")
                continue
            
            start1 = time.time()
            compress_bag(bag_path, out_dir, format, quality)
            end1 = time.time()
            print(f"Compression process lasted {end1 - start1:.2f} seconds")

    logging.info(f"Complete process finished for the directory {dir_path} into the directory {output_dir_path}")


# --------------------------------------------------------------------------------------------------------


if __name__ == "__main__":
    logging_configuration()
    parser = argparse.ArgumentParser(description="Compress images to PNG or JPG with specified quality")
    parser.add_argument('--input_dir', type=str, required=True, help="Directory yyyy_mm_dd with bags to compress")
    parser.add_argument('--output_dir', type=str, required=True, help="Directory where we want to generate the compressed bags")
    parser.add_argument('--format', type=str, choices=['png', 'jpg', 'jpeg'], default='png', help="Compression format: 'png' (lossless) or 'jpg/jpeg' (lossy)")
    parser.add_argument('--quality', type=int, default=9, help="Compression level: 0 (low) to 9 (high) for PNG, or 0 (low) to 100 (high) for JPEG")
    args = parser.parse_args()

    # we convert 'jpeg' to 'jpg' for consistency
    if args.format.lower() == "jpeg":
        args.format = "jpg"
    
    logging.debug(f"Starting compression process for directory {args.input_dir} FLAG 1")
    if is_there_bags_with_images(args.input_dir):
        compress_all_directory(args.input_dir, args.output_dir, args.format, args.quality)
    else:
        logging.info(f"In this directory there are no images in {args.input_dir} to compress.")
    
    logging.info(f"Finished compressing images in {args.input_dir} using format {args.format} and quality {args.quality}.")