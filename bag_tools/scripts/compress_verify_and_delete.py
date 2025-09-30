#!/usr/bin/env python3


import os
import rosbag
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import logging
import sys
import argparse
from decompress_images import decompress_bag_in_directory
from cv_bridge import CvBridge
from collections import defaultdict
import tempfile
import random
import time

start_total = time.time()

def logging_configuration():
    logging.basicConfig(
        level=logging.DEBUG,
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
    for root,_ , files in os.walk(dir_path):
        # we make it go earlier to files containing certain words like ‘camera’ or ‘stereo’ to go faster
        files.sort(key = lambda name: 0 if 'camera' in name or 'stereo' in name else 1)
        
        for f in files:
            if not (f.endswith('.bag') or f.endswith('.bag.active')):
                continue

            bag_file = os.path.join(root,f)
            try:
                with rosbag.Bag(bag_file, 'r') as bag:
                    for topic , msg, _ in bag.read_messages():
                        if msg._type == 'sensor_msgs/Image' and 'image' in topic.lower():
                            logging.info(f"There is at least one bag with images in the {dir_path} directory: we proceed to compress it")
                            return True
            except Exception as e:
                logging.error(f"Error when reading {bag_file}: {e}")
                continue

    logging.info(f"There is no bag with images in the directory {dir_path}")
    return False


# ---------------------------------------------------------------------------
# in this section there will be the functions that will compress the bagfiles
# ---------------------------------------------------------------------------


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


def bag_contains_raw_and_compressed(bag_path):
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
                    logging.info(f"We'll delete {compressed_topic} as it is redundant with {topic}")
                    topics_to_delete.append(compressed_topic)

                # if we have different number of messages, we keep the topic with the most messages
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


def compress_bag(input_bag_path):
    try:
        filename = os.path.basename(input_bag_path)
        if filename.endswith('.bag.active'):
            base_name = filename[:-11]
        elif filename.endswith('.bag'):
            base_name = filename[:-4]
        else:
            base_name = os.path.splitext(filename)[0]
            logging.warning(f"Extension not recognized in {filename}, base will be used: {base_name}")
        output_bag_path = os.path.join(os.path.dirname(input_bag_path), base_name + '_compressed.bag')

        logging.info(f"Compressing the file: {input_bag_path}")
        
        # a filter to ensure that we have compressed something
        modified = False
        # we are going to store in a variable which is the first timestamp with images, then it will be useful for us
        first_timestamp_img = None

        # we check if we have both raw and compressed images in the bag
        topics_to_delete = bag_contains_raw_and_compressed(input_bag_path)
        
        if topics_to_delete:
            input_bag_path = delete_repeated_topic(input_bag_path, topics_to_delete)
        
        else:
            logging.info(f"No redundant compressed topics found in {input_bag_path}, skipping clean up step")

        with rosbag.Bag(input_bag_path, 'r') as inbag, rosbag.Bag(output_bag_path, 'w') as outbag:
        
            for topic, msg, t in inbag.read_messages():
                
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

                    # logging.debug(f"Image processing in topic {topic} with encoding {msg.encoding}, number of channels {num_channels}")
                    # logging.debug(f"Numpy shape before compression: {image_np.shape}, dtype: {image_np.dtype}")
                    # logging.debug(f"Min/Max pixel values: {image_np.min()}/{image_np.max()}")
                    
                    success, compressed_data = cv2.imencode('.png', image_np, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                    if not success:
                        logging.warning(f"Failed to compress image in {topic}")
                        continue

                    comp_msg = CompressedImage()
                    comp_msg.header = msg.header
                    # we save here the original encoding in this way
                    comp_msg.format = f"{msg.encoding}; png compressed {msg.encoding}"
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


# --------------------------------------------------------------------------------------------------------------------------
# this section here will be in charge of verifying that the compression has gone well, and if so, to delete the original bag
# --------------------------------------------------------------------------------------------------------------------------

bridge = CvBridge()


def compare_n_messages(path1, path2):
    def count_messages(path):
        counts = defaultdict(int)
        with rosbag.Bag(path, 'r') as bag:
            for topic, msg, _ in bag.read_messages():
                # this is because String messages sometimes create problems, and here are not relevant, so we skip them
                if msg._type == 'std_msgs/String':
                    continue
                counts[topic] += 1
        return counts
        
    counts1 = count_messages(path1)
    counts2 = count_messages(path2)

    all_ok = True
    for topic in sorted(set(counts1.keys()).union(counts2.keys())):
        n1 = counts1.get(topic, 0)
        n2 = counts2.get(topic, 0)
        if n1 != n2:
            logging.warning(f"Difference in number of messages for '{topic}': original={n1}, compressed={n2}")
            all_ok = False
        else:
            logging.info(f"Same number of messages for {topic}: {n1} messages")
    return all_ok


# function that allows us to compare how “similar” 2 images are
def calculate_mse(img1, img2):
    if img1.shape != img2.shape:
        return float('inf')
    err = np.mean((img1.astype("float") - img2.astype("float")) ** 2)
    return err


# in this case we have chosen 5 images, but we could put another number.
def compare_random_images(path1, path2, top_n=5):
    all_ok = True
    with rosbag.Bag(path1, 'r') as bag1:
        image_topics = [topic for topic,info in bag1.get_type_and_topic_info()[1].items() if (info.msg_type == 'sensor_msgs/Image' and 'image' in topic.lower())]
    
    for topic in image_topics:
        with rosbag.Bag(path1, 'r') as b1:
            msgs1 = [(m, t) for _, m, t in b1.read_messages(topic)]
        with rosbag.Bag(path2, 'r') as b2:
            msgs2 = [(m, t) for _, m, t in b2.read_messages(topic)]

        if len(msgs1) != len(msgs2) or len(msgs1) == 0:
            logging.warning(f"You cannot compare images in {[topic]} (n_msgs={len(msgs1)} vs {len(msgs2)})")
            all_ok = False
            continue

        indexes = random.sample(range(len(msgs1)), min(top_n, len(msgs1)))
        for idx in indexes:
            m1, _ = msgs1[idx]
            m2, _ = msgs2[idx]
            try:
                cv1_image = bridge.imgmsg_to_cv2(m1, desired_encoding = 'passthrough')
                cv2_image = bridge.imgmsg_to_cv2(m2, desired_encoding = 'passthrough')
                mse = calculate_mse(cv1_image, cv2_image)
                # we place this numerical tolerance in case of small computational errors
                if mse < 1e-6:
                    logging.info(f"Identical image in {topic}")
                else:
                    logging.info(f"Different image in {topic}")
                    all_ok = False
            except Exception as e:
                logging.error(f"Error comparing images in {topic}: {e}")
                all_ok = False

    return all_ok


def cut_bag(bag_path, beggining_sec, max_duration=30):
    with rosbag.Bag(bag_path, 'r') as bag:
        final = bag.get_end_time()
        # if it lasts less than the tolerance, we return the original one
        if final - beggining_sec <= max_duration:
            logging.info(f"The piece of bag with images has a shorter duration than the tolerance.")
            return bag_path
        
    tmpdir = tempfile.gettempdir()

    input_filename = os.path.basename(bag_path)
    if input_filename.endswith(".bag.active"):
        base_name = input_filename[:-11]
    elif input_filename.endswith("_compressed.bag"):
        base_name = input_filename[:-15]
    elif input_filename.endswith(".bag"):
        base_name = input_filename[:-4]
    else:
        base_name = os.path.splitext(input_filename)[0]

    output_filename = f"{base_name}_recorte_{max_duration}s.bag"
    exit_path = os.path.join(tmpdir, output_filename)

    with rosbag.Bag(bag_path, 'r') as inbag, rosbag.Bag(exit_path, 'w') as outbag:
        start_time = beggining_sec
        end_time = start_time + max_duration
        for topic, msg, t in inbag.read_messages():
            if start_time <= t.to_sec() <= end_time:
                outbag.write(topic, msg, t)

    return exit_path


def verify_and_delete(bag_path, beggining_sec, compressed_bag_path):
    logging.info(f"Starting verification for: {bag_path}")

    # creates and uses a temporary directory that deletes itself
    with tempfile.TemporaryDirectory(prefix = "verification_tmp_") as tmp_dir:
        
        success, decompressed_path = decompress_bag_in_directory(compressed_bag_path, tmp_dir)

        if not success or not decompressed_path:
            logging.warning(f"Decompression to verify has not been successful for {compressed_bag_path}")
            return
        
        if not os.path.exists(decompressed_path):
            logging.warning(f"The decompressed path corresponding to {compressed_bag_path} was not found")
            return
        
        try:
            start3 = time.time()
            original_cutted = cut_bag(bag_path, beggining_sec)
            decompressed_cutted = cut_bag(decompressed_path, beggining_sec)
            end3 = time.time()
            print(f"Cutting process {end3 - start3:.2f} seconds")

            start4 = time.time()
            ok1 = compare_n_messages(original_cutted, decompressed_cutted)
            end4 = time.time()
            print(f"Message comparison process lasted {end4 - start4:.2f} seconds")

            start5 = time.time()
            ok2 = compare_random_images(original_cutted, decompressed_cutted)
            end5 = time.time()
            print(f"Image comparison process lasted {end5 - start5:.2f} seconds")

            if ok1 and ok2:
                logging.info(f"Successful verification. Deleting original: {bag_path}")
                try:
                    os.remove(bag_path)
                except Exception as e:
                    logging.error(f"Error deleting {bag_path}: {e}")
            else:
                logging.warning(f"Failed to verify {bag_path}. It is not deleted")
        
        except Exception as e:
            logging.error(f"Error during verification process for {bag_path}")
    
    logging.info("Temporary directory deleted")


# -------------------------------------------------------------
# in this section is the main function that controls the script
# -------------------------------------------------------------


def main(dir_path):
    if not is_there_bags_with_images(dir_path):
        logging.info(f"Nothing to compress in this directory {dir_path}")
        return
    
    for root, _, files in os.walk(dir_path):
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
            success, t0, compressed_bag_path = compress_bag(bag_path)
            end1 = time.time()
            print(f"Compression process lasted {end1 - start1:.2f} seconds")
            if success and t0 and compressed_bag_path:
                start2 = time.time()
                verify_and_delete(bag_path, t0, compressed_bag_path)
                end2 = time.time()
                print(f"Verification and elimination process lasted {end2 - start2:.2f} seconds")

    logging.info(f"Complete process finished for the directory {dir_path}")


# ---------------------------------------------------------------------------------------------------------------------------


if __name__ == "__main__":
    logging_configuration()
    parser = argparse.ArgumentParser(description="Compress RAW images to PNG, verify and elminate original bags preserving encoding")
    parser.add_argument('--input_dir', type=str, required=True, help="Directory yyyy_mm_dd with original bags")
    args = parser.parse_args()
    main(args.input_dir)