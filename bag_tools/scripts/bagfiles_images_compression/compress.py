#!/usr/bin/env python3


import os
import rosbag
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import logging
import sys
import argparse


def configurar_logging():
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


def hay_bags_con_imagenes(dir_path):
    for root,_ , files in os.walk(dir_path):
        # we make it go earlier to files containing certain words like ‘camera’ or ‘stereo’ to go faster
        files.sort(key = lambda name: 0 if 'camera' in name or 'stereo' in name else 1)
        
        for f in files:
            if not (f.endswith('.bag') or f.endswith('.bag.active')):
                continue

            bag_file = os.path.join(root,f)
            try:
                with rosbag.Bag(bag_file, 'r') as bag:
                    for _, msg, _ in bag.read_messages():
                        if msg._type == 'sensor_msgs/Image':
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


def obtener_numero_canales(encoding):
    if encoding.startswith('mono') or encoding.startswith('bayer'):
        return 1
    elif encoding in ['rgb8', 'bgr8', 'rgb16', 'bgr16']:
        return 3
    elif encoding in ['rgba8', 'bgra8', 'rgba16', 'bgra16']:
        return 4
    else:
        logging.warning(f"Encoding not recognized {encoding}: is assumed to be 1")
        return 1


def bag_contiene_imagen_relevante(bag_path):
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            for _, msg, _ in bag.read_messages():
                if msg._type == 'sensor_msgs/Image':
                    return True
    except Exception as e:
        logging.error(f"Error when reading {bag_path}: {e}")
    return False


def comprimir_bag(input_bag_path, format='png', quality=9):
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
        modificado = False
        # we are going to store in a variable which is the first timestamp with images, then it will be useful for us
        primer_timestamp_img = None

        # we get a set of excluded topics to make a faster process
        excluded_topics = set()
        
        with rosbag.Bag(input_bag_path, 'r') as inbag, rosbag.Bag(output_bag_path, 'w') as outbag:
            for topic, msg, t in inbag.read_messages():
                if topic in excluded_topics:
                    continue

                # we only make the modification for messages of type Image, the rest we rewrite the same
                if msg._type != 'sensor_msgs/Image':
                    outbag.write(topic, msg, t)
                    continue
                
                # we save the first timestamp with message of type Image
                if primer_timestamp_img is None:
                    primer_timestamp_img = t.to_sec()

                try:
                    # we have to differentiate 2 cases according to the number of channels.
                    # the process is: bytes(ROS, original message) -> image (with any reshape) -> compressed image -> bytes(ROS, compressed message keeping the original encoding)
                    num_canales = obtener_numero_canales(msg.encoding)
                    image_np = np.frombuffer(msg.data, dtype=np.uint8)
                    if num_canales == 1:
                        image_np = image_np.reshape((msg.height, msg.width))
                    else:
                        image_np = image_np.reshape((msg.height, msg.width, num_canales))

                    logging.debug(f"Image processing in topic {topic} with encoding {msg.encoding}, number of channels {num_canales}")
                    logging.debug(f"Numpy shape before compression: {image_np.shape}, dtype: {image_np.dtype}")
                    logging.debug(f"Min/Max pixel values: {image_np.min()}/{image_np.max()}")
                    
                    if format == 'png':
                        compression_param = [cv2.IMWRITE_PNG_COMPRESSION, max(0, min(9, quality))]
                        ext = '.png'
                    
                    elif format == 'jpg':
                        # JPG does not support some encodings, so we have to check them
                        encodings_validos_jpg_mono = ['mono8', '8uc1']

                        if msg.encoding.lower().startswith('bayer'):
                            logging.warning(f"Encoding is {msg.encoding}: Cannot compress Bayer images to JPG in {topic} without losing inormation. Skiping compression")
                            excluded_topics.add(topic)
                            continue
                        
                        if num_canales == 1 and msg.encoding.lower() not in encodings_validos_jpg_mono:
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
                    modificado = True

                except Exception as e:
                    logging.error(f"Error while processing {topic}: {e}")
        if modificado:
            logging.info(f"Compressed file saved in:{output_bag_path}")
            return True, primer_timestamp_img, output_bag_path
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


def comprimir_todo_el_directorio(dir_path, format='png', quality=9):
    for root, _, files in os.walk(dir_path):
        for f in files:
            if not (f.endswith('.bag') or f.endswith('.bag.active')):
                continue
            # this is to avoid reading the bags that we have just compressed
            if f.endswith('_compressed.bag'):
                continue
            bag_path = os.path.join(root, f)
            if not bag_contiene_imagen_relevante(bag_path):
                logging.info(f"Omiting {bag_path} as it does not contain relevant images")
                continue
            
            comprimir_bag(bag_path, format, quality)


    logging.info(f"Complete process finished for the directory {dir_path}")


# --------------------------------------------------------------------------------------------------------


if __name__ == "__main__":
    configurar_logging()
    parser = argparse.ArgumentParser(description="Compress images to PNG or JPG with specified quality")
    parser.add_argument('--input_dir', type=str, required=True, help="Directory yyyy_mm_dd with bags to compress")
    parser.add_argument('--format', type=str, choices=['png', 'jpg'], default='png', help="Compression format: 'png' (lossless) or 'jpg' (lossy)")
    parser.add_argument('--quality', type=int, default=9, help="Compression level: 0 (low) to 9 (high) for PNG, or 0 (low) to 100 (high) for JPEG")
    args = parser.parse_args()
    
    if hay_bags_con_imagenes(args.input_dir):
        comprimir_todo_el_directorio(args.input_dir, args.format, args.quality)
    else:
        logging.info(f"In this directory there are no images in {args.input_dir} to compress.")
    
    logging.info(f"Finished compressing images in {args.input_dir} using format {args.format} and quality {args.quality}.")