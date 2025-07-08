#!/usr/bin/env python3


import os
import rosbag
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import logging
import sys
import argparse
from decompress import descomprimir_bag_en_directorio
from cv_bridge import CvBridge
from collections import defaultdict
import tempfile
import random


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


# ---------------------------------------------------------------------------
# in this section there will be the functions that will compress the bagfiles
# ---------------------------------------------------------------------------


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


def comprimir_bag(input_bag_path):
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

        with rosbag.Bag(input_bag_path, 'r') as inbag, rosbag.Bag(output_bag_path, 'w') as outbag:
            for topic, msg, t in inbag.read_messages():
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
                    
                    success, compressed_data = cv2.imencode('.png', image_np, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                    if not success:
                        logging.warning(f"Failed to compress image in {topic}")
                        continue

                    comp_msg = CompressedImage()
                    comp_msg.header = msg.header
                    # we save here the original encoding in this way
                    comp_msg.format = f"{msg.encoding}; png compressed {msg.encoding}"
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


# --------------------------------------------------------------------------------------------------------------------------
# this section here will be in charge of verifying that the compression has gone well, and if so, to delete the original bag
# --------------------------------------------------------------------------------------------------------------------------

bridge = CvBridge()


def comparar_n_mensajes(path1, path2):
    def contar_mensajes(path):
        counts = defaultdict(int)
        with rosbag.Bag(path, 'r') as bag:
            for topic, msg, _ in bag.read_messages():
                # this is because String messages sometimes create problems, and here are not relevant, so we skip them
                if msg._type == 'std_msgs/String':
                    continue
                counts[topic] += 1
        return counts
        
    counts1 = contar_mensajes(path1)
    counts2 = contar_mensajes(path2)

    todos_ok = True
    for topic in sorted(set(counts1.keys()).union(counts2.keys())):
        n1 = counts1.get(topic, 0)
        n2 = counts2.get(topic, 0)
        if n1 != n2:
            logging.warning(f"Difference in number of messages for '{topic}': original={n1}, compressed={n2}")
            todos_ok = False
        else:
            logging.info(f"Same number of messages for {topic}: {n1} messages")
    return todos_ok


# function that allows us to compare how “similar” 2 images are
def calcular_mse(img1, img2):
    if img1.shape != img2.shape:
        return float('inf')
    err = np.mean((img1.astype("float") - img2.astype("float")) ** 2)
    return err


# in this case we have chosen 5 images, but we could put another number.
def comparar_imagenes_random(path1, path2, top_n=5):
    all_ok = True
    with rosbag.Bag(path1, 'r') as bag1:
        imagen_topics = [topic for topic,info in bag1.get_type_and_topic_info()[1].items() if info.msg_type == 'sensor_msgs/Image']
    
    for topic in imagen_topics:
        with rosbag.Bag(path1, 'r') as b1:
            msgs1 = [(m, t) for _, m, t in b1.read_messages(topic)]
        with rosbag.Bag(path2, 'r') as b2:
            msgs2 = [(m, t) for _, m, t in b2.read_messages(topic)]

        if len(msgs1) != len(msgs2) or len(msgs1) == 0:
            logging.warning(f"You cannot compare images in {[topic]} (n_msgs={len(msgs1)} vs {len(msgs2)})")
            all_ok = False
            continue

        indices = random.sample(range(len(msgs1)), min(top_n, len(msgs1)))
        for idx in indices:
            m1, _ = msgs1[idx]
            m2, _ = msgs2[idx]
            try:
                cv1_image = bridge.imgmsg_to_cv2(m1, desired_encoding = 'passthrough')
                cv2_image = bridge.imgmsg_to_cv2(m2, desired_encoding = 'passthrough')
                mse = calcular_mse(cv1_image, cv2_image)
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


def recortar_bag(bag_path, inicio_seg, duracion_max=30):
    with rosbag.Bag(bag_path, 'r') as bag:
        final = bag.get_end_time()
        # if it lasts less than the tolerance, we return the original one
        if final - inicio_seg <= duracion_max:
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

    output_filename = f"{base_name}_recorte_{duracion_max}s.bag"
    path_salida = os.path.join(tmpdir, output_filename)

    with rosbag.Bag(bag_path, 'r') as inbag, rosbag.Bag(path_salida, 'w') as outbag:
        start_time = inicio_seg
        end_time = start_time + duracion_max
        for topic, msg, t in inbag.read_messages():
            if start_time <= t.to_sec() <= end_time:
                outbag.write(topic, msg, t)

    return path_salida


def verificar_y_eliminar(bag_path, inicio_seg, compressed_bag_path):
    logging.info(f"Starting verification for: {bag_path}")

    # creates and uses a temporary directory that deletes itself
    with tempfile.TemporaryDirectory(prefix = "verification_tmp_") as tmp_dir:
        
        exito, path_descomprimido = descomprimir_bag_en_directorio(compressed_bag_path, tmp_dir)

        if not exito or not path_descomprimido:
            logging.warning(f"Decompression to verify has not been successful for {compressed_bag_path}")
            return
        
        if not os.path.exists(path_descomprimido):
            logging.warning(f"The decompressed path corresponding to {compressed_bag_path} was not found")
            return
        
        try:
            original_recortado = recortar_bag(bag_path, inicio_seg)
            descomprimido_recortado = recortar_bag(path_descomprimido, inicio_seg)

            ok1 = comparar_n_mensajes(original_recortado, descomprimido_recortado)
            ok2 = comparar_imagenes_random(original_recortado, descomprimido_recortado)

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
    if not hay_bags_con_imagenes(dir_path):
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
            if not bag_contiene_imagen_relevante(bag_path):
                logging.info(f"Omiting {bag_path} as it does not contain relevant images")
                continue

            exito, t0, compressed_bag_path = comprimir_bag(bag_path)
            if exito and t0 and compressed_bag_path:
                verificar_y_eliminar(bag_path, t0, compressed_bag_path)

    logging.info(f"Complete process finished for the directory {dir_path}")


# ---------------------------------------------------------------------------------------------------------------------------


if __name__ == "__main__":
    configurar_logging()
    parser = argparse.ArgumentParser(description="Compress RAW images to PNG, verify and elminate original bags preserving encoding")
    parser.add_argument('--input_dir', type=str, required=True, help="Directory yyyy_mm_dd with original bags")
    args = parser.parse_args()
    main(args.input_dir)