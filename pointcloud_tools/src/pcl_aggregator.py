#!/usr/bin/env python3

import rospy
import tf2_ros
import ros_numpy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from ros_numpy import numpify # Importamos numpify para convertir la TF a matriz

class OnlineAggregator:
    def __init__(self):
        rospy.init_node('online_cloud_aggregator')

        # --- Parámetros ---
        self.target_frame = rospy.get_param('~world_frame', 'world_ned')
        
        # CAMBIO: Definimos explícitamente el frame base del robot
        self.base_frame = rospy.get_param('~robot_base_frame', 'sparus2/base_link')
        
        self.save_path = rospy.get_param('~output_file', '/tmp/full_cloud.ply')
        self.voxel_size = rospy.get_param('~voxel_size', 0.05)
        self.save_every_n = rospy.get_param('~auto_save_n_msgs', 0) 

        # Buffers
        # Aumentamos buffer para permitir busquedas en el pasado si el bag va rapido
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.global_map = o3d.geometry.PointCloud()
        self.msg_count = 0

        # Subscriber
        input_topic = rospy.get_param('~input_topic', '/stereo/points2')
        # Aumentamos queue_size para no perder mensajes si el procesamiento se retrasa
        self.sub = rospy.Subscriber(input_topic, PointCloud2, self.callback, queue_size=10)

        rospy.loginfo(f"Agregador iniciado.")
        rospy.loginfo(f"Topic entrada: {input_topic}")
        rospy.loginfo(f"Transformación: {self.base_frame} -> {self.target_frame}")

    def callback(self, cloud_msg):
        try:
            # 1. SOLUCIÓN A HUECOS: Esperar la transformación
            # Buscamos la transformación desde el BASE LINK (no la cámara) al MUNDO
            if not self.tf_buffer.can_transform(self.target_frame, self.base_frame, cloud_msg.header.stamp, rospy.Duration(0.5)):
                # Si falla, advertencia suave y saltamos
                return 

            # 2. Obtener Transformación (World <--- Base Link)
            # Forzamos el uso de self.base_frame en lugar de cloud_msg.header.frame_id
            trans_stamped = self.tf_buffer.lookup_transform(
                self.target_frame, 
                self.base_frame, 
                cloud_msg.header.stamp, 
                rospy.Duration(0.1)
            )

            # 3. Convertir Mensaje ROS a Open3D
            pc = ros_numpy.point_cloud2.pointcloud2_to_array(cloud_msg)
            
            # Filtrar NaNs
            mask = np.isfinite(pc['x']) & np.isfinite(pc['y']) & np.isfinite(pc['z'])
            pc = pc[mask]
            
            if len(pc) == 0: return

            # Extraer XYZ
            points = np.column_stack((pc['x'], pc['y'], pc['z'])).astype(np.float64)
            new_pcd = o3d.geometry.PointCloud()
            new_pcd.points = o3d.utility.Vector3dVector(points)

            # 4. Aplicar Transformación Manualmente
            # Convertimos la TF de ROS a Matriz 4x4 de Numpy
            transform_matrix = numpify(trans_stamped.transform)
            
            # Transformamos la nube: P_world = T_world_base * P_local
            new_pcd.transform(transform_matrix)

            # 5. Acumular y Submuestrear
            self.global_map += new_pcd
            
            # Mantenimiento periódico del mapa (cada 20 scans) para velocidad
            if self.msg_count % 20 == 0: 
                self.global_map = self.global_map.voxel_down_sample(self.voxel_size)

            self.msg_count += 1
            
            # Log menos frecuente para no saturar consola
            if self.msg_count % 10 == 0:
                rospy.loginfo(f"Procesados: {self.msg_count} | Puntos acumulados: {len(self.global_map.points)}")

            # Guardado intermedio
            if self.save_every_n > 0 and self.msg_count % self.save_every_n == 0:
                self.save_map()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            pass # Ignoramos fallos puntuales de TF

    def save_map(self):
        if len(self.global_map.points) > 0:
            rospy.loginfo(f"Guardando nube ({len(self.global_map.points)} pts) en {self.save_path}...")
            o3d.io.write_point_cloud(self.save_path, self.global_map, write_ascii=True)
        else:
            rospy.logwarn("Nube vacía, no se guardó nada.")

    def run(self):
        rospy.spin()
        self.save_map()

if __name__ == '__main__':
    node = OnlineAggregator()
    node.run()