#!/usr/bin/env python3
import rospy
import subprocess
import glob
import sys
import os
import signal
import time

# Variable global para el subproceso actual
player_process = None
stop_requested = False

def signal_handler(sig, frame):
    # Manejar el cierre para matar a rosbag si cerramos el launch
    global player_process, stop_requested
    rospy.logwarn("Señal de parada recibida. Terminando reproducción...")
    stop_requested = True
    if player_process:
        player_process.terminate()
    sys.exit(0)

def main():
    global player_process, stop_requested
    rospy.init_node('folder_player', anonymous=True)
    
    # Capturar señales de cierre (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 1. Obtener Argumentos desde el Launchfile
    if len(sys.argv) < 4:
        rospy.logerr("Uso: play_folder.py <carpeta> <rate> <start_time>")
        return

    folder = sys.argv[1]
    rate = sys.argv[2]
    start_time = sys.argv[3]

    # 2. Buscar archivos .bag
    search_path = os.path.join(folder, "*.bag")
    # Ordenamos alfabéticamente para asegurar el orden secuencial
    bag_files = sorted(glob.glob(search_path))

    if not bag_files:
        rospy.logerr(f"No se encontraron archivos .bag en: {search_path}")
        return

    rospy.loginfo(f"Encontrados {len(bag_files)} archivos. Reproduciendo secuencialmente...")

    # 3. BUCLE DE REPRODUCCIÓN SECUENCIAL
    for i, bag_file in enumerate(bag_files):
        if stop_requested or rospy.is_shutdown():
            break

        filename = os.path.basename(bag_file)
        rospy.loginfo(f"[{i+1}/{len(bag_files)}] Reproduciendo: {filename}")

        # Construir el comando para UN solo archivo
        command = [
            "rosbag", "play", 
            "--clock", 
            "-r", str(rate), 
            "-s", str(start_time),
            bag_file  # Solo pasamos el archivo actual
        ]

        # 4. Ejecutar y Esperar
        try:
            # Lanzamos el proceso
            player_process = subprocess.Popen(command)
            
            # Esperamos a que termine este bag antes de ir al siguiente
            # wait() bloqueará el script hasta que rosbag termine su reproducción
            player_process.wait()
            
            # Pequeña pausa entre bags para dar tiempo a limpiar buffers si es necesario
            if not stop_requested:
                time.sleep(1.0) 

        except Exception as e:
            rospy.logerr(f"Error ejecutando rosbag para {filename}: {e}")
            break

    rospy.loginfo("Reproducción de carpeta finalizada.")

if __name__ == '__main__':
    main()