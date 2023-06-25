from pymavlink import mavutil
import sys
import time
import math

connection = mavutil.mavlink_connection('udp:localhost:14550')

msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=10)

if msg:
    print('connection success!')
else:
    print('No recibio heartbeat en 10 segs')
    sys.exit()

# Establecer las coordenadas objetivo
target_lat = 47.3977508  # Latitud del punto objetivo
target_lon = 8.5456074  # Longitud del punto objetivo
target_alt = 100  # Altitud en metros

# Comando para pausar la misión en curso
connection.mav.command_long_send(
    connection.target_system, connection.target_component,
    184, 0, 1, 0, 0, 0, 0, 0, 0
)

# Definir los modos de vuelo deseados
main_mode = mavutil.mavlink.PX4_CUSTOM_MAIN_MODE_AUTO
sub_mode = mavutil.mavlink.PX4_CUSTOM_SUB_MODE_AUTO_LOITER

connection.mav.set_mode_encode(
    connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    main_mode << 4 | sub_mode
)


    

