from pymavlink import mavutil
import sys
import time

connection = mavutil.mavlink_connection('udp:localhost:14550')

msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=10)

if msg:
    print('connection success!')
else:
    print('No recibio heartbeat en 10 segs')
    sys.exit()

# Comando para pausar la misión en curso
connection.mav.command_long_send(
    connection.target_system, connection.target_component,
    184, 0, 1, 0, 0, 0, 0, 0, 0
)

# Comando para cambiar al modo "Hold" en el punto actual
connection.mav.set_mode_send(
    connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4)

time.sleep(3) # Esperar 3 segundos

# Comando para reanudar la misión
connection.mav.command_long_send(
    connection.target_system, connection.target_component,
    184, 0, 0, 0, 0, 0, 0, 0, 0
)