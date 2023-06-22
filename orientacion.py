from pymavlink import mavutil
import sys

connection = mavutil.mavlink_connection('udp:localhost:14550')

msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=10)

if msg:
    print('connection success!')
else:
    print('No recibio heartbeat en 10 segs')

# Envía una solicitud de la orientación del dron
connection.mav.request_data_stream_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,  # Frecuencia de actualización en Hz
    1  # Habilitar la transmisión
)

# Espera hasta recibir la respuesta de la orientación 1 segundo como máximo
msg = connection.recv_match(type='ATTITUDE', blocking=True, timeout=1)
if msg:
    print('Orientación del dron:')
    print('Roll:', msg.roll)
    print('Pitch:', msg.pitch)
    print('Yaw:', msg.yaw)