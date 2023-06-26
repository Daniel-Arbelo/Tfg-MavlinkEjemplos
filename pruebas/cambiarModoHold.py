from pymavlink import mavutil
import sys

connection = mavutil.mavlink_connection('udp:localhost:14550')

msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=10)

if msg:
    print('connection success!')
else:
    print('No recibio heartbeat en 10 segs')
    sys.exit()

# Comando para cancelar la misión en curso
connection.mav.command_long_send(
    connection.target_system, connection.target_component,
    45, 0, 0, 0, 0, 0, 0, 0, 0
)

# Esto hace que llegue al siguiente punto y se quede en hold
connection.mav.command_long_send(
    connection.target_system, connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    2,
    0, 0, 0, 0, 0, 0
)