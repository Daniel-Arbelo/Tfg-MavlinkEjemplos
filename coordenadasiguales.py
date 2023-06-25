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
target_alt = 50  # Altitud en metros

# Obtener la posición actual del dron
connection.mav.request_data_stream_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

# Esperar y recibir mensajes

message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)  # Espera un mensaje de posición global
if message is not None:
    # Acceder a la información de ubicación
    current_lat = message.lat / 1e7  # Latitud en grados
    current_lon = message.lon / 1e7  # Longitud en grados
    current_alt = message.alt / 1e3  # Altitud en metros


# Comprobar si se ha alcanzado la posición de destino
if math.isclose(current_lat, target_lat, abs_tol=0.0001) and \
    math.isclose(current_lon, target_lon, abs_tol=0.0001):
    # Realizar acciones adicionales o enviar comandos adicionales
    # Aquí puedes agregar el código para continuar con la misión
    print('Llegaste al destino')