from pymavlink import mavutil
import sys
import time
import math

# Establecer las coordenadas objetivo
target_lat = 47.3977508  # Latitud del punto objetivo
target_lon = 8.5456074  # Longitud del punto objetivo
target_alt = 50  # Altitud en metros

# Configurar los parámetros del círculo
radius = 50  # Radio del círculo en metros
angular_speed = 20  # Velocidad angular en grados por segundo

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

# Solicitar coordenadas actuales
connection.mav.request_data_stream_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

# Esperar y recibir mensajes

message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)  # Espera un mensaje de posición global
if message is not None:
    # Acceder a la información de ubicación
    current_lat = message.lat / 1e7  # Latitud actual en grados decimales
    current_lon = message.lon / 1e7  # Longitud actual en grados decimales
    current_alt = message.alt / 1000  # Altitud actual en metros


connection.mav.command_long_send(
    target_system=connection.target_system,
    target_component=connection.target_component,
    command=20,  # MAV_CMD_NAV_WAYPOINT
    confirmation=0,  # Comandante autoaceptado
    param1=0,  # Ignorar el parámetro 1
    param2=0,  # Ignorar el parámetro 2
    param3=0,  # Ignorar el parámetro 3
    param4=0,  # Ignorar el parámetro 4
    param5=target_lat,  # Latitud
    param6=target_lon,  # Longitud
    param7=target_alt  # Altitud
)

# Esperar hasta que se alcancen las coordenadas de destino
while True:
    # Obtener la posición actual del dron
    current_lat = connection.messages['GLOBAL_POSITION_INT'].lat / 1e7
    current_lon = connection.messages['GLOBAL_POSITION_INT'].lon / 1e7
    current_alt = connection.messages['GLOBAL_POSITION_INT'].alt / 1000.0

    # Comprobar si se ha alcanzado la posición de destino
    if math.isclose(current_lat, target_lat, abs_tol=0.0001) and \
       math.isclose(current_lon, target_lon, abs_tol=0.0001) and \
       math.isclose(current_alt, target_alt, abs_tol=0.1):
        # Realizar acciones adicionales o enviar comandos adicionales
        # Aquí puedes agregar el código para continuar con la misión
        print('Llegaste al destino')
        # Después de llegar a las coordenadas de destino
        break  # Salir del bucle si se ha alcanzado el destino

    # Esperar un tiempo antes de verificar nuevamente
    time.sleep(1)

# Comando para reanudar la misión
connection.mav.command_long_send(
    connection.target_system, connection.target_component,
    184, 0, 0, 0, 0, 0, 0, 0, 0
)
