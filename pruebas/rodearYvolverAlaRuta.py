from pymavlink import mavutil
import math
import time

# Establecer las coordenadas objetivo
target_lat = 47.3977508  # Latitud del punto objetivo
target_lon = 8.5456074  # Longitud del punto objetivo
target_alt = 50  # Altitud en metros

# Configurar la distancia y velocidad de la ruta circular
radius = 50  # Radio de la ruta circular en metros
angular_speed = 20  # Velocidad angular en grados por segundo

# Establecer la conexión con el vehículo
connection = mavutil.mavlink_connection('udp:localhost:14550')

msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=10)

if msg:
    print('connection success!')
else:
    print('No recibio heartbeat en 10 segs')
    sys.exit()

# Solicitar la posición actual del vehículo
connection.mav.request_data_stream_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

# Esperar la respuesta y obtener la posición actual
while True:
    message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if message:
        current_lat = message.lat / 1e7  # Latitud actual en grados decimales
        current_lon = message.lon / 1e7  # Longitud actual en grados decimales
        current_alt = message.alt / 1000  # Altitud actual en metros
        break

# Calcular el ángulo inicial hacia el punto objetivo
delta_lat = target_lat - current_lat
delta_lon = target_lon - current_lon
initial_bearing = math.atan2(delta_lon, delta_lat)
initial_bearing_deg = math.degrees(initial_bearing)

# Calcular el tiempo requerido para completar la ruta circular
circle_distance = 2 * math.pi * radius
circle_duration = circle_distance / angular_speed

# Iniciar el movimiento circular alrededor del punto objetivo
start_time = time.time()
while time.time() - start_time < circle_duration:
    elapsed_time = time.time() - start_time
    bearing = math.radians(initial_bearing_deg + angular_speed * elapsed_time)
    next_lat = target_lat + (radius / 111111) * math.cos(bearing)
    next_lon = target_lon + (radius / (111111 * math.cos(math.radians(target_lat)))) * math.sin(bearing)

    # Enviar comandos al vehículo para moverse hacia las coordenadas actuales
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
        next_lat * 1e7, next_lon * 1e7, target_alt
    )
    time.sleep(0.1)  # Esperar un breve período de tiempo entre comandos

# Volver a la ruta original
connection.mav.command_long_send(
    connection.target_system, connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
    target_lat * 1e7, target_lon * 1e7, target_alt
)
