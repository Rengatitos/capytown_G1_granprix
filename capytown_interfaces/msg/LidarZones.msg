# Distancias procesadas del LiDAR MS200, agrupadas por zona angular
# relativa al frente del robot. Publicado por lidar_processor_node.
# Todas las distancias en metros. Si una zona no tiene lecturas validas
# dentro de rango, el campo *_valid correspondiente es false y la
# distancia se reporta como +inf (no debe usarse esa lectura).

std_msgs/Header header

float32 front          # Zona frontal: pared u obstaculo al frente
float32 right_front    # S1: sensor derecho delantero (paralelismo, metodo de 2 puntos)
float32 right          # Zona lateral derecha (distancia a la pared derecha)
float32 right_rear     # S2: sensor derecho trasero (paralelismo, metodo de 2 puntos)
float32 left            # Zona lateral izquierda
float32 left_front     # S1 izquierdo: sensor izquierdo delantero (paralelismo, espejo de right_front)
float32 left_rear      # S2 izquierdo: sensor izquierdo trasero (paralelismo, espejo de right_rear)

bool front_valid
bool right_front_valid
bool right_valid
bool right_rear_valid
bool left_valid
bool left_front_valid
bool left_rear_valid

# Ajuste de recta a TODOS los puntos del LiDAR en el lado derecho
# (regresion por minimos cuadrados, mas robusto al ruido que S1/S2).
# Usado por wall_follower_node como metodo principal de seguimiento;
# right_front/right_rear se mantienen para ALINEAR y como referencia.
float32 right_line_angle_rad     # angulo de la pared respecto al frente del robot
float32 right_line_distance_m    # distancia perpendicular del robot a la pared
bool right_line_valid

# Espejo de right_line_* para el lado izquierdo (seguimiento de pared
# izquierda, lado_seguimiento: IZQUIERDA).
float32 left_line_angle_rad
float32 left_line_distance_m
bool left_line_valid
