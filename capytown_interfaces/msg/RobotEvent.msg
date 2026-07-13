# Evento discreto emitido por state_machine_node para que
# metrics_logger_node registre metricas sin acoplarse a la logica
# de navegacion. "tipo" usa valores de texto fijos (ver constantes
# en capytown_granprix/event_types.py):
#   INICIO, CRUCE, GIRO, DEAD_END, COLISION,
#   PARE_DETECTADO, PARE_RESPETADO, PARE_FALSO,
#   CELDA_AVANZADA, META, TIMEOUT

std_msgs/Header header

string tipo
string detalle
