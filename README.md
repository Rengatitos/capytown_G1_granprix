# Reto Final ROBÓTICA — Yahboom ROSMASTER R2 — Gran Prix CapyTown

Navegación autónoma por **seguimiento de pared derecha** para el laberinto
Gran Prix CapyTown (pista 360×240 cm, rejilla 6×4 celdas de 60×60 cm,
inicio en **A4**, meta en **F1**). Robot Yahboom ROSMASTER R2 (chasis
Ackermann) con Raspberry Pi 5, ROS 2 Humble, LiDAR MS200 y cámara opcional.

Documentos de referencia del reto (en esta misma carpeta):
`logica_pared_derecha_robot.md`, `DETALLE RETO 3.md`, `DETALLE_PISTA.md`,
`PROPIEDADES_ROBOT.md`.

---

## 1. Estructura del proyecto

```text
Reto Final/
├── capytown_interfaces/          # Mensajes personalizados (ament_cmake)
│   └── msg/
│       ├── LidarZones.msg        # Distancias por zona + ajuste de linea (right_line_*)
│       └── RobotEvent.msg        # Eventos discretos para métricas
│
├── capytown_granprix/             # Paquete principal de navegación (ament_python)
│   ├── capytown_granprix/
│   │   ├── geometry_utils.py      # yaw, ángulos, clamp
│   │   ├── lidar_utils.py         # extracción de zonas + ajuste de línea del /scan
│   │   ├── grid_map.py            # celda/heading por conteo de movimientos
│   │   ├── maze_map.py            # grafo de intersecciones + BFS (memoria de ruta Ronda 2)
│   │   ├── event_types.py         # constantes de RobotEvent.tipo
│   │   ├── image_codec.py         # Image <-> array BGR sin cv_bridge (compartido)
│   │   ├── camera_publisher_node.py   # (1) Driver propio de cámara (sin cv_bridge, sin GUI)
│   │   ├── lidar_processor_node.py    # (2) Nodo de lectura LiDAR
│   │   ├── wall_follower_node.py      # (3) Wall following derecha
│   │   ├── state_machine_node.py      # (4) Decisión de intersecciones + FSM
│   │   ├── stop_sign_detector_node.py # (5) Cámara: detección de PARE
│   │   └── metrics_logger_node.py     # (6) Métricas -> CSV
│   ├── config/granprix_params.yaml    # (7) Umbrales y parámetros
│   ├── launch/granprix_bringup.launch.py  # (8) Launch file
│   └── package.xml / setup.py / setup.cfg
│
├── sim_local/                     # Simulador local sin ROS2 (ver sección 5.3.1)
│   ├── environment.py             # pasillo + LiDAR simulado (ray casting)
│   ├── robot_model.py             # cinemática de uniciclo
│   ├── wall_follow_control.py     # MISMA lógica que los nodos reales, portable
│   └── run_sim.py                 # visualización matplotlib en vivo
│
├── lidar_viz.py                   # Diagnóstico visual del /scan REAL en el robot
│
└── README.md                      # (9) este archivo
```

### Arquitectura de nodos y tópicos

```text
/dev/video0 ──> camera_publisher_node ──> /camera/image_raw ──> stop_sign_detector_node ──> /pare_detectado ──┐
                                                                                                                 │
/scan ──> lidar_processor_node ──> /lidar_zones ──┬──> wall_follower_node ──> /wall_follow/cmd_vel_suggestion ─┤
                                                    └──────────────────────────────────────> state_machine_node ◄┘
/odom_raw ─────────────────────────────────────────────────────────────────────────────────> state_machine_node
                                                                                                     │
                                                                                                     ├──> /cmd_vel  (único publicador)
                                                                                                     ├──> /robot_state (String, debug)
                                                                                                     └──> /robot_event (RobotEvent)
                                                                                                              │
                                                                                                              └──> metrics_logger_node ──> metricas_granprix.csv
```

**Regla de diseño clave:** `state_machine_node` es el **único** nodo que
escribe en `/cmd_vel`. `wall_follower_node` solo publica una *sugerencia*
de velocidad; `state_machine_node` la reenvía mientras el estado es
`AVANZAR_PARALELO` y calcula sus propios comandos en el resto de estados
(girar, alinear, detenerse). Esto evita que dos nodos manden comandos de
movimiento contradictorios al mismo tiempo.

---

## 2. Máquina de estados

```text
INICIAR
  │
  ▼
AVANZAR_PARALELO  ◄─────────────────────────────┐
  │ (avanza 60 cm manteniendo pared derecha)      │
  ▼                                               │
DETECTAR_CRUCE (detenido, confirma D/F/I)         │
  ▼                                               │
BUSCAR_PARE (cámara, hasta 3 s si detecta PARE)   │
  ▼                                               │
DECIDIR (derecha→frente→izquierda→180°)           │
  ▼                                               │
GIRAR (arco lento + yaw de odometría) ── (recto) ─┤
  ▼                                               │
ALINEAR (empareja S1≈S2 con la pared derecha)     │
  ▼                                               │
VERIFICAR_META ────────── no es meta ─────────────┘
  │
  ▼ (celda == F1)
META (fin)
```

Estado adicional `DETENIDO`: red de seguridad si se supera
`max_celdas_recorridas` sin llegar a la meta (evita loops infinitos por
fallas de sensor). No forma parte del flujo pedido, solo protege la corrida.

### Por qué no se pega a la pared ni va en diagonal

- La distancia objetivo a la pared derecha es **20 cm** (rango aceptable
  18–25 cm), usando el **promedio** de dos zonas del LiDAR (S1 delantera y
  S2 trasera del lado derecho), no una distancia mínima "pegada".
- Antes de corregir distancia, el robot corrige **paralelismo** (S1≈S2).
  Esto es lo que evita el zigzag/diagonal descrito en
  `logica_pared_derecha_robot.md`.
- Los giros de intersección solo ocurren con el robot **detenido primero**
  (frenado explícito antes de `GIRAR`) y se **alinea** después de cada giro
  antes de volver a avanzar.

### Adaptación por chasis Ackermann

El ROSMASTER R2 tiene dirección Ackermann: **no puede rotar sobre su
propio eje** (radio de giro cero), a diferencia de un robot diferencial.
El documento de referencia asume motores independientes izquierda/derecha;
aquí se adapta así:

- Los giros (`GIRAR`) y la alineación fina (`ALINEAR`) se hacen con un
  **arco de avance muy lento** (`velocidad_giro_lineal_mps` /
  `velocidad_alineacion_lineal_mps`, ambas bajas) combinado con la
  velocidad angular máxima permitida, cerrando el lazo con el **yaw real
  de `/odom_raw`** (no con un tiempo fijo), hasta alcanzar el error
  angular objetivo (90°, 180° o el ángulo de alineación).
- Esta aproximación necesita espacio libre alrededor (el pasillo de 60 cm
  y la separación de ~20-30 cm a las paredes que ya mantiene el robot dan
  margen suficiente), pero **debe calibrarse en pista** — ver sección 5.

---

## 3. Instalación

Flujo de trabajo: editar en el PC, copiar los archivos al robot por
SSH (`scp`/`rsync`), compilar en el robot dentro del contenedor Docker.

### Copiar el código al robot (desde el PC)

```bash
rsync -avz --exclude '__pycache__' --exclude '*.pyc' \
  ./ root@10.42.0.1:/root/yahboomcar_ws/src/reto-final/
```

### En el robot (`ssh root@10.42.0.1` → `docker exec -it friendly_pike bash`)

```bash
cd /root/yahboomcar_ws
colcon build --packages-select capytown_interfaces capytown_granprix
source install/setup.bash
```

> `capytown_interfaces` se debe compilar antes que `capytown_granprix`
> (colcon respeta esta dependencia automáticamente por `package.xml`, pero
> si se compila con `--packages-select` asegúrate de listar ambos).

Dependencias del sistema usadas por los nodos (ya deberían estar en la
imagen del robot; si falta alguna):

```bash
sudo apt install python3-opencv
pip3 install numpy
```

> **`cv_bridge` NO es una dependencia** de este proyecto a propósito:
> en este robot, instanciar `CvBridge()` produce un *segmentation
> fault* (desajuste entre la versión con la que está compilado
> `cv_bridge` y el `python3-opencv` instalado). `stop_sign_detector_node.py`
> y `dashboard_server_node.py` convierten `sensor_msgs/Image` a
> arrays de OpenCV a mano con `numpy` en su lugar — si algún día se
> reinstala `cv_bridge` en el robot y quieres usarlo, verifica primero
> con `python3 -c "from cv_bridge import CvBridge; CvBridge()"` que no
> siga fallando.

### Verificar tópicos del robot antes de lanzar

```bash
ros2 topic list
ros2 topic info /scan
ros2 topic info /odom_raw
```

Si el bringup del robot (`yahboomcar_bringup_launch.py`, paquete
`yahboomcar_bringup`) no está corriendo, lanzarlo primero — este
proyecto **no** reemplaza al driver del robot, solo consume sus
tópicos (`/scan`, `/odom_raw`, `/cmd_vel`).

> **La cámara es la excepción:** este proyecto trae su propio driver
> mínimo (`camera_publisher_node.py`), porque el script de demo de
> Yahboom para esta cámara (`yahboomcar_astra/colorHSV.py`, la que
> abre `/dev/video0`) usa `cv_bridge` (segfault en este robot, ver
> nota arriba) y además requiere una ventana gráfica (`cv.imshow`),
> por lo que no sirve corriendo por SSH sin servidor X. No hace falta
> lanzar nada aparte para la cámara: `granprix_bringup.launch.py` ya
> la levanta siempre (independiente de `usar_camara`). **No corras
> `colorHSV.py`/`colorTracker.py` al mismo tiempo** — ambos abren
> `/dev/video0` y competirían por el mismo dispositivo.

---

## 4. Ejecución

La cámara (`camera_publisher_node`) siempre se levanta con este launch,
sin necesidad de ningún flag. `usar_camara` solo controla si además se
lanza `stop_sign_detector_node` (detección de PARE) como consumidor de
esas imágenes — por defecto ya es `true`.

### Ronda 1 — Exploración (con detección de PARE)

```bash
ros2 launch capytown_granprix granprix_bringup.launch.py ronda:=1
```

### Ronda 2 — Time Attack

```bash
ros2 launch capytown_granprix granprix_bringup.launch.py ronda:=2
```

### Sin detección de PARE (pruebas de solo LiDAR; la cámara sigue activa igual)

```bash
ros2 launch capytown_granprix granprix_bringup.launch.py usar_camara:=false
```

### Dashboard en vivo (mapa + celda/heading + cámara + LiDAR)

`dashboard_server_node` también se levanta siempre con este launch —
no hace falta correrlo aparte. Mientras el robot navega, abre en el
navegador (misma red que la Pi):

```
http://10.42.0.1:8000
```

Muestra el mapa de intersecciones ya descubierto, la celda/heading/
estado FSM actual, la imagen de cámara en vivo y los puntos del LiDAR
en el marco del robot — todo por HTTP plano, sin RViz ni rosbridge.
Puerto configurable con el parámetro `puerto` de `dashboard_server`.

### Verificar en vivo

```bash
ros2 topic echo /lidar_zones
ros2 topic echo /robot_state
ros2 topic echo /robot_event
ros2 topic echo /pare_detectado
```

### Visualización en RViz

`state_machine_node` publica, además de lo anterior:

- `/trayectoria` (`nav_msgs/Path`) — trayectoria recorrida según
  `/odom_raw` corregido (decimada cada `path_min_dist_m`, 2 cm por
  defecto).
- `/marcadores_granprix` (`visualization_msgs/MarkerArray`) — un
  marcador persistente por cada intersección detectada (azul) y por
  cada señal PARE (rojo al detectarla, verde al respetarla).

Para verlo: `rviz2`, fijar el *Fixed Frame* al mismo `frame_id` que usa
`/odom_raw` (normalmente `odom`), y agregar un display `Path` sobre
`/trayectoria` y un `MarkerArray` sobre `/marcadores_granprix`.

Al llegar a la meta (o agotar `max_celdas_recorridas`), se escribe una
fila en `~/capytown_resultados/metricas_granprix.csv` (ruta configurable
con el parámetro `csv_path` de `metrics_logger`). Formato exacto en
`DETALLE RETO 3.md`, sección 12.

> **`pare_falsos`** no se puede medir de forma 100% automática: el robot
> no tiene forma de saber si una detección fue realmente una señal PARE
> real de la pista sin comparar contra el video/observación humana.
> Revisar manualmente los eventos `PARE_DETECTADO` contra las señales
> reales de la pista y ajustar ese campo en el CSV si corresponde.

---

## 5. Calibración

Seguir el orden sugerido por `logica_pared_derecha_robot.md` (sección 20),
probando por partes antes de correr el laberinto completo.

### 5.1 Escala del odómetro (`state_machine` — `factor_dist_odom` / `factor_ang_odom`)

El `/odom_raw` del ROSMASTER R2 sobreestima tanto distancia como ángulo de
forma consistente (no es ruido aleatorio, es un factor de escala fijo).
Calibrar esto primero: si la odometría miente sobre cuánto avanzó o giró
el robot, ninguna otra calibración (avance de 60 cm, giro de 90°) va a
dar buenos resultados aunque el control esté bien ajustado.

1. Con el robot quieto, leer la pose:
   ```bash
   ros2 topic echo /odom_raw --once
   ```
2. Empujar el robot **a mano** una distancia real conocida en línea recta
   (por ejemplo 60 cm, medida con cinta métrica) y volver a leer. Calcular
   `distancia_odom = sqrt((x2-x1)^2 + (y2-y1)^2)`.
3. Con el robot quieto de nuevo, anotar el quaternion de orientación,
   girarlo **a mano** un ángulo real conocido (90°, ayudándose de una
   escuadra) sin trasladarlo, y volver a leer. El yaw (para un quaternion
   con x=y=0) es `yaw = 2*atan2(z, w)`; calcular `angulo_odom = yaw2 - yaw1`.
4. Calcular los factores de corrección:
   ```text
   factor_dist_odom = distancia_real / distancia_odom
   factor_ang_odom  = angulo_real / angulo_odom
   ```
5. Poner esos valores en `granprix_params.yaml`, dentro de `state_machine`:
   ```yaml
   factor_dist_odom: 0.9474   # ejemplo calibrado: avance real 76 cm / odometro 78.3 cm
   factor_ang_odom: 0.9899    # ejemplo calibrado: giro real 90° / odometro 90.92°
   ```
   `state_machine_node` aplica estos factores a `/odom_raw` apenas lo
   recibe (`_on_odom`), así que tanto el avance por celda (60 cm) como el
   cierre de los giros por yaw quedan corregidos automáticamente — no hace
   falta tocar `wall_follower` ni `lidar_processor` para esto.
6. Repetir la prueba 2-3 veces (avance y giro) para confirmar que el
   factor es estable; si varía mucho entre pruebas, sospechar de
   deslizamiento de ruedas más que de un error de escala fijo.

### 5.2 Orientación y sentido del LiDAR (`lidar_processor`)

La forma más confiable de calibrar esto es **visual**, con el script de
diagnóstico `lidar_viz.py` (raíz del repo, no requiere `colcon build`):

```bash
python3 /root/yahboomcar_ws/src/reto-final/lidar_viz.py
```

Dibuja en vivo los puntos crudos de `/scan` en el marco del robot (frente
arriba) con 4 sectores de color (verde=frente, rojo=derecha,
azul=izquierda, naranja=atrás) y la distancia mínima de cada uno. Pon un
objeto grande (caja/tablón, para evitar ambigüedad angular) en cada lado
físico real del robot, uno a la vez, y confirma en qué sector de color
aparece la mancha de puntos:

- Si aparece en el sector correcto → no hace falta tocar nada.
- Si frente/atrás están cambiados → ajustar `front_offset_deg` (probar
  `180.0`).
- Si izquierda/derecha quedan cambiadas → `invert_left_right: true`.

**Importante — no confiar en un solo lado a la vez:** verificar
izquierda y derecha por separado, con un solo objeto genérico, puede
ser ambiguo (ruido del cuarto, objetos parecidos a ambos lados). La
prueba confiable es poner **dos objetos con distancias distintas y
reconocibles, uno en cada lado real, al mismo tiempo** (por ejemplo,
algo a ~40 cm en la derecha real y algo a ~13 cm en la izquierda
real) y comparar directamente: `DERECHA` debe mostrar ~0.40 e
`IZQUIERDA` ~0.13. Si salen cambiados (`DERECHA`≈0.13,
`IZQUIERDA`≈0.40), están invertidos. Cambiar `front_offset_deg` por sí
solo no permite inferir el estado de izquierda/derecha — hay que
probarlas explícitamente cada vez que se toque el offset.

Puedes probar valores sin editar el YAML todavía, pasando parámetros
directo al script:
```bash
python3 lidar_viz.py --ros-args -p front_offset_deg:=180.0 -p invert_left_right:=false
```

Una vez confirmado con la prueba de las dos distancias, fijar el valor
en `granprix_params.yaml` (`lidar_processor.front_offset_deg` /
`invert_left_right`) — en este robot quedó en **`front_offset_deg:
180.0`**, **`invert_left_right: true`**.

**Nota sobre esta calibración:** la prueba estática de las dos
distancias (robot detenido) dio `invert_left_right: false` como
correcto. Pero en la prueba dinámica real (robot avanzando con
`wall_follower` en movimiento) terminó siguiendo la pared izquierda en
vez de la derecha, y se corrigió a `invert_left_right: true` por
observación directa en movimiento. Si el comportamiento vuelve a
verse invertido, repetir la prueba de las dos distancias pero con el
robot en movimiento (no detenido), ya que parece haber una diferencia
entre ambos escenarios que no está del todo explicada.

Como verificación final con el paquete ya compilado:
```bash
ros2 run capytown_granprix lidar_processor_node
ros2 topic echo /lidar_zones
```
y confirmar que `front`, `right`/`right_front`/`right_rear` y `left`
bajan al acercar un objeto al lado físico correspondiente.

### 5.3 Avance recto y distancia a pared (`wall_follower`)

El control usa **regresión de línea**: `lidar_processor_node` ajusta
una recta por mínimos cuadrados a *todos* los puntos del LiDAR dentro
de `right_side_window_deg` (por defecto -135° a -45°, todo el lado
derecho), no solo 2 puntos sueltos (S1/S2) como una versión anterior —
mucho más robusto al ruido, porque un solo punto malo pesa poco entre
decenas. El resultado (`right_line_angle_rad`, `right_line_distance_m`,
`right_line_valid` en `/lidar_zones`) es lo que usa `wall_follower`:

Corrige **ángulo y distancia a la vez** (suma, no alternando entre uno
u otro):

```text
error_distancia = distancia_objetivo_m - right_line_distance_m
angular.z = ganancia_angulo * right_line_angle_rad
           + ganancia_distancia * error_distancia
```

(**sin signo negativo** en el término de ángulo — ver nota de signo
abajo). La primera versión alternaba ("si el ángulo está mal, corregir
*solo* ángulo; si no, corregir *solo* distancia"), pero eso crea un
ciclo que nunca se amortigua: cada corrección de distancia gira el
robot, lo que induce un error de ángulo, que dispara la corrección de
ángulo, que vuelve a inducir error de distancia — oscilación sostenida
de ±1.4 cm verificada en el simulador. Sumar ambas correcciones en el
mismo ciclo de control converge de verdad (std < 0.01 cm en la cola
del recorrido simulado).

Sin pared derecha de referencia (pasillo abierto), mantiene el rumbo
con un **Kp de heading** sobre el yaw de `/odom_raw` (`ganancia_heading`):
captura el yaw en el instante que pierde la pared y corrige cualquier
desviación de ese rumbo mientras dure el tramo sin pared — evita que
un sesgo mecánico del chasis (dirección Ackermann no perfectamente
centrada) lo desvíe sin que nada lo corrija.

> **Nota de signo (importante si tocas las ganancias):** con la pared
> horizontal en el mundo y el robot con yaw θ respecto a ella,
> `right_line_angle_rad ≈ -θ` (ver derivación en el comentario de
> `_on_zones` en `wall_follower_node.py`). Para corregir θ→0 hace falta
> `angular.z = -k·θ = +k·right_line_angle_rad`, **sin** signo negativo.
> Con el signo invertido el lazo es de realimentación **positiva** y el
> robot diverge (gira hasta ~90° y se sale del pasillo) en menos de 1
> segundo — se detectó y corrigió con el simulador local antes de tocar
> el robot real, ver sección 5.3.1.

**Distancia objetivo — por qué 12 cm y no menos:** el robot mide 16 cm
de ancho (8 cm de medio-ancho). Si el LiDAR está cerca del centro del
robot (supuesto por defecto), pedir una distancia objetivo menor a 8 cm
significa pedirle que la carrocería se meta en la pared. Un barrido de
parámetros en el simulador local (`sim_local/`) confirmó: con 9 cm o
menos, el robot choca en algún punto del recorrido sin importar la
ganancia; a partir de 10 cm deja de chocar pero con muy poco margen
(~0.5 cm); **12 cm da el mejor resultado** (margen real mínimo ~2.5 cm,
menor oscilación, robusto a ruido del LiDAR). Por eso
`distancia_objetivo_m: 0.12` es el valor por defecto, no algo más
ajustado — **antes de bajarlo, verificar en el robot real dónde está
montado el LiDAR respecto al borde derecho** (si está más cerca del
borde que del centro, un valor menor podría ser seguro).

#### 5.3.1 Simulador local (`sim_local/`) — probar sin arriesgar el robot

`sim_local/` es un simulador **Python puro, sin ROS2** (no requiere
`colcon build`, corre en el PC) que reproduce el mismo algoritmo de
control antes de tocar el robot real: un pasillo simulado con paredes
como segmentos, un LiDAR simulado por ray casting, y exactamente la
misma lógica de ajuste de línea + Kp que `lidar_processor_node` /
`wall_follower_node` (ver `sim_local/wall_follow_control.py` —
diseñado para portarse literalmente, no solo "inspirar").

```bash
cd sim_local
python run_sim.py
python run_sim.py --distancia-objetivo 0.10 --ganancia-distancia 1.5
python run_sim.py --gap      # prueba el fallback sin pared derecha
python run_sim.py --ruido 0.01
```

Dibuja el **robot a escala real** (24×16 cm) en el pasillo, con los
puntos del LiDAR simulado, la recta ajustada, y — lo más importante —
el **margen real hasta la carrocería** (no solo la distancia del
sensor), en rojo con `*** CHOQUE ***` si es negativo. Usar esto para
probar combinaciones de ganancias/distancia objetivo antes de arriesgar
el robot: cambiar un parámetro, correr, ver si choca o converge bien, y
solo entonces portar el valor al YAML real.

1. Colocar el robot en un pasillo recto de 60 cm, pared a la derecha.
2. `ros2 run capytown_granprix wall_follower_node` y observar
   `/wall_follow/cmd_vel_suggestion`.
3. Ajustar `distancia_objetivo_m`, `ganancia_angulo`, `ganancia_distancia`
   y `angular_max_radps` hasta que el robot recorra ~60 cm sin desviarse
   y sin zigzaguear (si oscila, bajar ganancias; si corrige muy lento,
   subirlas) — o probar primero en `sim_local/` antes de gastar tiempo
   de pista.
4. **Antes de probar a velocidad real:** con el robot detenido, acércalo
   a mano hasta la distancia objetivo configurada y confirma que la
   carrocería todavía no toca físicamente la pared.

### 5.4 Giro de 90° (`state_machine`, estado GIRAR)

1. Probar en una intersección real o simulando espacio libre a un lado.
2. Ajustar `velocidad_giro_lineal_mps`, `velocidad_giro_angular_radps` y
   `tolerancia_giro_deg` hasta lograr un giro de ~90° con error menor a
   ±3-4°, sin chocar contra las paredes del cruce.
3. El radio aproximado del arco es `r ≈ velocidad_giro_lineal_mps /
   velocidad_giro_angular_radps` (con los valores por defecto, 0.08/0.5 ≈
   0.16 m). Un radio pequeño es indispensable para que el giro de 180°
   (callejón sin salida) quepa dentro de una celda de 60 cm: si el robot
   se sale del pasillo al girar, bajar la velocidad lineal y/o subir la
   angular para reducir el radio, no al revés.

### 5.5 Alineación tras el giro (`ALINEAR`)

Ajustar `tolerancia_alineacion_m`, `velocidad_alineacion_lineal_mps` y
`velocidad_alineacion_angular_radps` hasta que, tras un giro, el robot
quede con S1≈S2 (diferencia menor a 2-3 cm) en un tiempo razonable
(menor a `tiempo_max_alinear_s`).

### 5.6 Intersección completa

Verificar que en un cruce real el robot: se detiene, confirma
correctamente derecha/frente/izquierda (`muestras_confirmacion`,
`consenso_minimo`), decide, gira si corresponde, se alinea y retoma el
avance recto — sin quedar en diagonal.

### 5.7 Señal de PARE (`stop_sign_detector`)

1. Colocar una señal de PARE (roja) frente a la cámara y correr:
   ```bash
   ros2 run capytown_granprix stop_sign_detector_node
   ros2 topic echo /pare_detectado
   ```
2. Si no detecta o detecta de más, ajustar `rango1_min/max`,
   `rango2_min/max` (segmentación HSV de rojo), `area_minima_px`,
   `area_maxima_px` y la relación de aspecto. Activar
   `publicar_debug: true` y ver `/pare_detectado/debug_image` en RViz o
   `rqt_image_view` (el robot tiene TigerVNC con entorno gráfico) para
   ver el recuadro detectado.
3. Ajustar `frames_confirmacion` (más alto = menos falsos positivos, más
   lento para reaccionar) y `frames_perdida`.

### 5.8 Umbrales de decisión

`umbral_frente_pared_m`, `umbral_frente_libre_m`, `umbral_lado_libre_m` y
`umbral_colision_m` en `state_machine` son los que definen "pared cerca",
"camino libre" y "colisión". Ajustar según el ancho real del pasillo
(60 cm) y el tamaño del robot (24×16 cm).

---

## 6. Notas de diseño y limitaciones conocidas

- **Localización por conteo de celdas:** no hay marcas físicas de meta ni
  ArUco; la celda actual se estima contando avances de 60 cm y giros
  (`grid_map.py`), asumiendo ejecución sin deslizamiento severo. Si el
  robot pierde tracción o se desalinea mucho, la estimación de celda
  puede desincronizarse del mapa real.
- **Objeto al frente — regla general de seguridad:** no hay tópico
  confirmado de parachoques/bumper en el bringup del robot (ver
  `PROPIEDADES_ROBOT.md`); se usa el LiDAR frontal como proxy. Esta
  regla es independiente del estado de la máquina de estados (aplica
  siempre): si `front < umbral_colision_m` (0.10 m por defecto), el
  robot se detiene de inmediato, espera `tiempo_espera_obstaculo_s`
  (2 s) y vuelve a comprobar si ya está libre; si sigue bloqueado,
  repite la espera en bucle. Cada activación también cuenta como un
  evento `COLISION` para las métricas.
- **Pasillo sin pared derecha:** si tras un giro no hay pared derecha de
  referencia, `ALINEAR` se salta (el `yaw` de `GIRAR` ya dejó al robot
  orientado al cardinal correcto) — ver `logica_pared_derecha_robot.md`
  sección 15.
- **Ronda 2 (time attack) — memoria de ruta:** ver sección 6.1 abajo.

### 6.1 Memoria de ruta (Ronda 2)

Ronda 1 siempre es 100% reactiva (regla de mano derecha, explora "por
todos lados" incluyendo callejones sin salida) — eso no cambió. Pero
además, cada vez que confirma una intersección con el LiDAR, la va
guardando en un grafo de conectividad (`maze_map.py`) que escribe a
disco (`mapa_path`, por defecto
`~/capytown_resultados/mapa_granprix.json`) al llegar a la meta o al
agotar `max_celdas_recorridas`.

Si `ronda: 2` y `usar_mapa_ronda2: true` (por defecto), `state_machine`
carga ese mapa al arrancar y calcula con BFS la ruta más corta
**conocida** de inicio a meta. En cada intersección, si el siguiente
tramo del plan coincide con lo que el LiDAR confirma libre en ese
momento, gira directo sin evaluar la regla de mano derecha; si no
coincide (pared nueva, mapa incompleto), descarta el plan para el
resto de la corrida y sigue 100% reactivo, igual que Ronda 1 — nunca
confía ciegamente en el mapa viejo.

**Importante — solo tiene sentido si el layout NO cambia entre
rondas.** El reto oficial permite reconfigurar paredes y PARE entre
Ronda 1 y Ronda 2 justamente para evitar memorización; si tu
evaluación sí reconfigura, pon `usar_mapa_ronda2: false` en
`granprix_params.yaml` — Ronda 2 vuelve a ser 100% reactiva, igual que
Ronda 1, sin depender de ningún mapa previo.

---

## 7. Flujo de trabajo (PC ↔ robot)

Editar siempre en el PC, luego copiar al robot y compilar dentro del
contenedor. Resumen:

```bash
# PC
rsync -avz --exclude '__pycache__' --exclude '*.pyc' \
  ./ root@10.42.0.1:/root/yahboomcar_ws/src/reto-final/

# Robot (dentro del contenedor)
cd /root/yahboomcar_ws
colcon build --packages-select capytown_interfaces capytown_granprix
source install/setup.bash
ros2 launch capytown_granprix granprix_bringup.launch.py
```
