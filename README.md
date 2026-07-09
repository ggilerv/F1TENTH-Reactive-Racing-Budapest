# 🏎️ F1TENTH Reactive Racing — Budapest

**Autor:** George Gabriel Giler Vega
**Institución:** Escuela Superior Politécnica del Litoral (ESPOL)
**Stack tecnológico:** ROS 2 (Humble), Python, Simulador F1TENTH

Controlador de carreras puramente reactivo para el simulador F1TENTH, ajustado para el circuito de Budapest. Implementa el algoritmo **Follow The Gap (FTG)** combinado con **Disparity Extender**, un filtro temporal adaptativo sobre el LiDAR y una planificación de velocidad basada en curvatura y distancia de frenado. Incluye además un nodo de cronometraje de vueltas (`lap_timer`) al estilo de una torre de tiempos de Fórmula 1.

## Tabla de contenidos

1. [Arquitectura del controlador](#arquitectura-del-controlador)
2. [Demostración en pista](#demostración-en-pista)
3. [Instalación desde cero](#instalación-desde-cero)
   1. [Requisitos: ROS 2 Humble](#1-requisitos-ros-2-humble)
   2. [Instalar el simulador F1TENTH](#2-instalar-el-simulador-f1tenth)
   3. [Agregar este controlador al workspace](#3-agregar-este-controlador-al-workspace)
   4. [Agregar el mapa de Budapest](#4-agregar-el-mapa-de-budapest)
   5. [Configurar `sim.yaml`](#5-configurar-simyaml)
   6. [Compilar el workspace](#6-compilar-el-workspace)
   7. [Ejecutar el sistema completo](#7-ejecutar-el-sistema-completo)
4. [Problemas comunes](#problemas-comunes)
5. [Parámetros principales](#parámetros-principales)

## Arquitectura del controlador

El nodo `ftg_control` (`f1_reactive/ftg_node.py`) procesa cada escaneo del LiDAR en el siguiente orden:

1. **Recorte y suavizado:** se limita el escaneo al campo de visión frontal configurado y se aplica una media móvil espacial para atenuar el ruido punto a punto.
2. **Filtro temporal EMA adaptativo:** una media móvil exponencial suaviza el escaneo entre ciclos; su ganancia sube automáticamente cuando la discrepancia respecto al ciclo anterior indica un cambio real del entorno (curva cerrada) en vez de ruido del sensor.
3. **Disparity Extender:** extiende cada borde de obstáculo (salto brusco de distancia entre rayos) hacia el lado más cercano, por un ángulo equivalente al radio del vehículo, para evitar rozar esquinas.
4. **Burbuja de seguridad:** descarta la región angular alrededor de todo punto por debajo de una distancia mínima de alerta.
5. **Selección del hueco objetivo:** se elige el tramo libre más ancho del escaneo procesado y se calcula su centroide ponderado como ángulo de dirección objetivo.
6. **Protecciones de estabilidad sobre el ángulo de dirección:**
   - Límite de salto máximo del objetivo por ciclo de control.
   - Filtro EMA y limitador de tasa de giro adaptativo (más rápido ante un cambio real del entorno).
   - Tope de ángulo máximo según la velocidad actual, consistente con la aceleración lateral admisible.
7. **Planificación de velocidad:** la velocidad objetivo es el mínimo entre la velocidad admisible por curvatura (modelo Ackermann, acotada por `a_lat_max`), la velocidad admisible para frenar dentro de la distancia libre detectada (acotada por `a_freno_max`) y `velocidad_max`. Un frenado de emergencia se activa si hay un obstáculo demasiado cerca.
8. **Rampa de aceleración/frenado** antes de publicar el comando final en `/drive`.

El nodo `lap_timer` (`f1_reactive/lap_timer.py`) se suscribe a la odometría del vehículo (`/ego_racecar/odom`) y usa una máquina de estados espacial (arranque por velocidad → alejamiento de la línea de meta → cruce de regreso) para cronometrar cada vuelta y mostrar un tablero en vivo en la terminal.

## Demostración en pista

Evidencia del controlador completando el circuito de Budapest de forma autónoma:

https://youtu.be/MNEFUip30QQ

## Instalación desde cero

Estos pasos asumen Ubuntu 22.04 y una instalación limpia, desde la instalación de ROS 2 hasta ver el vehículo corriendo el circuito de Budapest.

### 1. Requisitos: ROS 2 Humble

Instala ROS 2 Humble siguiendo la guía oficial:
👉 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Verifica la instalación:
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

### 2. Instalar el simulador F1TENTH

Sigue la guía de instalación del workspace base del simulador (motor `f1tenth_gym`, puente ROS 2, RViz, dependencias vía `rosdep`, etc.):
👉 https://github.com/widegonz/F1Tenth-Repository

### 3. Agregar este controlador al workspace

Clona este repositorio directamente como el paquete `f1_reactive` dentro de `src/`:
```bash
cd ~/F1Tenth-Repository/src
git clone https://github.com/ggilerv/F1TENTH-Reactive-Racing-Budapest.git f1_reactive
```

### 4. Agregar el mapa de Budapest

Este repositorio incluye `Mapas-F1Tenth.zip` con el mapa de Budapest (y otros circuitos adicionales). Extrae los archivos del circuito de Budapest a la carpeta de mapas del simulador:
```bash
cd ~/F1Tenth-Repository/src/f1_reactive
unzip Mapas-F1Tenth.zip
cp Mapas-F1Tenth/Budapest_map.png Mapas-F1Tenth/Budapest_map.yaml \
   ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/
```

### 5. Configurar `sim.yaml`

Edita `~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml` para apuntar al mapa de Budapest y alinear el vehículo con la recta principal del circuito:
```yaml
    map_path: '/home/<tu_usuario>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Budapest_map'
    map_img_ext: '.png'

    num_agent: 1

    # posición y orientación inicial del vehículo en el mapa
    sx: 0.0
    sy: 0.0
    stheta: -0.70
```
Reemplaza `<tu_usuario>` por el nombre de usuario de tu máquina. El valor `stheta: -0.70` ya deja el vehículo alineado con la recta principal del circuito de Budapest; si más adelante cambias de mapa o de punto de arranque, puedes reajustar la orientación en vivo desde RViz con la herramienta **`2D Pose Estimate`**, arrastrando el cursor para que la flecha verde quede paralela a los muros de la pista.

### 6. Compilar el workspace

```bash
cd ~/F1Tenth-Repository
colcon build --symlink-install
source install/setup.bash
```

El `setup.py` del paquete `f1_reactive` ya declara los puntos de entrada (`entry_points`) necesarios para que, tras compilar, ambos nodos queden disponibles directamente desde la terminal con `ros2 run`:
```python
entry_points={
    'console_scripts': [
        'ftg_control = f1_reactive.ftg_node:main',
        'lap_timer = f1_reactive.lap_timer:main',
    ],
},
```

### 7. Ejecutar el sistema completo

Se necesitan **tres terminales**, cada una con el workspace ya *sourced*:
```bash
source ~/F1Tenth-Repository/install/setup.bash
```

**Terminal 1 — Simulador (bridge + RViz + mapa):**
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 — Cronometraje de vueltas:**
```bash
ros2 run f1_reactive lap_timer
```

**Terminal 3 — Controlador reactivo:**
```bash
ros2 run f1_reactive ftg_control
```

El vehículo debe empezar a moverse solo en cuanto arranca el nodo `ftg_control`, reaccionando al escaneo del LiDAR publicado por el simulador.

## Problemas comunes

Si el `launch` del simulador falla por un conflicto con la librería `coverage`, elimínala y recompila:
```bash
python3 -m pip uninstall -y coverage
sudo apt purge -y python3-coverage

cd ~/F1Tenth-Repository
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

## Parámetros principales

Todos los parámetros se declaran como parámetros de ROS 2 en `FollowTheGapRacing.__init__` (ver docstring de la clase en `f1_reactive/ftg_node.py` para la lista completa con unidades). Los más relevantes para ajustar el comportamiento en pista:

| Parámetro | Efecto |
|---|---|
| `velocidad_max` | Velocidad máxima absoluta en recta. |
| `a_lat_max` | Aceleración lateral máxima admisible en curva: a menor valor, entradas a curva más conservadoras. |
| `a_freno_max` | Desaceleración máxima de frenado. |
| `rango_frenado` | Distancia máxima considerada al planificar el frenado. |
| `radio_vehiculo` | Semiancho del vehículo más margen de seguridad, usado por el Disparity Extender y la burbuja de seguridad. |
| `max_rate_steering` / `max_rate_steering_alta` | Tasa de giro máxima del servo en condiciones normales / ante un cambio real del entorno. |
| `velocidad_steer_completo` | Velocidad por debajo de la cual se permite el ángulo de dirección completo. |
