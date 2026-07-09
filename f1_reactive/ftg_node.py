#!/usr/bin/env python3
"""
Follow The Gap + Disparity Extender — F1TENTH Reactive Racing (Budapest)
==========================================================================

Autor: George Gabriel Giler Vega
Repositorio: https://github.com/ggilerv/F1TENTH-Reactive-Racing-Budapest

Controlador de carreras puramente reactivo para el simulador F1TENTH
(ROS 2), diseñado y ajustado para el circuito de Budapest. Implementa el
algoritmo Follow The Gap (FTG) combinado con Disparity Extender para la
evasión de obstáculos, un filtro temporal adaptativo sobre el LiDAR, un
limitador de tasa de giro adaptativo y una planificación de velocidad
basada en el ángulo de dirección objetivo y la distancia libre de
frenado.

Pipeline por cada escaneo del LiDAR (ver `scan_callback`):
    1. Limpieza, recorte al campo de visión (FOV) y suavizado espacial.
    2. Filtro temporal EMA adaptativo (memoria entre ciclos del escaneo).
    3. Disparity Extender: extiende cada borde de obstáculo según el
       radio del vehículo.
    4. Burbuja de seguridad: descarta la zona angular alrededor de todo
       obstáculo cercano.
    5. Selección del hueco más ancho disponible y su centroide ponderado.
    6. Conversión a ángulo de dirección, con clamp de salto máximo por
       ciclo, filtro EMA, limitador de tasa de giro adaptativo y tope de
       dirección por velocidad.
    7. Planificación de velocidad: mínimo entre la velocidad admisible
       por curvatura (aceleración lateral), por distancia de frenado y
       la velocidad máxima configurada.
    8. Rampa de aceleración/frenado y publicación en `/drive`.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class FollowTheGapRacing(Node):
    """
    Nodo ROS 2 que implementa el controlador Follow The Gap Racing.

    Se suscribe a `/scan` (sensor_msgs/LaserScan) y publica en `/drive`
    (ackermann_msgs/AckermannDriveStamped) el ángulo de dirección y la
    velocidad objetivo en cada ciclo de escaneo del LiDAR.

    Parámetros ROS 2 declarados (ver `_cargar_parametros`):

    Percepción:
        rango_maximo (float, m): distancia máxima considerada al recortar
            el escaneo para la selección de huecos.
        radio_vehiculo (float, m): semiancho del vehículo más margen de
            seguridad; usado por el Disparity Extender y la burbuja.
        fov_grados (float, °): campo de visión frontal considerado por el
            algoritmo Follow The Gap.
        umbral_disparidad (float, m): salto mínimo de distancia entre
            rayos consecutivos para considerarlo un borde de obstáculo.
        distancia_alerta_burbuja (float, m): distancia por debajo de la
            cual un punto activa la burbuja de seguridad.
        factor_umbral_libre (float): multiplicador de radio_vehiculo que
            define cuánto espacio libre se considera un hueco transitable.
        ventana_suavizado (int, rayos): tamaño de la ventana de la media
            móvil aplicada espacialmente al escaneo.

    Perfil de motor:
        velocidad_max (float, m/s): velocidad máxima absoluta.
        velocidad_min (float, m/s): velocidad mínima en curva.
        alpha_steering (float, 0–1): peso del filtro EMA sobre el ángulo
            de dirección (0 = máximo suavizado, 1 = sin suavizado).

    Física y cinemática:
        a_lat_max (float, m/s²): aceleración lateral máxima admisible en
            curva, usada para limitar la velocidad según la curvatura.
        a_freno_max (float, m/s²): desaceleración máxima de frenado.
        max_aceleracion (float, m/s²): aceleración longitudinal máxima.
        wheelbase (float, m): distancia entre ejes del vehículo.
        max_steering_angle (float, rad): ángulo de dirección máximo
            físico del vehículo.
        rango_frenado (float, m): distancia máxima considerada al
            planificar la velocidad de frenado.
        cono_frenado_grados (float, °): ancho angular del cono usado para
            medir la distancia libre de frenado.

    Dinámica de dirección:
        beta_filtro_temporal (float, 0–1): ganancia base del filtro EMA
            temporal del escaneo LiDAR.
        max_rate_steering (float, °/s): tasa de giro máxima del servo en
            condiciones normales.
        beta_filtro_temporal_max (float, 0–1): ganancia máxima del filtro
            EMA temporal ante un cambio real del entorno.
        max_rate_steering_alta (float, °/s): tasa de giro máxima del
            servo ante un cambio real del entorno (curva cerrada).
        umbral_cambio_bajo (float, m): discrepancia entre escaneos por
            debajo de la cual se considera ruido normal del sensor.
        umbral_cambio_alto (float, m): discrepancia a partir de la cual
            se considera un cambio real del entorno.

    Protecciones de estabilidad:
        max_delta_objetivo_grados (float, °): máximo salto permitido del
            ángulo objetivo de dirección por ciclo de control.
        velocidad_steer_completo (float, m/s): velocidad por debajo de la
            cual se permite el ángulo de dirección completo.
    """

    def __init__(self):
        """Declara los parámetros ROS 2 (ver docstring de la clase), crea la
        suscripción a `/scan` y el publicador de `/drive`, e inicializa el
        estado interno del controlador."""
        super().__init__('follow_the_gap')

        self.declare_parameter('rango_maximo', 5.0)
        self.declare_parameter('radio_vehiculo', 0.40)
        self.declare_parameter('fov_grados', 180.0)
        self.declare_parameter('umbral_disparidad', 0.30)
        self.declare_parameter('distancia_alerta_burbuja', 1.6)
        self.declare_parameter('factor_umbral_libre', 1.6)
        self.declare_parameter('ventana_suavizado', 3)

        self.declare_parameter('velocidad_max', 17.0)
        self.declare_parameter('velocidad_min', 1.5)
        self.declare_parameter('alpha_steering', 0.60)

        self.declare_parameter('a_lat_max', 5.8)
        self.declare_parameter('a_freno_max', 8.5)
        self.declare_parameter('max_aceleracion', 6.0)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('rango_frenado', 12.0)
        self.declare_parameter('cono_frenado_grados', 20.0)

        self.declare_parameter('beta_filtro_temporal', 0.40)
        self.declare_parameter('max_rate_steering', 400.0)
        self.declare_parameter('beta_filtro_temporal_max', 0.95)
        self.declare_parameter('max_rate_steering_alta', 420.0)
        self.declare_parameter('umbral_cambio_bajo', 0.05)
        self.declare_parameter('umbral_cambio_alto', 0.40)

        self.declare_parameter('max_delta_objetivo_grados', 20.0)
        self.declare_parameter('velocidad_steer_completo', 4.5)

        self._cargar_parametros()

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.angulo_anterior = 0.0
        self.velocidad_anterior = 0.0
        self.ultimo_tiempo = self.get_clock().now()
        self._ranges_ema = None
        self._factor_cambio = 0.0

        self.get_logger().info(
            'Follow The Gap Racing node initialized. '
            f'max_rate_alta={math.degrees(self.MAX_RATE_STEERING_ALTA_RAD_S):.0f} deg/s, '
            f'umbral_cambio_alto={self.UMBRAL_CAMBIO_ALTO:.2f} m, '
            f'max_delta_objetivo={math.degrees(self.MAX_DELTA_OBJETIVO_RAD):.0f} deg, '
            f'velocidad_steer_completo={self.V_STEER_COMPLETO:.1f} m/s')

    def _cargar_parametros(self):
        """Lee los parámetros ROS 2 declarados y los asigna como atributos de instancia."""
        gp = self.get_parameter
        self.RANGO_MAXIMO = float(gp('rango_maximo').value)
        self.RADIO_VEHICULO = float(gp('radio_vehiculo').value)
        self.FOV_RAD = math.radians(float(gp('fov_grados').value))
        self.UMBRAL_DISPARIDAD = float(gp('umbral_disparidad').value)
        self.DISTANCIA_ALERTA_BURBUJA = float(gp('distancia_alerta_burbuja').value)
        self.FACTOR_UMBRAL_LIBRE = float(gp('factor_umbral_libre').value)
        self.VENTANA_SUAVIZADO = max(1, int(gp('ventana_suavizado').value))
        self.VELOCIDAD_MAX = float(gp('velocidad_max').value)
        self.VELOCIDAD_MIN = float(gp('velocidad_min').value)
        self.ALPHA = float(gp('alpha_steering').value)
        self.A_LAT_MAX = float(gp('a_lat_max').value)
        self.A_FRENO_MAX = float(gp('a_freno_max').value)
        self.MAX_ACCEL = float(gp('max_aceleracion').value)
        self.WHEELBASE = float(gp('wheelbase').value)
        self.MAX_STEER = float(gp('max_steering_angle').value)
        self.RANGO_FRENADO = float(gp('rango_frenado').value)
        self.CONO_FRENADO_RAD = math.radians(float(gp('cono_frenado_grados').value))
        self.BETA_FILTRO_TEMPORAL = float(gp('beta_filtro_temporal').value)
        self.MAX_RATE_STEERING_RAD_S = math.radians(float(gp('max_rate_steering').value))
        self.BETA_FILTRO_TEMPORAL_MAX = float(gp('beta_filtro_temporal_max').value)
        self.MAX_RATE_STEERING_ALTA_RAD_S = math.radians(float(gp('max_rate_steering_alta').value))
        self.UMBRAL_CAMBIO_BAJO = float(gp('umbral_cambio_bajo').value)
        self.UMBRAL_CAMBIO_ALTO = float(gp('umbral_cambio_alto').value)
        self.MAX_DELTA_OBJETIVO_RAD = math.radians(float(gp('max_delta_objetivo_grados').value))
        self.V_STEER_COMPLETO = float(gp('velocidad_steer_completo').value)

    def scan_callback(self, msg: LaserScan):
        """
        Ciclo principal de control, ejecutado en cada escaneo del LiDAR.

        Procesa el escaneo (recorte, suavizado, filtro temporal,
        Disparity Extender y burbuja de seguridad), selecciona el hueco
        objetivo, calcula el ángulo de dirección con sus protecciones de
        estabilidad, planifica la velocidad según la curvatura y la
        distancia de frenado disponible, y publica el comando resultante
        en `/drive`.

        Args:
            msg: escaneo de LiDAR recibido en el tópico `/scan`.
        """
        ahora = self.get_clock().now()
        dt = (ahora - self.ultimo_tiempo).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02
        self.ultimo_tiempo = ahora

        angle_increment = max(msg.angle_increment, 1e-6)
        n_total = len(msg.ranges)
        if n_total < 10:
            return

        centro_idx_total = n_total // 2

        ranges_completos = np.array(msg.ranges, dtype=np.float64)
        ranges_completos = np.where(np.isfinite(ranges_completos),
                                     ranges_completos, 100.0)
        ranges_completos = np.clip(ranges_completos, 0.0, 100.0)

        medio_fov_idx = int(round((self.FOV_RAD / 2.0) / angle_increment))
        start_idx = max(0, centro_idx_total - medio_fov_idx)
        end_idx = min(n_total, centro_idx_total + medio_fov_idx)
        ranges = ranges_completos[start_idx:end_idx].copy()
        ranges = np.clip(ranges, 0.0, self.RANGO_MAXIMO)

        if self.VENTANA_SUAVIZADO > 1:
            kernel = np.ones(self.VENTANA_SUAVIZADO) / self.VENTANA_SUAVIZADO
            ranges_suaves = np.convolve(ranges, kernel, mode='same')
        else:
            ranges_suaves = ranges

        min_dist_bruto = float(np.min(ranges_suaves))

        ranges_ft = self._filtrar_temporalmente(ranges_suaves)
        ranges_seguros = self._extender_disparidades(ranges_ft, angle_increment)
        ranges_seguros = self._aplicar_burbuja_vectorizada(ranges_seguros,
                                                            angle_increment)

        idx_centro_fov = len(ranges_seguros) // 2
        umbral_libre = self.FACTOR_UMBRAL_LIBRE * self.RADIO_VEHICULO
        best_idx = self._elegir_objetivo(ranges_seguros, idx_centro_fov,
                                          umbral_libre)

        real_idx = best_idx + start_idx
        steering_angle = msg.angle_min + real_idx * angle_increment
        steering_angle = max(-self.MAX_STEER, min(self.MAX_STEER, steering_angle))

        steering_angle = max(
            self.angulo_anterior - self.MAX_DELTA_OBJETIVO_RAD,
            min(self.angulo_anterior + self.MAX_DELTA_OBJETIVO_RAD,
                steering_angle))

        smoothed_steering = (self.ALPHA * steering_angle
                             + (1.0 - self.ALPHA) * self.angulo_anterior)
        smoothed_steering = max(-self.MAX_STEER,
                                min(self.MAX_STEER, smoothed_steering))

        tasa_efectiva = (self.MAX_RATE_STEERING_RAD_S
                         + self._factor_cambio
                         * (self.MAX_RATE_STEERING_ALTA_RAD_S
                            - self.MAX_RATE_STEERING_RAD_S))
        max_delta_steer = tasa_efectiva * dt
        delta_steer = smoothed_steering - self.angulo_anterior
        delta_steer = max(-max_delta_steer, min(max_delta_steer, delta_steer))
        smoothed_steering = self.angulo_anterior + delta_steer
        smoothed_steering = max(-self.MAX_STEER,
                                min(self.MAX_STEER, smoothed_steering))

        v_actual = max(self.velocidad_anterior, 0.5)
        if v_actual > self.V_STEER_COMPLETO:
            max_steer_actual = min(
                self.MAX_STEER,
                self.MAX_STEER * self.V_STEER_COMPLETO / v_actual)
            smoothed_steering = max(-max_steer_actual,
                                    min(max_steer_actual, smoothed_steering))

        self.angulo_anterior = smoothed_steering

        distancia_frenado = self._distancia_frenado_extendida(
            ranges_completos, real_idx, angle_increment, centro_idx_total)

        velocidad_objetivo = self._planificar_velocidad(
            smoothed_steering, distancia_frenado, min_dist_bruto)

        if velocidad_objetivo > self.velocidad_anterior:
            max_delta = self.MAX_ACCEL * dt
        else:
            max_delta = self.A_FRENO_MAX * dt
        delta = velocidad_objetivo - self.velocidad_anterior
        delta = max(-max_delta, min(max_delta, delta))
        velocidad_final = self.velocidad_anterior + delta
        velocidad_final = max(0.0, min(self.VELOCIDAD_MAX, velocidad_final))
        self.velocidad_anterior = velocidad_final

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = smoothed_steering
        drive_msg.drive.speed = velocidad_final
        self.drive_pub.publish(drive_msg)

    def _filtrar_temporalmente(self, ranges_suaves):
        """
        Aplica un filtro EMA (media móvil exponencial) adaptativo sobre
        el escaneo, para atenuar el ruido del LiDAR entre ciclos.

        La ganancia del filtro (beta) sube desde `beta_filtro_temporal`
        hasta `beta_filtro_temporal_max` cuando la discrepancia entre el
        escaneo nuevo y la memoria EMA supera `umbral_cambio_alto`, lo
        que indica un cambio real del entorno (curva cerrada) en vez de
        ruido del sensor. El factor de cambio resultante (`_factor_cambio`,
        0–1) también alimenta el limitador de tasa de giro adaptativo.

        Args:
            ranges_suaves: escaneo recortado y suavizado espacialmente.

        Returns:
            El escaneo filtrado temporalmente (memoria EMA actualizada).
        """
        if (self._ranges_ema is None
                or len(self._ranges_ema) != len(ranges_suaves)):
            self._ranges_ema = ranges_suaves.copy()
            self._factor_cambio = 0.0
            return self._ranges_ema

        discrepancia = float(np.mean(np.abs(ranges_suaves - self._ranges_ema)))
        rango_umbral = max(self.UMBRAL_CAMBIO_ALTO - self.UMBRAL_CAMBIO_BAJO,
                           1e-6)
        factor = (discrepancia - self.UMBRAL_CAMBIO_BAJO) / rango_umbral
        factor = max(0.0, min(1.0, factor))
        self._factor_cambio = factor

        beta = (self.BETA_FILTRO_TEMPORAL
                + factor * (self.BETA_FILTRO_TEMPORAL_MAX
                            - self.BETA_FILTRO_TEMPORAL))
        self._ranges_ema = (beta * ranges_suaves
                            + (1.0 - beta) * self._ranges_ema)
        return self._ranges_ema

    def _distancia_frenado_extendida(self, ranges_completos, idx_objetivo_total,
                                      angle_increment, idx_centro_total):
        """
        Calcula la distancia libre hacia adelante para planificar el
        frenado, usando el escaneo completo (sin recortar a rango_maximo).

        Promedia los rayos dentro de un cono angular (`cono_frenado_grados`)
        centrado en el ángulo objetivo y otro centrado al frente del
        vehículo, y devuelve el mínimo entre ambos, acotado a
        `rango_frenado`.

        Args:
            ranges_completos: escaneo completo del LiDAR, sin recortar.
            idx_objetivo_total: índice del ángulo de dirección objetivo
                dentro del escaneo completo.
            angle_increment: resolución angular del LiDAR [rad].
            idx_centro_total: índice del rayo frontal del vehículo.

        Returns:
            Distancia libre estimada hacia adelante [m], acotada a
            `rango_frenado`.
        """
        cono_idx = max(1, int(round(self.CONO_FRENADO_RAD / angle_increment)))
        n = len(ranges_completos)

        def prom_cono(idx):
            """Promedio de distancias del escaneo completo en un cono angular centrado en idx."""
            i0 = max(0, idx - cono_idx)
            i1 = min(n, idx + cono_idx + 1)
            return float(np.mean(ranges_completos[i0:i1]))

        idx_objetivo_total = max(0, min(n - 1, idx_objetivo_total))
        idx_centro_total = max(0, min(n - 1, idx_centro_total))
        distancia = min(prom_cono(idx_objetivo_total),
                        prom_cono(idx_centro_total))
        return min(distancia, self.RANGO_FRENADO)

    def _extender_disparidades(self, ranges, angle_increment):
        """
        Disparity Extender: extiende cada borde de obstáculo (disparidad
        entre rayos consecutivos mayor a `umbral_disparidad`) hacia el
        lado más cercano, por un ángulo equivalente al radio del
        vehículo a esa distancia. Evita que el vehículo intente pasar
        rozando la esquina de un obstáculo cuyo borde lejano el LiDAR
        todavía no resuelve con suficiente margen.

        Args:
            ranges: escaneo tras el filtro temporal.
            angle_increment: resolución angular del LiDAR [rad].

        Returns:
            El escaneo con los bordes de obstáculo extendidos.
        """
        r = ranges.copy()
        n = len(ranges)
        diffs = np.diff(ranges)
        indices_disparidad = np.where(np.abs(diffs) > self.UMBRAL_DISPARIDAD)[0]

        for i in indices_disparidad:
            if ranges[i] < ranges[i + 1]:
                near_val = max(float(ranges[i]), 0.05)
                n_ext = int(math.ceil(
                    math.atan2(self.RADIO_VEHICULO, near_val) / angle_increment))
                r[i + 1:min(n, i + 1 + n_ext)] = np.minimum(
                    r[i + 1:min(n, i + 1 + n_ext)], near_val)
            else:
                near_val = max(float(ranges[i + 1]), 0.05)
                n_ext = int(math.ceil(
                    math.atan2(self.RADIO_VEHICULO, near_val) / angle_increment))
                inicio = max(0, i + 1 - n_ext)
                r[inicio:i + 1] = np.minimum(r[inicio:i + 1], near_val)
        return r

    def _aplicar_burbuja_vectorizada(self, ranges, angle_increment):
        """
        Marca como ocupada (distancia 0) la región angular alrededor de
        todo punto por debajo de `distancia_alerta_burbuja`, con un
        ancho angular equivalente al radio del vehículo a esa distancia.
        Es la burbuja de seguridad clásica del algoritmo Follow The Gap,
        vectorizada con NumPy para procesar el escaneo completo sin
        iterar rayo por rayo.

        Args:
            ranges: escaneo tras el Disparity Extender.
            angle_increment: resolución angular del LiDAR [rad].

        Returns:
            El escaneo con las burbujas de seguridad aplicadas.
        """
        bajo_umbral = ranges < self.DISTANCIA_ALERTA_BURBUJA
        if not np.any(bajo_umbral):
            return ranges

        r = ranges.copy()
        n = len(ranges)
        distancias_seguras = np.maximum(ranges, 0.05)
        angulos_burbuja = np.arctan2(self.RADIO_VEHICULO, distancias_seguras)
        n_burbuja = np.ceil(angulos_burbuja / angle_increment).astype(int)

        cierre = np.zeros(n, dtype=bool)
        for idx in np.where(bajo_umbral)[0]:
            nb = int(n_burbuja[idx])
            cierre[max(0, idx - nb):min(n, idx + nb + 1)] = True
        r[cierre] = 0.0
        return r

    def _elegir_objetivo(self, ranges, idx_centro, umbral_libre):
        """
        Selecciona el índice objetivo dentro del escaneo procesado:
        localiza el hueco (tramo contiguo con distancia mayor a
        `umbral_libre`) más ancho; en caso de empate, el más cercano al
        frente del vehículo. Si no hay ningún hueco libre, apunta al
        rayo con la mayor distancia disponible.

        Args:
            ranges: escaneo procesado (tras Disparity Extender y burbuja
                de seguridad).
            idx_centro: índice del rayo frontal del vehículo, usado como
                referencia para el desempate entre huecos del mismo ancho.
            umbral_libre: distancia mínima para considerar un rayo como
                parte de un hueco transitable.

        Returns:
            Índice (dentro de `ranges`) del centroide ponderado del hueco
            elegido.
        """
        mask = ranges > umbral_libre
        cambios = np.diff(mask.astype(np.int8))
        inicios = list(np.where(cambios == 1)[0] + 1)
        finales = list(np.where(cambios == -1)[0])
        if mask[0]:
            inicios.insert(0, 0)
        if mask[-1]:
            finales.append(len(mask) - 1)

        if not inicios:
            return int(np.argmax(ranges))

        mejor_inicio = inicios[0]
        mejor_fin = finales[0]
        mejor_largo = finales[0] - inicios[0] + 1
        for s, e in zip(inicios, finales):
            largo = e - s + 1
            if largo > mejor_largo:
                mejor_largo, mejor_inicio, mejor_fin = largo, s, e
            elif largo == mejor_largo:
                ca = (mejor_inicio + mejor_fin) / 2.0
                cn = (s + e) / 2.0
                if abs(cn - idx_centro) < abs(ca - idx_centro):
                    mejor_inicio, mejor_fin = s, e

        segmento = ranges[mejor_inicio:mejor_fin + 1]
        indices_segmento = np.arange(mejor_inicio, mejor_fin + 1)
        centroide = float(np.sum(indices_segmento * segmento) / np.sum(segmento))
        return int(round(centroide))

    def _planificar_velocidad(self, steering_angle, distancia_frenado,
                               min_dist_bruto):
        """
        Calcula la velocidad objetivo como el mínimo entre tres límites:
        la velocidad máxima admisible por curvatura (modelo Ackermann,
        acotada por `a_lat_max`), la velocidad máxima admisible para
        poder frenar dentro de `distancia_frenado` (acotada por
        `a_freno_max`), y `velocidad_max`. Si hay un obstáculo
        demasiado cerca (por debajo de `radio_vehiculo + 0.10` m) fuerza
        un frenado de emergencia.

        Args:
            steering_angle: ángulo de dirección comandado [rad].
            distancia_frenado: distancia libre estimada hacia adelante [m].
            min_dist_bruto: distancia mínima bruta del escaneo recortado,
                usada para el frenado de emergencia.

        Returns:
            Velocidad objetivo [m/s].
        """
        if abs(steering_angle) > 1e-3:
            curvatura = abs(math.tan(steering_angle)) / self.WHEELBASE
            v_curvatura = math.sqrt(self.A_LAT_MAX / curvatura)
        else:
            v_curvatura = self.VELOCIDAD_MAX

        distancia_segura = max(0.0, distancia_frenado - self.RADIO_VEHICULO)
        v_frenado = math.sqrt(2.0 * self.A_FRENO_MAX * distancia_segura)

        velocidad = min(self.VELOCIDAD_MAX, v_curvatura, v_frenado)
        velocidad = max(self.VELOCIDAD_MIN, velocidad)

        umbral_emergencia = self.RADIO_VEHICULO + 0.10
        if min_dist_bruto < umbral_emergencia:
            velocidad = min(velocidad, self.VELOCIDAD_MIN * 0.4)

        return velocidad


def main(args=None):
    """Punto de entrada del nodo: inicializa ROS 2, hace spin y cierra limpiamente."""
    rclpy.init(args=args)
    nodo = FollowTheGapRacing()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
