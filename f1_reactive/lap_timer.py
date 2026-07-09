#!/usr/bin/env python3
"""
F1 Timing Tower — F1TENTH Reactive Racing (Budapest)
=======================================================

Autor: George Gabriel Giler Vega
Repositorio: https://github.com/ggilerv/F1TENTH-Reactive-Racing-Budapest

Nodo ROS 2 de cronometraje de vueltas para el simulador F1TENTH. Se
suscribe a la odometría del vehículo (`/ego_racecar/odom`) y usa una
máquina de estados espacial, basada en la distancia euclidiana al punto
de arranque, para detectar el cruce de la línea de meta en cada vuelta.
Muestra en terminal un tablero de tiempos en vivo al estilo de una torre
de cronometraje de Fórmula 1 (vuelta actual, mejor vuelta, historial de
vueltas).

Detección de vuelta:
    1. Espera a que la velocidad del vehículo supere un umbral mínimo
       para descartar el ruido de odometría en reposo; en ese instante
       ancla la línea de meta a la posición actual y arranca el reloj.
    2. Espera a que el vehículo se aleje más de `DISTANCIA_SALIDA` del
       punto de arranque (evita registrar una vuelta falsa antes de que
       el vehículo realmente haya completado el circuito).
    3. Una vez alejado, cuando el vehículo vuelve a estar a menos de
       `DISTANCIA_META` del punto de arranque, se registra la vuelta y
       se reinicia el ciclo.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class F1TimingTower(Node):
    """
    Nodo de cronometraje de vueltas basado en odometría.

    Attributes:
        odom_sub: suscripción al tópico de odometría del vehículo ego.
        race_started (bool): True una vez que el vehículo superó el
            umbral de velocidad de arranque.
        start_x, start_y (float | None): coordenadas de la línea de
            meta, ancladas a la posición del vehículo en el instante de
            arranque.
        lap_start_time: timestamp ROS del inicio de la vuelta actual.
        laps_completed (int): número de vueltas completadas.
        TOTAL_LAPS (int): número total de vueltas de la sesión.
        best_lap_time (float): mejor tiempo de vuelta registrado [s].
        lap_history (list[float]): tiempo de cada vuelta completada [s].
        is_outside_start_zone (bool): True si el vehículo ya se alejó lo
            suficiente de la línea de meta como para poder registrar el
            próximo cruce como una vuelta válida.
        DISTANCIA_SALIDA (float): distancia mínima [m] a la que debe
            alejarse el vehículo de la línea de meta antes de que un
            regreso cuente como vuelta completada.
        DISTANCIA_META (float): distancia máxima [m] a la línea de meta
            para considerar que el vehículo la cruzó.
        live_timer: temporizador que refresca el tablero en pantalla.
    """

    def __init__(self):
        """Crea la suscripción a odometría, inicializa el estado de la
        carrera y el temporizador de refresco del tablero."""
        super().__init__('lap_timer')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)

        self.race_started = False
        self.start_x = None
        self.start_y = None
        self.lap_start_time = None

        self.laps_completed = 0
        self.TOTAL_LAPS = 10
        self.best_lap_time = float('inf')
        self.lap_history = []

        self.is_outside_start_zone = False
        self.DISTANCIA_SALIDA = 6.0
        self.DISTANCIA_META = 2.5

        self.live_timer = self.create_timer(0.1, self.print_dashboard)

    def format_time(self, seconds):
        """
        Convierte un tiempo en segundos a formato de cronómetro F1
        (minutos:segundos.milisegundos).

        Args:
            seconds (float): tiempo en segundos.

        Returns:
            str: tiempo formateado como "M:SS.mmm".
        """
        m = int(seconds // 60)
        s = seconds % 60
        return f"{m}:{s:06.3f}"

    def odom_callback(self, msg):
        """
        Callback de odometría: implementa la máquina de estados espacial
        de detección de vueltas descrita en el docstring del módulo.

        Ignora los mensajes una vez completadas todas las vueltas de la
        sesión (`TOTAL_LAPS`).

        Args:
            msg (nav_msgs.msg.Odometry): odometría del vehículo ego.
        """
        if self.laps_completed >= self.TOTAL_LAPS:
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)

        if not self.race_started:
            self.start_x = current_x
            self.start_y = current_y

            if speed > 0.15:
                self.race_started = True
                self.lap_start_time = self.get_clock().now()
            return

        dist = math.hypot(current_x - self.start_x, current_y - self.start_y)

        if not self.is_outside_start_zone:
            if dist > self.DISTANCIA_SALIDA:
                self.is_outside_start_zone = True
        else:
            if dist < self.DISTANCIA_META:
                self.record_lap()

    def record_lap(self):
        """
        Registra el cierre de una vuelta: calcula su duración, actualiza
        el mejor tiempo y el historial, y reinicia el cronómetro para la
        siguiente vuelta. Si se completó `TOTAL_LAPS`, detiene el
        refresco del tablero y muestra el estado final.
        """
        current_time = self.get_clock().now()
        lap_duration = (current_time - self.lap_start_time).nanoseconds / 1e9

        self.laps_completed += 1
        self.lap_history.append(lap_duration)

        if lap_duration < self.best_lap_time:
            self.best_lap_time = lap_duration

        self.lap_start_time = current_time
        self.is_outside_start_zone = False

        if self.laps_completed >= self.TOTAL_LAPS:
            self.live_timer.cancel()
            self.print_dashboard()

    def print_dashboard(self):
        """
        Refresca el tablero de cronometraje en la terminal: limpia la
        pantalla y muestra el estado de la sesión, la vuelta en curso,
        el mejor tiempo registrado y el historial completo de vueltas.
        """
        print('\033c', end='')

        print("="*55)
        print(" 🏎️  F1TENTH TIMING TOWER - HUNGARORING (BUDAPEST) ")
        print("="*55)

        if not self.race_started:
            print("\n 🔴 STATUS: WAITING FOR START")
            print(" 🛑 Esperando que el vehículo acelere...\n")
            print("="*55)
            return

        if self.laps_completed >= self.TOTAL_LAPS:
            print(f"\n 🏁 ¡BANDERA A CUADROS! SESIÓN FINALIZADA 🏁")
        else:
            print(f"\n 🟢 STATUS: RACE LIVE")

        print(f" 🔄 LAP: {min(self.laps_completed + 1, self.TOTAL_LAPS)} / {self.TOTAL_LAPS}\n")

        if self.laps_completed < self.TOTAL_LAPS:
            now = self.get_clock().now()
            current_lap_time = (now - self.lap_start_time).nanoseconds / 1e9
            print(f" ⏱️  CURRENT LAP:  {self.format_time(current_lap_time)}")
        else:
            print(f" ⏱️  CURRENT LAP:  --:--.---")

        if self.best_lap_time != float('inf'):
            print(f" 🟣 BEST LAP:     {self.format_time(self.best_lap_time)}")
        else:
            print(f" ⚪ BEST LAP:     --:--.---")

        print("\n [ HISTORIAL DE VUELTAS ]")

        if len(self.lap_history) == 0:
            print("    Aún no hay tiempos registrados.")
        else:
            for i, t in enumerate(self.lap_history):
                marker = "🟣" if t == self.best_lap_time else "⚪"
                print(f"    {marker} Vuelta {i + 1:02d}:   {self.format_time(t)}")

        print("\n" + "="*55)


def main(args=None):
    """Punto de entrada del nodo: inicializa ROS 2, hace spin y cierra limpiamente."""
    rclpy.init(args=args)
    node = F1TimingTower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
