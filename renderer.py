"""
Finestra Pygame: disegno della scena (Renderer) e ciclo di esecuzione interattivo (run_interactive).

Solo visualizzazione, nessuna logica di simulazione. Pygame è importato solo quando si apre la
finestra: la modalità --headless e gli esperimenti non lo richiedono.

Tasti: P pausa, V vettori, C comunicazione, Q esci.
"""

from typing import Callable, Optional, Tuple

import numpy as np

from simulation import SwarmSimulation
from world import clamp_magnitude

WINDOW_WIDTH = 1600  # finestra Pygame
WINDOW_HEIGHT = 960  # aspect ratio 20:12

# Stile linee tratteggiate (RGBA)
DASHED_LINE_TARGET_COLOR = (255, 110, 255, 100)
DASHED_LINE_ANCHOR_COLOR = (110, 250, 110, 100)
COMMUNICATION_LINE_COLOR = (100, 160, 220, 60)


class Renderer:
    def __init__(self, sim: SwarmSimulation, show_force_vectors: bool, show_communication: bool):
        import pygame  # import locale: la modalità --headless non richiede Pygame
        self.pg = pygame
        self.sim = sim
        self.cfg = sim.cfg
        self.show_force_vectors = show_force_vectors
        self.show_communication = show_communication
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        self.line_overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        pygame.display.set_caption("X-Coverage — Swarm Simulation 2D (Pygame)")
        self.font = pygame.font.SysFont("monospace", 15)

    def world_to_screen(self, pos: np.ndarray) -> Tuple[int, int]:
        """Converte coordinate mondo in pixel schermo con asse Y invertito per Pygame."""
        return (int(pos[0] / self.cfg.AREA_WIDTH * WINDOW_WIDTH),
                int((1.0 - pos[1] / self.cfg.AREA_HEIGHT) * WINDOW_HEIGHT))

    def world_length_to_screen(self, length: float) -> int:
        """Converte una lunghezza in metri in pixel (scala orizzontale)."""
        return max(1, int(length / self.cfg.AREA_WIDTH * WINDOW_WIDTH))

    def close(self) -> None:
        self.pg.quit()

    def _draw_transparent_circle(self, color, center, radius) -> None:
        pg = self.pg
        radius = max(1, int(radius))
        target_rect = pg.Rect(center[0] - radius, center[1] - radius, radius * 2, radius * 2)
        shape_surface = pg.Surface(target_rect.size, pg.SRCALPHA)
        pg.draw.circle(shape_surface, color, (radius, radius), radius)
        self.screen.blit(shape_surface, target_rect)

    def _draw_dashed_line(self, surface, color, p0, p1, dash_len: int, gap_len: int) -> None:
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        length = max(1.0, float(np.hypot(dx, dy)))
        steps = int(length // (dash_len + gap_len))
        for i in range(steps):
            start_t = i * (dash_len + gap_len) / length
            end_t = min(1.0, (i * (dash_len + gap_len) + dash_len) / length)
            self.pg.draw.line(surface, color,
                              (p0[0] + dx * start_t, p0[1] + dy * start_t),
                              (p0[0] + dx * end_t, p0[1] + dy * end_t), 1)

    def _draw_vector(self, start_pos: np.ndarray, vec: np.ndarray, color: Tuple[int, int, int], scale: float, weight: int = 1) -> None:
        if np.linalg.norm(vec) < 1e-8:
            return
        end_pos = start_pos + clamp_magnitude(vec, self.cfg.MAX_FORCE_ON_TARGET) * scale
        self.pg.draw.line(self.screen, color, self.world_to_screen(start_pos), self.world_to_screen(end_pos), weight)

    def draw_scene(self, paused: bool = False) -> None:
        pg = self.pg
        sim = self.sim
        self.screen.fill((20, 20, 25))
        self.line_overlay.fill((0, 0, 0, 0))

        # 1. Stazioni idriche (cerchio = area di servizio, tratteggio = anello di attesa)
        for station_idx, station_pos in enumerate(sim.world.water_stations):
            st_x, st_y = self.world_to_screen(station_pos)
            pg.draw.circle(self.screen, (30, 100, 200), (st_x, st_y), self.world_length_to_screen(self.cfg.WATER_STATION_SERVICE_RADIUS), 2)
            pg.draw.circle(self.line_overlay, (30, 100, 200, 70), (st_x, st_y), self.world_length_to_screen(self.cfg.WATER_STATION_WAIT_RADIUS), 1)
            txt = self.font.render(f"WATER {station_idx}", True, (100, 180, 255))
            self.screen.blit(txt, (st_x - 28, st_y - self.world_length_to_screen(self.cfg.WATER_STATION_SERVICE_RADIUS) - 18))

        # 2. Incendi
        for fire in sim.world.fires:
            fx, fy = self.world_to_screen(fire.pos)
            det_r = self.world_length_to_screen(self.cfg.FIRE_DETECTION_RADIUS)
            ext_r = self.world_length_to_screen(self.cfg.FIRE_EXTINGUISH_RADIUS)

            fire_txt = self.font.render(f"{fire.health:.0f}", True, (255, 200, 50))
            self.screen.blit(fire_txt, (fx - 10, fy - 30))

            pg.draw.circle(self.screen, (255, 140, 40), (fx, fy), det_r, 1)
            pg.draw.circle(self.screen, (200, 80, 0), (fx, fy), ext_r, 1)

            ratio = max(0.0, min(fire.health, self.cfg.FIRE_HEALTH) / self.cfg.FIRE_HEALTH)
            f_color = (int(100 + 155 * ratio), int(200 - 120 * ratio), int(100 - 100 * ratio), 50)
            self._draw_transparent_circle(f_color, (fx, fy), ext_r)

        # 3. Connessioni di comunicazione (sull'overlay: l'alpha funziona solo su superfici SRCALPHA)
        if self.show_communication:
            for drone in sim.drones:
                for other in sim.last_neighbors.get(drone.idx, []):
                    if other.idx > drone.idx:
                        pg.draw.line(self.line_overlay, COMMUNICATION_LINE_COLOR,
                                     self.world_to_screen(drone.position), self.world_to_screen(other.position), 1)

        # 4. Linee tratteggiate drone -> target e drone -> ancora
        for drone in sim.drones:
            p = self.world_to_screen(drone.position)
            self._draw_dashed_line(self.line_overlay, DASHED_LINE_TARGET_COLOR, p, self.world_to_screen(drone.target), 6, 5)
            self._draw_dashed_line(self.line_overlay, DASHED_LINE_ANCHOR_COLOR, p, self.world_to_screen(drone.anchor_target), 4, 5)

        self.screen.blit(self.line_overlay, (0, 0))

        # 5. Droni, target e vettori
        for drone in sim.drones:
            px, py = self.world_to_screen(drone.position)
            tx, ty = self.world_to_screen(drone.target)
            ox, oy = self.world_to_screen(drone.original_target)
            ax, ay = self.world_to_screen(drone.anchor_target)

            pg.draw.circle(self.screen, (50, 220, 80), (ox, oy), 3)
            pg.draw.circle(self.screen, (160, 160, 255), (ax, ay), 3, 1)
            pg.draw.circle(self.screen, (230, 50, 50), (tx, ty), 4)

            if self.show_force_vectors:
                self._draw_vector(drone.target, drone.compute_repulsion_between_targets(), (255, 200, 50), 0.2)
                self._draw_vector(drone.target, drone.compute_anchor_force(), (50, 255, 200), 0.2)
                self._draw_vector(drone.target, drone.compute_boundary_force(), (50, 50, 230), 0.2)
                self._draw_vector(drone.target, drone.last_applied_force, (255, 255, 255), 0.2)
                # Correzione di velocità dovuta al collision avoidance (applicata al drone, non al target)
                self._draw_vector(drone.position, drone.last_avoidance_force, (255, 80, 255), 0.5)

            if drone.is_extinguishing_fire():
                pg.draw.circle(self.screen, (255, 230, 20), (px, py), 12, 2)

            # Drone ammesso alla stazione (ha uno slot di servizio)
            if drone.reloading and drone.station_slot is not None:
                pg.draw.circle(self.screen, (20, 200, 255), (px, py), 14, 2)

            w_ratio = drone.water / self.cfg.DRONE_WATER_CAPACITY
            d_color = (int(255 * (1.0 - w_ratio)), int(180 * w_ratio + 50), int(255 * w_ratio))
            pg.draw.circle(self.screen, d_color, (px, py), self.world_length_to_screen(self.cfg.DRONE_IMPACT_RADIUS))

        # 6. Overlay statistiche
        status = sim.summary() + ("   [PAUSA]" if paused else "")
        self.screen.blit(self.font.render(status, True, (255, 255, 0)), (10, 10))
        help_txt = "P pausa  V vettori  C comunicazione  Q esci"
        ablations = self._ablation_label()
        if ablations:
            help_txt += "   |  " + ablations
        self.screen.blit(self.font.render(help_txt, True, (150, 150, 150)), (10, 28))

        pg.display.flip()

    def _ablation_label(self) -> str:
        cfg = self.cfg
        parts = []
        if cfg.AVOIDANCE_MODE != "full":
            parts.append(f"avoidance={cfg.AVOIDANCE_MODE}")
        if cfg.PACKET_LOSS > 0.0:
            parts.append(f"packet-loss={cfg.PACKET_LOSS:g}")
        if not cfg.SATURATION_BOUNCE:
            parts.append("no-bounce")
        return "  ".join(parts)


def run_interactive(sim: SwarmSimulation, show_vectors: bool = False, show_communication: bool = False,
                    on_step: Optional[Callable[[], None]] = None, fps: int = 60) -> None:
    """Apre la finestra e fa avanzare `sim` finché l'utente non chiude. `on_step` è chiamato dopo ogni step."""
    renderer = Renderer(sim, show_vectors, show_communication)
    pg = renderer.pg
    clock = pg.time.Clock()
    paused = False
    try:
        while True:
            for event in pg.event.get():
                if event.type == pg.QUIT or (event.type == pg.KEYDOWN and event.key == pg.K_q):
                    return
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_p:
                        paused = not paused
                    elif event.key == pg.K_v:
                        renderer.show_force_vectors = not renderer.show_force_vectors
                    elif event.key == pg.K_c:
                        renderer.show_communication = not renderer.show_communication
            if not paused:
                sim.step()
                if on_step is not None:
                    on_step()
            renderer.draw_scene(paused)
            clock.tick(fps)
    finally:
        renderer.close()
