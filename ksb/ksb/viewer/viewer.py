"""KSB Pygame viewer — real-time playback of a SimulationResult.

Only this file and run_viewer.py are allowed to import pygame.
"""
from __future__ import annotations

import math
from typing import List, Optional, TYPE_CHECKING

import os

import numpy as np
import pygame
import pygame._freetype as _ft   # C extension — avoids pygame.font/sysfont circular import

from ksb.motion.trajectories import P, V, A
from ksb.simulation.result import SimulationResult

if TYPE_CHECKING:
    from ksb.analysis.events import SegmentEvents
    from ksb.analysis.cost import BBCostResult

# Built-in font bundled with pygame (no system font lookup needed)
_FONT_PATH = os.path.join(os.path.dirname(pygame.__file__), "freesansbold.ttf")


# ---------------------------------------------------------------------------
# Layout / colour constants
# ---------------------------------------------------------------------------
MARGIN = 16
LANE_H = 60
HUD_H  = 32
ITEM_PAD = 5          # vertical gap between item rect and lane top/bottom

BG_COLOR          = (240, 242, 245)
UPSTREAM_COLOR    = (180, 182, 186)
BUFFER_COLOR      = (160, 170, 200)
REGISTRAR_COLOR   = (130, 150, 195)   # slightly deeper blue — transition zone
DOWNSTREAM_COLOR  = (180, 182, 186)
ZONE_LINE_COLOR   = (60,  60,  70)
ZONE_LABEL_COLOR  = (60,  60,  70)

ITEM_COLORS = [(80, 140, 220), (60, 110, 190)]   # alternating blue shades
ITEM_RED    = (220,  60,  60)
ITEM_BORDER = ( 40,  40,  60)

SLOT_COLOR  = ( 30,  30,  40)

HUD_BG      = ( 30,  32,  36)
HUD_TEXT    = (220, 222, 226)
HUD_DIM     = (140, 142, 146)

FPS = 60
MAX_SPEED = 16.0
MIN_SPEED = 1.0 / 16.0


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
def _px(metres: float, ppm: float) -> int:
    return int(round(metres * ppm))


# ---------------------------------------------------------------------------
# KSBViewer
# ---------------------------------------------------------------------------
class KSBViewer:
    """Real-time pygame viewer for a KSB SimulationResult.

    Usage::

        viewer = KSBViewer(result, cfg, speed=1.0, ppm=120, events=None, cost=None)
        viewer.run()

    Parameters
    ----------
    result : SimulationResult
        Simulation output with trajectories and metrics
    cfg : dict
        Configuration dictionary (must match result)
    speed : float
        Initial playback multiplier (default 1.0)
    ppm : int
        Pixels per metre for rendering (default 120)
    events : SegmentEvents, optional
        Segment event times and kinematics (enables segment coloring)
    cost : BBCostResult, optional
        Bang-bang cost analysis (enables segment coloring)

    Controls
    --------
    SPACE       Pause / resume
    R           Reset to t_begin
    LEFT/RIGHT  Jump ±1 second
    [ / ]       Step ±1 frame
    + / =       Double playback speed (max 16×)
    -           Halve playback speed  (min 1/16×)
    ESC / close Quit
    """

    def __init__(
        self,
        result: SimulationResult,
        cfg: dict,
        speed: float = 1.0,
        ppm: int = 120,
        events: Optional[SegmentEvents] = None,
        cost: Optional[BBCostResult] = None,
    ) -> None:
        self.result = result
        self.cfg    = cfg
        self.speed  = float(speed)
        self.ppm    = int(ppm)
        self.events = events
        self.cost   = cost

        # ------------------------------------------------------------------
        # Physical dimensions from cfg
        # ------------------------------------------------------------------
        self.L_up   = float(cfg.get("L_upstream",   1.0))
        self.L_buf  = float(cfg.get("L_buffer",      2.0))
        self.L_reg  = float(cfg.get("L_registrar",   0.0))
        self.L_dn   = float(cfg.get("L_downstream",  1.0))
        self.L_tot  = self.L_up + self.L_buf + self.L_reg + self.L_dn

        self.input_length = float(cfg.get("input_length", 0.32))
        self.n_buffer_seg = int(cfg.get("n_buffer_seg", 5))
        self.n_reg_seg    = int(cfg.get("n_reg_seg", 1))

        # ------------------------------------------------------------------
        # Simulation timing
        # ------------------------------------------------------------------
        t_spawn = np.asarray(result.t_spawn, dtype=float)
        self.t_begin = float(t_spawn[0])
        self.t_end   = float(max(
            t_spawn[i] + result.composite_trajectories[i].T
            for i in range(len(t_spawn))
        ))

        # ------------------------------------------------------------------
        # Physics
        # ------------------------------------------------------------------
        # gap_min matches KSBSimulation.min_gap_on_buffer
        self.gap_min = self.L_buf / self.n_buffer_seg * 2

        slot_rate_ppm = float(cfg.get("slot_rate_ppm", 180))
        slot_gap    = float(cfg.get("slot_length",    0.40))
        self.vd          = (slot_rate_ppm / 60.0) * slot_gap
        self.slot_period = slot_gap / self.vd

        # ------------------------------------------------------------------
        # HUD precomputed stats
        # ------------------------------------------------------------------
        self.n_items      = len(result.t_spawn)
        self.n_skips      = len(result.skip_indices)
        self.n_violations = sum(
            1 for p in result.pair_records
            if p.min_gap is not None and p.min_gap < self.gap_min
        )
        self.solver_name = cfg.get("solver_name", "")

        # ------------------------------------------------------------------
        # Window geometry (pixels)
        # ------------------------------------------------------------------
        belt_w = _px(self.L_tot, self.ppm)
        self.win_w = MARGIN * 2 + belt_w
        self.win_h = MARGIN * 2 + LANE_H + HUD_H

        # Lane rect (the horizontal belt strip)
        self.lane = pygame.Rect(MARGIN, MARGIN, belt_w, LANE_H)

        # Zone pixel edges (left x of each zone boundary)
        self.x_buf_start = MARGIN + _px(self.L_up,                          self.ppm)
        self.x_reg_start = MARGIN + _px(self.L_up + self.L_buf,             self.ppm)
        self.x_dn_start  = MARGIN + _px(self.L_up + self.L_buf + self.L_reg, self.ppm)
        self.x_right     = MARGIN + belt_w

        # HUD rect
        hud_top = MARGIN + LANE_H
        self.hud_rect = pygame.Rect(0, hud_top, self.win_w, HUD_H + MARGIN)

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------
    def run(self) -> None:
        pygame.init()
        try:
            screen = pygame.display.set_mode((self.win_w, self.win_h))
            pygame.display.set_caption("KSB Viewer")

            _ft.init()
            font_small = _ft.Font(_FONT_PATH, 13)
            font_label = _ft.Font(_FONT_PATH, 11)
            clock = pygame.time.Clock()

            t   = self.t_begin
            paused = True

            while True:
                dt_ms = clock.tick(FPS)
                dt    = (dt_ms / 1000.0) * self.speed

                # ── Event handling ─────────────────────────────────────
                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        return
                    if ev.type == pygame.KEYDOWN:
                        if ev.key == pygame.K_ESCAPE:
                            return
                        elif ev.key == pygame.K_SPACE:
                            paused = not paused
                        elif ev.key == pygame.K_r:
                            t = self.t_begin
                            paused = True
                        elif ev.key in (pygame.K_LEFT, pygame.K_j):
                            t = max(self.t_begin, t - 1.0)
                        elif ev.key in (pygame.K_RIGHT, pygame.K_l):
                            t = min(self.t_end, t + 1.0)
                        elif ev.key in (pygame.K_LEFTBRACKET,):
                            t = max(self.t_begin, t - 1.0 / FPS)
                        elif ev.key in (pygame.K_RIGHTBRACKET,):
                            t = min(self.t_end, t + 1.0 / FPS)
                        elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS):
                            self.speed = min(MAX_SPEED, self.speed * 2.0)
                        elif ev.key == pygame.K_MINUS:
                            self.speed = max(MIN_SPEED, self.speed / 2.0)

                # ── Advance time ────────────────────────────────────────
                if not paused:
                    t += dt
                    t = min(t, self.t_end)

                # ── Draw ────────────────────────────────────────────────
                self._draw(screen, font_small, font_label, t)
                pygame.display.flip()

        finally:
            pygame.quit()

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------
    def _draw(
        self,
        screen: pygame.Surface,
        font_small: _ft.Font,
        font_label: _ft.Font,
        t: float,
    ) -> None:
        screen.fill(BG_COLOR)
        self._draw_zones(screen, font_small)
        self._draw_slot_separators(screen, t)

        item_rects, item_colors = self._compute_items(t)

        # Items
        for i, rect in enumerate(item_rects):
            if rect is None:
                continue
            color = item_colors[i]
            pygame.draw.rect(screen, color, rect, border_radius=3)
            pygame.draw.rect(screen, ITEM_BORDER, rect, width=1, border_radius=3)
            self._draw_label(screen, font_label, f"{i+1}", rect)

        self._draw_hud(screen, font_small, t)

    def _get_buffer_segment_color(self, segment_idx: int) -> tuple:
        """Compute color for a buffer segment. Override based on events/cost here."""
        return BUFFER_COLOR

    def _draw_zones(
        self,
        screen: pygame.Surface,
        font: _ft.Font,
    ) -> None:
        lane = self.lane
        y, h = lane.top, lane.height

        # Upstream zone
        r_up = pygame.Rect(lane.left, y, self.x_buf_start - lane.left, h)
        pygame.draw.rect(screen, UPSTREAM_COLOR, r_up)

        # Buffer zone — draw individual segments as rectangles
        section_len = self.L_buf / self.n_buffer_seg
        for k in range(self.n_buffer_seg):
            px_left = MARGIN + _px(self.L_up + k * section_len, self.ppm)
            px_right = MARGIN + _px(self.L_up + (k + 1) * section_len, self.ppm)
            segment_w = px_right - px_left
            seg_rect = pygame.Rect(px_left, y, segment_w, h)
            seg_color = self._get_buffer_segment_color(k)
            pygame.draw.rect(screen, seg_color, seg_rect)
            # Draw segment border
            pygame.draw.rect(screen, ZONE_LINE_COLOR, seg_rect, width=1)

        # Registrar zone (may be zero-width if L_reg == 0)
        if self.x_dn_start > self.x_reg_start:
            r_reg = pygame.Rect(self.x_reg_start, y, self.x_dn_start - self.x_reg_start, h)
            pygame.draw.rect(screen, REGISTRAR_COLOR, r_reg)
            # Registrar section dividers
            if self.n_reg_seg > 1:
                reg_section_len = self.L_reg / self.n_reg_seg
                reg_div_color = (90, 100, 155)
                for k in range(1, self.n_reg_seg):
                    px = MARGIN + _px(self.L_up + self.L_buf + k * reg_section_len, self.ppm)
                    pygame.draw.line(screen, reg_div_color, (px, y + 4), (px, y + h - 4), 1)

        # Downstream zone
        r_dn = pygame.Rect(self.x_dn_start, y, self.x_right - self.x_dn_start, h)
        pygame.draw.rect(screen, DOWNSTREAM_COLOR, r_dn)

        # Outer border
        pygame.draw.rect(screen, ZONE_LINE_COLOR, lane, width=1)

        # Zone boundary lines
        for bx in (self.x_buf_start, self.x_reg_start, self.x_dn_start):
            pygame.draw.line(screen, ZONE_LINE_COLOR, (bx, y), (bx, y + h), 1)

        # Zone labels
        labels = [
            ("upstream",   (lane.left + self.x_buf_start) // 2),
            ("buffer",     (self.x_buf_start + self.x_reg_start) // 2),
            ("downstream", (self.x_dn_start + self.x_right) // 2),
        ]
        if self.x_dn_start > self.x_reg_start:
            labels.append(("registrar", (self.x_reg_start + self.x_dn_start) // 2))
        for text, cx in labels:
            surf, _ = font.render(text, ZONE_LABEL_COLOR)
            screen.blit(surf, surf.get_rect(centerx=cx, top=y + 3))

    def _draw_slot_separators(self, screen: pygame.Surface, t: float) -> None:
        """Tile moving vertical lines across the downstream zone."""
        x_left  = self.x_dn_start
        x_right = self.x_right
        y       = self.lane.top
        h       = self.lane.height

        if self.slot_period <= 0 or self.vd <= 0:
            return

        k_min = math.ceil((t - self.L_dn / self.vd) / self.slot_period)
        k_max = math.floor(t / self.slot_period)

        dn_origin = self.L_up + self.L_buf + self.L_reg   # B^{RD}
        for k in range(k_min, k_max + 1):
            pos = dn_origin + self.vd * (t - k * self.slot_period)
            if pos < dn_origin - 1e-9 or pos > self.L_tot + 1e-9:
                continue
            px = MARGIN + _px(pos, self.ppm)
            pygame.draw.line(screen, SLOT_COLOR, (px, y + 2), (px, y + h - 2), 2)

    def _compute_items(
        self,
        t: float,
    ):
        """Return parallel lists of (rect | None) and color for each item."""
        result = self.result
        n = self.n_items
        t_spawn = np.asarray(result.t_spawn, dtype=float)

        rects:  List[Optional[pygame.Rect]] = [None] * n
        colors: List                        = [ITEM_COLORS[i % 2] for i in range(n)]

        p_leads: List[Optional[float]]      = [None] * n
        p_trails: List[Optional[float]]     = [None] * n

        X: List[Optional[float]]     = [None] * n

        # First pass: positions
        for i, traj in enumerate(result.composite_trajectories):
            t_local = t - t_spawn[i]
            if t_local < 0.0 or t_local > traj.T:
                continue
            state  = traj.eval(t_local)   # shape (3,)
            p_lead = float(state[P])
            p_trail = p_lead - self.input_length

            px_trail = MARGIN + _px(p_trail, self.ppm)
            px_lead  = MARGIN + _px(p_lead,  self.ppm)
            rect_w   = max(2, px_lead - px_trail)

            rects[i]   = pygame.Rect(
                px_trail,
                self.lane.top + ITEM_PAD,
                rect_w,
                self.lane.height - 2 * ITEM_PAD,
            )
            p_leads[i] = p_lead
            p_trails[i] = p_lead - self.input_length

            X[i] = state

        return rects, colors

    def _draw_gap_overlays(
        self,
        screen: pygame.Surface,
        rects: list,
        colors: list,
    ) -> None:
        """Faint red band between consecutive pairs flagged red (buffer + registrar zones)."""
        n = self.n_items
        for i in range(n - 1):
            if rects[i] is None or rects[i + 1] is None:
                continue
            if colors[i] is not ITEM_RED:
                continue
            # Clamp overlay to the buffer zone in x
            x_buf_left  = self.x_buf_start
            x_buf_right = self.x_dn_start

            # Band spans from trailing item's left to leading item's right,
            # clamped to the buffer zone
            r_a, r_b = rects[i + 1], rects[i]   # i+1 is trailing (smaller x)
            x_left  = max(r_a.left,  x_buf_left)
            x_right = min(r_b.right, x_buf_right)
            band_w  = max(1, x_right - x_left)
            if x_left >= x_right:
                continue

            overlay = pygame.Surface((band_w, self.lane.height), pygame.SRCALPHA)
            overlay.fill((220, 60, 60, 55))
            screen.blit(overlay, (x_left, self.lane.top))

    @staticmethod
    def _draw_label(
        screen: pygame.Surface,
        font: _ft.Font,
        text: str,
        rect: pygame.Rect,
    ) -> None:
        surf, _ = font.render(text, (255, 255, 255))
        if surf.get_width() + 4 <= rect.width:
            pos = surf.get_rect(center=rect.center)
        else:
            pos = surf.get_rect(midbottom=(rect.centerx, rect.top - 1))
        screen.blit(surf, pos)

    def _draw_hud(
        self,
        screen: pygame.Surface,
        font: _ft.Font,
        t: float,
    ) -> None:
        pygame.draw.rect(screen, HUD_BG, self.hud_rect)

        parts = [
            f"t = {t:7.3f} s",
            f"{self.speed:g}x",
            self.solver_name or "-",
            f"items: {self.n_items}  skips: {self.n_skips}  violations: {self.n_violations}",
        ]
        text = "   |   ".join(parts)
        surf, _ = font.render(text, HUD_TEXT)
        screen.blit(surf, surf.get_rect(
            midleft=(MARGIN, self.hud_rect.centery)
        ))
