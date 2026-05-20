import math
from datetime import datetime

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.screen import ModalScreen
from textual.widgets import Footer, Header, Input, Label, RichLog, Static


class GotoModal(ModalScreen):
    """Modal that collects a 'x y z yaw_deg' line for a GOTO command."""

    BINDINGS = [Binding("escape", "dismiss", "Cancel", show=True)]

    def compose(self) -> ComposeResult:
        with Vertical(id="goto-modal"):
            yield Label("Goto target — enter:  x y z yaw_deg   (NED meters, yaw in degrees)")
            yield Input(placeholder="e.g.  2.0 1.0 -3.0 0", id="goto-input")
            yield Label("Enter to confirm, Esc to cancel", id="goto-hint")

    def on_input_submitted(self, event: Input.Submitted) -> None:
        parts = event.value.strip().split()
        if len(parts) != 4:
            self.app.log_event("ERR: goto needs 4 numbers (x y z yaw_deg)")
            self.dismiss(None)
            return
        try:
            x, y, z, yaw = (float(p) for p in parts)
        except ValueError:
            self.app.log_event("ERR: goto values must be numeric")
            self.dismiss(None)
            return
        self.dismiss((x, y, z, yaw))


class TakeoffModal(ModalScreen):
    BINDINGS = [Binding("escape", "dismiss", "Cancel", show=True)]

    def compose(self) -> ComposeResult:
        with Vertical(id="goto-modal"):
            yield Label("Takeoff altitude AGL (meters):")
            yield Input(placeholder="e.g.  3.0", id="takeoff-input", value="3.0")
            yield Label("Enter to confirm, Esc to cancel")

    def on_input_submitted(self, event: Input.Submitted) -> None:
        try:
            alt = float(event.value.strip())
        except ValueError:
            self.app.log_event("ERR: takeoff altitude must be a number")
            self.dismiss(None)
            return
        self.dismiss(alt)


class StatePanel(Static):
    """Live state table on the left."""

    def update_state(self, msg, nav_states, arming_states, battery_warnings) -> None:
        if msg is None:
            self.update("[dim]waiting for /tui/state…[/]")
            return

        height = -msg.z  # NED -> +up
        roll_deg = math.degrees(msg.roll)
        pitch_deg = math.degrees(msg.pitch)
        yaw_deg = math.degrees(msg.yaw)

        nav_name = nav_states.get(int(msg.nav_state), f"NAV_{msg.nav_state}")
        arm_name = arming_states.get(int(msg.arming_state), f"ARM_{msg.arming_state}")
        bat_warn = battery_warnings.get(int(msg.battery_warning), str(msg.battery_warning))

        remaining_pct = (msg.battery_remaining * 100.0) if msg.battery_remaining >= 0 else float("nan")

        goto_line = (
            f"goto:  ({msg.goto_x:+.2f}, {msg.goto_y:+.2f}, {msg.goto_z:+.2f}) "
            f"yaw={math.degrees(msg.goto_yaw):+.1f}°"
            if msg.goto_active
            else "goto:  [dim]inactive[/]"
        )

        body = f"""[b]Battery[/b]
  voltage     {msg.battery_voltage:>6.2f} V
  remaining   {remaining_pct:>6.1f} %
  warning     {bat_warn}

[b]Pose (NED)[/b]
  x  {msg.x:+7.2f} m
  y  {msg.y:+7.2f} m
  z  {msg.z:+7.2f} m   alt {height:+.2f} m
  roll  {roll_deg:+7.1f}°
  pitch {pitch_deg:+7.1f}°
  yaw   {yaw_deg:+7.1f}°

[b]Status[/b]
  arming     {arm_name}
  nav_state  {nav_name}
  offboard   {"YES" if msg.offboard_active else "no"}
  failsafe   {"YES" if msg.failsafe else "no"}

[b]Controller[/b]
  {goto_line}
"""
        self.update(body)


class TuiApp(App):
    CSS = """
    Screen { layout: vertical; }
    #main { height: 1fr; }
    StatePanel { width: 50%; padding: 1 2; border: round $accent; }
    #log { width: 50%; padding: 0 1; border: round $accent; }
    #goto-modal { background: $panel; padding: 1 2; border: thick $accent; width: 60; height: auto; }
    """

    BINDINGS = [
        Binding("t", "takeoff", "Takeoff"),
        Binding("l", "land", "Land"),
        Binding("e", "emergency", "EMERGENCY"),
        Binding("g", "goto", "Goto"),
        Binding("h", "hold", "Hold"),
        Binding("q", "quit", "Quit"),
    ]

    def __init__(self, node, nav_states, arming_states, battery_warnings):
        super().__init__()
        self.node = node
        self.nav_states = nav_states
        self.arming_states = arming_states
        self.battery_warnings = battery_warnings

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id="main"):
            yield StatePanel(id="state")
            yield RichLog(id="log", highlight=False, markup=True, wrap=True)
        yield Footer()

    def on_mount(self) -> None:
        self.set_interval(0.1, self._refresh_state)
        self.log_event("px4_tui started — keys: t/l/e/g/h/q")

    def _refresh_state(self) -> None:
        panel = self.query_one(StatePanel)
        panel.update_state(
            self.node.latest_state,
            self.nav_states,
            self.arming_states,
            self.battery_warnings,
        )

    def log_event(self, text: str) -> None:
        log = self.query_one("#log", RichLog)
        log.write(f"[dim]{datetime.now().strftime('%H:%M:%S')}[/dim] {text}")

    # --- actions ---

    def action_takeoff(self) -> None:
        def _cb(alt):
            if alt is None:
                return
            self.node.send_takeoff(alt)
            self.log_event(f"[yellow]TAKEOFF[/] altitude={alt:.2f} m")

        self.push_screen(TakeoffModal(), _cb)

    def action_land(self) -> None:
        self.node.send_land()
        self.log_event("[yellow]LAND[/]")

    def action_emergency(self) -> None:
        self.node.send_emergency()
        self.log_event("[bold red]EMERGENCY — disarming[/]")

    def action_goto(self) -> None:
        def _cb(result):
            if result is None:
                return
            x, y, z, yaw = result
            self.node.send_goto(x, y, z, yaw)
            self.log_event(f"[green]GOTO[/] ({x:+.2f}, {y:+.2f}, {z:+.2f}) yaw={yaw:+.1f}°")

        self.push_screen(GotoModal(), _cb)

    def action_hold(self) -> None:
        self.node.send_hold()
        self.log_event("[cyan]HOLD[/] at current position")
