#!/usr/bin/env python3
"""
Interactive Visualizer for the SO-100 Robot Arm using MeshCat.
Phase 3 of the MVP.

The user controls the end-effector goal position via browser sliders.
The arm animates toward the IK solution in real-time. A goal marker
(green/red sphere) shows the target; a blue sphere shows the actual
end-effector position computed by forward kinematics.

Architecture:
  Python side  -- Tornado HTTP+WS server, MeshCat visualizer, animation loop.
  Browser side -- wrapper HTML page embedding MeshCat in an iframe with
                  X/Y/Z sliders that send values over WebSocket.
"""

import argparse
import json
import time
import xml.etree.ElementTree as ET
from typing import List, Dict, Optional, Set

import numpy as np
import meshcat
import meshcat.geometry as geom
import tornado.ioloop
import tornado.web
import tornado.websocket
from yourdfpy import URDF

from ik_solver import SO100IKSolver


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

def compute_workspace_radius(urdf_path: str) -> float:
    """
    Estimate the arm's maximum reach by summing joint-origin offsets from the
    kinematics URDF.  The first joint (shoulder_pan) is a base-height offset
    and is excluded.  A conservative 0.85 factor is applied since joint limits
    prevent full extension in all directions.
    """
    tree: ET.ElementTree = ET.parse(urdf_path)
    root: ET.Element = tree.getroot()

    total_reach: float = 0.0
    skip_first: bool = True
    for joint_elem in root.findall("joint"):
        origin: Optional[ET.Element] = joint_elem.find("origin")
        if origin is not None:
            xyz_str: str = origin.get("xyz", "0 0 0")
            xyz: List[float] = [float(v) for v in xyz_str.split()]
            link_length: float = float(np.linalg.norm(xyz))
            if skip_first:
                skip_first = False
                continue
            total_reach += link_length

    if total_reach <= 0.0:
        # Fallback if parsing yields nothing useful
        return 0.35
    return total_reach * 0.85


def validate_ik_solution(
    solver: SO100IKSolver,
    joint_angles_rad: List[float],
    target_position: List[float],
    tolerance: float = 0.01,
) -> bool:
    """
    Check whether an IK result actually reaches the target by running forward
    kinematics and comparing positions.  lerobot's IK does not raise on
    failure -- it silently returns the last iteration, so FK validation is
    the only reliable check.

    Uses solver.kinematics.forward_kinematics (expects degrees).
    """
    joint_angles_deg: np.ndarray = np.rad2deg(np.array(joint_angles_rad))
    fk_result: np.ndarray = solver.kinematics.forward_kinematics(
        joint_pos_deg=joint_angles_deg
    )
    actual_pos: np.ndarray = fk_result[:3, 3]
    error: float = float(np.linalg.norm(actual_pos - np.array(target_position)))
    return error < tolerance


def interpolate_joints(
    current: np.ndarray, target: np.ndarray, alpha: float
) -> np.ndarray:
    """Linear interpolation in joint-space, alpha clamped to [0, 1]."""
    alpha = max(0.0, min(1.0, alpha))
    return current + alpha * (target - current)


# ---------------------------------------------------------------------------
# MeshCat rendering helpers
# ---------------------------------------------------------------------------

def init_arm_visuals(
    vis: meshcat.Visualizer, urdf_model: URDF, configuration: Dict[str, float]
) -> None:
    """
    First-time render: send geometry (set_object) and transforms for every
    node in the URDF scene graph.  Call once at startup.

    Color logic matches Phase 2: dark gray for motors, orange for structural parts.
    """
    urdf_model.update_cfg(configuration)

    for node_name in urdf_model.scene.graph.nodes:
        transform, _ = urdf_model.scene.graph.get(node_name)
        geom_names = urdf_model.scene.graph.geometry_nodes.get(node_name)

        if geom_names:
            if not isinstance(geom_names, (list, tuple)):
                geom_names = [geom_names]

            for idx, geom_name in enumerate(geom_names):
                geometry = urdf_model.scene.geometry.get(geom_name)
                if hasattr(geometry, "vertices") and hasattr(geometry, "faces"):
                    hex_color: int = 0xCCCCCC
                    if "Motor" in geom_name:
                        hex_color = 0x1A1A1A
                    elif any(
                        part in geom_name
                        for part in ["Base", "Rotation", "Arm", "Jaw", "Wrist"]
                    ):
                        hex_color = 0xFFD11E

                    material = geom.MeshLambertMaterial(color=hex_color)
                    vis[node_name][f"mesh_{idx}"].set_object(
                        geom.TriangularMeshGeometry(
                            vertices=geometry.vertices,
                            faces=geometry.faces,
                        ),
                        material,
                    )

        vis[node_name].set_transform(transform)


def update_arm_transforms(
    vis: meshcat.Visualizer, urdf_model: URDF, configuration: Dict[str, float]
) -> None:
    """
    Update transforms only (no geometry re-send).  Called every animation tick.
    """
    urdf_model.update_cfg(configuration)

    for node_name in urdf_model.scene.graph.nodes:
        transform, _ = urdf_model.scene.graph.get(node_name)
        vis[node_name].set_transform(transform)


def update_goal_marker(
    vis: meshcat.Visualizer, position: List[float], reachable: bool
) -> None:
    """
    Place a semi-transparent sphere at the goal position.
    Green if reachable, red if not.
    """
    color: int = 0x00FF00 if reachable else 0xFF0000
    material = geom.MeshPhongMaterial(color=color, opacity=0.45, transparent=True)
    vis["goal_marker"].set_object(geom.Sphere(0.015), material)

    tf: np.ndarray = np.eye(4)
    tf[0, 3] = position[0]
    tf[1, 3] = position[1]
    tf[2, 3] = position[2]
    vis["goal_marker"].set_transform(tf)


def update_ee_marker(
    vis: meshcat.Visualizer,
    solver: SO100IKSolver,
    joint_angles_rad: List[float],
) -> None:
    """
    Place a smaller blue sphere at the actual end-effector position
    obtained via forward kinematics.  Uses solver.kinematics.forward_kinematics
    (expects degrees).
    """
    joint_angles_deg: np.ndarray = np.rad2deg(np.array(joint_angles_rad))
    fk_result: np.ndarray = solver.kinematics.forward_kinematics(
        joint_pos_deg=joint_angles_deg
    )
    ee_pos: np.ndarray = fk_result[:3, 3]

    material = geom.MeshPhongMaterial(color=0x0088FF, opacity=0.6, transparent=True)
    vis["ee_marker"].set_object(geom.Sphere(0.01), material)

    tf: np.ndarray = np.eye(4)
    tf[0, 3] = float(ee_pos[0])
    tf[1, 3] = float(ee_pos[1])
    tf[2, 3] = float(ee_pos[2])
    vis["ee_marker"].set_transform(tf)


# ---------------------------------------------------------------------------
# Tornado handlers
# ---------------------------------------------------------------------------

# Shared state written by WebSocket handler, read by animation tick.
# Using a plain dict is safe because Tornado is single-threaded (IOLoop).
_shared_state: Dict[str, float] = {"x": 0.15, "y": 0.0, "z": 0.15}
_ws_clients: Set[tornado.websocket.WebSocketHandler] = set()


class SliderHandler(tornado.websocket.WebSocketHandler):
    """Receives slider value updates from the browser and stores them."""

    def check_origin(self, origin: str) -> bool:
        # Allow connections from any origin (local dev)
        return True

    def open(self) -> None:
        _ws_clients.add(self)

    def on_message(self, message: str) -> None:
        try:
            data: Dict[str, float] = json.loads(message)
            if "x" in data:
                _shared_state["x"] = float(data["x"])
            if "y" in data:
                _shared_state["y"] = float(data["y"])
            if "z" in data:
                _shared_state["z"] = float(data["z"])
        except (json.JSONDecodeError, ValueError):
            pass

    def on_close(self) -> None:
        _ws_clients.discard(self)


def _broadcast_status(reachable: bool, ee_pos: List[float]) -> None:
    """Send reachability and actual EE position back to all browser clients."""
    msg: str = json.dumps({
        "reachable": reachable,
        "ee_x": round(ee_pos[0], 5),
        "ee_y": round(ee_pos[1], 5),
        "ee_z": round(ee_pos[2], 5),
    })
    for client in _ws_clients:
        try:
            client.write_message(msg)
        except tornado.websocket.WebSocketClosedError:
            pass


def _build_wrapper_html(meshcat_url: str, ws_port: int, radius: float) -> str:
    """
    Return the wrapper HTML page as a string.

    Contains: MeshCat iframe, X/Y/Z sliders, status indicator, and
    a WebSocket connection to our Tornado server.
    """
    # Slider step: 1mm precision
    step: float = 0.001
    # Default values
    dx: float = 0.15
    dy: float = 0.0
    dz: float = 0.15
    # Ranges
    r: float = round(radius, 4)
    z_min: float = round(-0.05, 4)

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>SO-100 Interactive Visualizer</title>
<style>
  * {{ margin: 0; padding: 0; box-sizing: border-box; }}
  body {{ display: flex; height: 100vh; font-family: monospace; background: #1a1a1a; color: #eee; }}
  #viewer {{ flex: 1; border: none; }}
  #panel {{
    width: 280px; padding: 16px; display: flex; flex-direction: column; gap: 14px;
    background: #242424; border-left: 1px solid #444; overflow-y: auto;
  }}
  h2 {{ font-size: 14px; color: #FFD11E; margin-bottom: 4px; }}
  .slider-group {{ display: flex; flex-direction: column; gap: 4px; }}
  .slider-group label {{ font-size: 12px; display: flex; justify-content: space-between; }}
  .slider-group input[type=range] {{ width: 100%; accent-color: #FFD11E; }}
  #status {{
    padding: 8px; text-align: center; font-weight: bold; font-size: 13px;
    border-radius: 4px;
  }}
  .reachable {{ background: #0a3d0a; color: #4f4; }}
  .unreachable {{ background: #3d0a0a; color: #f44; }}
  #ee-info {{ font-size: 11px; color: #aaa; }}
  #legend {{ font-size: 11px; color: #777; margin-top: auto; line-height: 1.6; }}
</style>
</head>
<body>
  <iframe id="viewer" src="{meshcat_url}"></iframe>
  <div id="panel">
    <h2>SO-100 Controls</h2>

    <div class="slider-group">
      <label>X <span id="xVal">{dx}</span></label>
      <input type="range" id="sx" min="{-r}" max="{r}" step="{step}" value="{dx}"/>
    </div>
    <div class="slider-group">
      <label>Y <span id="yVal">{dy}</span></label>
      <input type="range" id="sy" min="{-r}" max="{r}" step="{step}" value="{dy}"/>
    </div>
    <div class="slider-group">
      <label>Z <span id="zVal">{dz}</span></label>
      <input type="range" id="sz" min="{z_min}" max="{r}" step="{step}" value="{dz}"/>
    </div>

    <div id="status" class="reachable">Reachable</div>
    <div id="ee-info">Actual EE: --</div>

    <div id="legend">
      Green sphere = goal position<br/>
      Blue sphere = actual end-effector
    </div>
  </div>

<script>
  const ws = new WebSocket("ws://" + location.hostname + ":{ws_port}/ws");

  const sx = document.getElementById("sx");
  const sy = document.getElementById("sy");
  const sz = document.getElementById("sz");
  const xVal = document.getElementById("xVal");
  const yVal = document.getElementById("yVal");
  const zVal = document.getElementById("zVal");
  const statusEl = document.getElementById("status");
  const eeInfo = document.getElementById("ee-info");

  function sendSliders() {{
    const msg = JSON.stringify({{
      x: parseFloat(sx.value),
      y: parseFloat(sy.value),
      z: parseFloat(sz.value)
    }});
    if (ws.readyState === WebSocket.OPEN) {{
      ws.send(msg);
    }}
    xVal.textContent = parseFloat(sx.value).toFixed(3);
    yVal.textContent = parseFloat(sy.value).toFixed(3);
    zVal.textContent = parseFloat(sz.value).toFixed(3);
  }}

  sx.addEventListener("input", sendSliders);
  sy.addEventListener("input", sendSliders);
  sz.addEventListener("input", sendSliders);

  ws.onmessage = function(evt) {{
    const data = JSON.parse(evt.data);
    if (data.reachable) {{
      statusEl.textContent = "Reachable";
      statusEl.className = "reachable";
    }} else {{
      statusEl.textContent = "Unreachable";
      statusEl.className = "unreachable";
    }}
    eeInfo.textContent = "Actual EE: ("
      + data.ee_x.toFixed(4) + ", "
      + data.ee_y.toFixed(4) + ", "
      + data.ee_z.toFixed(4) + ")";
  }};
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# CLI and main
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description="SO-100 Arm Interactive Visualizer"
    )
    parser.add_argument(
        "--urdf", type=str, default="models/so100.urdf",
        help="Path to the original URDF with meshes",
    )
    parser.add_argument(
        "--ik-urdf", type=str, default="models/so100_kinematics.urdf",
        help="Path to the kinematics-only URDF for the solver",
    )
    parser.add_argument(
        "--speed", type=float, default=3.0,
        help="Animation speed factor (higher = faster convergence)",
    )
    parser.add_argument(
        "--tolerance", type=float, default=0.03,
        help="IK validation tolerance in meters (default 3cm)",
    )
    parser.add_argument(
        "--port", type=int, default=7001,
        help="Port for the control-panel HTTP/WS server",
    )
    return parser.parse_args()


def main() -> None:
    args: argparse.Namespace = parse_args()

    # -- IK solver --
    print(f"Initializing IK Solver with {args.ik_urdf}...")
    solver: SO100IKSolver = SO100IKSolver(urdf_path=args.ik_urdf)
    joint_names: List[str] = solver.kinematics.joint_names
    num_joints: int = len(joint_names)

    # -- MeshCat visualizer --
    print("Initializing MeshCat visualizer...")
    vis: meshcat.Visualizer = meshcat.Visualizer()
    meshcat_url: str = vis.url()
    print(f"MeshCat internal URL: {meshcat_url}")

    # -- URDF model for visualization --
    print(f"Loading URDF visuals from {args.urdf}...")
    urdf_model: URDF = URDF.load(args.urdf)

    # -- Workspace bounds --
    workspace_radius: float = compute_workspace_radius(args.ik_urdf)
    print(f"Computed workspace radius: {workspace_radius:.4f} m")

    # -- Initial state --
    current_joints: np.ndarray = np.zeros(num_joints, dtype=np.float64)
    target_joints: np.ndarray = np.zeros(num_joints, dtype=np.float64)
    last_target_pos: List[float] = [0.0, 0.0, 0.0]
    reachable: bool = True

    # Initial render at zero configuration
    zero_config: Dict[str, float] = {name: 0.0 for name in joint_names}
    init_arm_visuals(vis, urdf_model, zero_config)
    update_goal_marker(vis, [_shared_state["x"], _shared_state["y"], _shared_state["z"]], True)
    update_ee_marker(vis, solver, current_joints.tolist())

    # -- Animation tick --
    last_tick_time: float = time.time()

    def _tick() -> None:
        nonlocal current_joints, target_joints, last_target_pos, reachable
        nonlocal last_tick_time

        now: float = time.time()
        dt: float = now - last_tick_time
        last_tick_time = now

        # Read goal from shared state (set by WebSocket handler)
        target_pos: List[float] = [
            _shared_state["x"],
            _shared_state["y"],
            _shared_state["z"],
        ]

        # Clamp to workspace bounds
        r: float = workspace_radius
        target_pos[0] = max(-r, min(r, target_pos[0]))
        target_pos[1] = max(-r, min(r, target_pos[1]))
        target_pos[2] = max(-0.05, min(r, target_pos[2]))

        # Only re-solve IK if the target moved (threshold 0.5mm)
        pos_delta: float = float(np.linalg.norm(
            np.array(target_pos) - np.array(last_target_pos)
        ))
        if pos_delta > 0.0005:
            last_target_pos = list(target_pos)
            try:
                ik_result: List[float] = solver.calculate_ik(
                    target_pos,
                    initial_joint_pos_rad=current_joints.tolist(),
                )
                reachable = validate_ik_solution(
                    solver, ik_result, target_pos, args.tolerance
                )
                # Always update target_joints with the solver's best attempt.
                # The arm animates toward it regardless; the goal marker color
                # indicates whether the target was precisely reached.
                target_joints = np.array(ik_result, dtype=np.float64)
            except Exception:
                reachable = False

            update_goal_marker(vis, target_pos, reachable)

            # Compute actual EE for status broadcast
            fk_deg: np.ndarray = np.rad2deg(current_joints)
            fk_pose: np.ndarray = solver.kinematics.forward_kinematics(
                joint_pos_deg=fk_deg
            )
            ee_pos: List[float] = fk_pose[:3, 3].tolist()
            _broadcast_status(reachable, ee_pos)

        # Interpolate toward target joints
        alpha: float = min(1.0, dt * args.speed)
        current_joints = interpolate_joints(current_joints, target_joints, alpha)

        # Update arm visualization
        config: Dict[str, float] = {}
        for i, name in enumerate(joint_names):
            config[name] = float(current_joints[i])
        update_arm_transforms(vis, urdf_model, config)

        # Update actual EE marker
        update_ee_marker(vis, solver, current_joints.tolist())

    # -- Tornado HTTP + WS server --
    wrapper_html: str = _build_wrapper_html(meshcat_url, args.port, workspace_radius)

    class IndexHandler(tornado.web.RequestHandler):
        """Serves the wrapper HTML page."""
        def get(self) -> None:
            self.write(wrapper_html)

    app: tornado.web.Application = tornado.web.Application([
        (r"/", IndexHandler),
        (r"/ws", SliderHandler),
    ])
    app.listen(args.port)

    # Register the animation tick (~30 Hz)
    loop: tornado.ioloop.IOLoop = tornado.ioloop.IOLoop.current()
    tick_callback: tornado.ioloop.PeriodicCallback = tornado.ioloop.PeriodicCallback(
        _tick, 33
    )
    tick_callback.start()

    print(f"\nOpen this URL in your browser: http://localhost:{args.port}")
    print("Press Ctrl+C to exit.\n")

    try:
        loop.start()
    except KeyboardInterrupt:
        print("Exiting...")
        tick_callback.stop()
        loop.stop()


if __name__ == "__main__":
    main()
