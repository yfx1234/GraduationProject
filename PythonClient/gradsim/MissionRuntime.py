import copy
import time

import numpy as np

from .AgentDrone import AgentDrone, DEFAULT_DRONE_CLASS
from .AgentGuidance import AgentGuidance
from .VisualIntercept import VisualInterceptView
from .client import TCPClient


class DroneMissionRuntime:
    def __init__(self, config, view=None):
        self.config = copy.deepcopy(config)
        self.runtime = None
        self.created_guidance = False
        self.created_interceptor = False
        self.created_target = False
        self.visual_session_started = False
        self.exit_reason = "finished"
        self.view = view
        self._apply_defaults()

    def _apply_defaults(self):
        self.config.setdefault("connection", {})
        self.config.setdefault("interceptor", {})
        self.config.setdefault("target", {})
        self.config.setdefault("guidance", {})
        self.config.setdefault("runtime", {})
        self.config.setdefault("view", {})

        connection = self.config["connection"]
        interceptor = self.config["interceptor"]
        target = self.config["target"]
        guidance = self.config["guidance"]
        runtime = self.config["runtime"]
        view = self.config["view"]

        connection.setdefault("host", "127.0.0.1")
        connection.setdefault("port", 9000)
        connection.setdefault("timeout", 10.0)

        self.config.setdefault("drone_class", DEFAULT_DRONE_CLASS)

        interceptor.setdefault("id", "drone_0")
        interceptor.setdefault("label", "Interceptor")
        interceptor.setdefault("role", "interceptor")
        interceptor.setdefault("spawn", [0.0, 0.0, 1.0])
        interceptor.setdefault("yaw", 0.0)
        interceptor.setdefault("altitude", 7.0)

        target.setdefault("id", "drone_1")
        target.setdefault("label", "Target")
        target.setdefault("role", "target")
        target.setdefault("spawn", [35.0, 12.0, 1.0])
        target.setdefault("yaw", 180.0)
        target.setdefault("altitude", 8.0)
        target.setdefault("trajectory", {})
        target["trajectory"].setdefault("kind", "circle")
        target["trajectory"].setdefault("speed", 6.0)
        target["trajectory"].setdefault("turn_rate", 0.30)
        target["trajectory"].setdefault("vertical_amp", 0.0)
        target["trajectory"].setdefault("lead_time", 5.0)

        guidance.setdefault("id", "guidance_0")
        guidance.setdefault("classname", "GuidanceActor")
        guidance.setdefault("label", "Guidance")

        runtime.setdefault("hz", 12.0)
        runtime.setdefault("max_time", 120.0)
        runtime.setdefault("telemetry_interval", 0.5)
        runtime.setdefault("show", True)
        runtime.setdefault("clean_existing", True)
        runtime.setdefault("keep_actors", False)
        runtime.setdefault("max_transient_errors", 3)

        view.setdefault("window_name", "auto_spawn_visual_intercept")
        if self.view is None:
            self.view = VisualInterceptView(runtime["show"], view["window_name"])

    def _ok(self, response):
        return AgentDrone.is_ok(response)

    def _actor_ids(self):
        return {
            "guidance": self.config["guidance"]["id"],
            "interceptor": self.config["interceptor"]["id"],
            "target": self.config["target"]["id"],
        }

    def _runtime_actors(self):
        return (
            ("target", self.created_target, self.runtime["target"]),
            ("interceptor", self.created_interceptor, self.runtime["interceptor"]),
            ("guidance", self.created_guidance, self.runtime["guidance"]),
        )

    def _runtime_drones(self):
        return self._runtime_actors()[:2]

    def _ensure_removed(self, actor_id):
        response = AgentDrone.unwrap_response(
            self.runtime["client"].request({"remove_actor": {"actor_id": actor_id}}),
            "remove_actor_return",
        )
        if self._ok(response):
            print(f"[INFO] removed existing actor: {actor_id}")

    def _spawn_drone_with_class(self, drone, drone_config, classname):
        spawn = drone_config["spawn"]
        return drone.create_raw(
            spawn_x=spawn[0],
            spawn_y=spawn[1],
            spawn_z=spawn[2],
            yaw=drone_config.get("yaw", 0.0),
            classname=classname,
            label=drone_config["label"],
        )

    def _spawn_drone(self, drone, drone_config):
        drone.mission_role = drone_config["role"]
        drone.label = drone_config["label"]

        primary_class = self.config["drone_class"]
        response = self._spawn_drone_with_class(drone, drone_config, primary_class)
        if self._ok(response):
            drone.classname = primary_class
            return True

        if primary_class != "DronePawn":
            print(f"[WARN] spawn with '{primary_class}' failed: {response}")
            fallback_response = self._spawn_drone_with_class(drone, drone_config, "DronePawn")
            if self._ok(fallback_response):
                print("[WARN] fallback class 'DronePawn' used (may have no visible mesh).")
                drone.classname = "DronePawn"
                return True
            print(f"[ERROR] fallback spawn failed: {fallback_response}")
            return False

        print(f"[ERROR] spawn failed: {response}")
        return False

    def _drone_state(self, drone):
        response = drone.get_state(frame="ue")
        if isinstance(response, dict):
            return response
        return {}

    def _drone_pos(self, drone):
        state = self._drone_state(drone)
        pos = state.get("position", [0.0, 0.0, 0.0]) if isinstance(state, dict) else [0.0, 0.0, 0.0]
        return np.array(pos[:3], dtype=np.float64)

    def _wait_for_altitude(self, drone, target_altitude, timeout_s=20.0, tolerance_m=0.8):
        deadline = time.time() + max(1.0, timeout_s)
        while time.time() < deadline:
            state = self._drone_state(drone)
            if self._ok(state):
                pos = state.get("position", [0.0, 0.0, 0.0])
                if isinstance(pos, list) and len(pos) >= 3:
                    altitude = float(pos[2])
                    if altitude >= (target_altitude - tolerance_m):
                        return True
            time.sleep(0.25)
        return False

    def _clean_existing_actors(self):
        if not self.config["runtime"]["clean_existing"]:
            return

        for actor_id in self._actor_ids().values():
            self._ensure_removed(actor_id)
        time.sleep(0.2)

    def _spawn_requested_drones(self):
        guidance_config = self.config["guidance"]
        self.created_guidance = self.runtime["guidance"].create(
            classname=guidance_config["classname"],
            label=guidance_config["label"],
        )
        if not self.created_guidance:
            return False

        self.created_interceptor = self._spawn_drone(self.runtime["interceptor"], self.config["interceptor"])
        if not self.created_interceptor:
            return False

        self.created_target = self._spawn_drone(self.runtime["target"], self.config["target"])
        return self.created_target

    def _verify_spawned_agents(self):
        time.sleep(0.3)
        current_ids = set(self.runtime["client"].get_agents())
        actor_ids = self._actor_ids()
        missing = [actor_id for actor_id in actor_ids.values() if actor_id not in current_ids]
        if missing:
            print(f"[ERROR] spawned actors missing in registry: {missing}, current={sorted(current_ids)}")
            return False

        print(
            f"[INFO] guidance={actor_ids['guidance']}, interceptor={actor_ids['interceptor']}, target={actor_ids['target']}"
        )
        return True

    def _takeoff_pair(self):
        print("[INFO] takeoff start")
        interceptor = self.runtime["interceptor"]
        target = self.runtime["target"]
        interceptor_config = self.config["interceptor"]
        target_config = self.config["target"]

        for drone, drone_config in ((interceptor, interceptor_config), (target, target_config)):
            drone.takeoff(altitude=drone_config["altitude"])
            if self._wait_for_altitude(drone, drone_config["altitude"]):
                continue

            print(f"[ERROR] {drone_config['role']} '{drone_config['id']}' did not reach takeoff altitude")
            return False

        print("[INFO] both drones airborne")
        interceptor.set_camera_angles(0.0, 0.0)
        guidance_reset = self.runtime["guidance"].reset()
        if not self._ok(guidance_reset):
            print(f"[ERROR] guidance reset failed: {guidance_reset}")
            return False
        return True

    @staticmethod
    def _try_call(action):
        try:
            return action()
        except Exception:
            return None

    def _cleanup_runtime(self):
        if not self.runtime:
            return

        actor_ids = self._actor_ids()
        if self.created_guidance and self.visual_session_started:
            self._try_call(
                lambda: self.runtime["guidance"].visual_intercept_stop(
                    interceptor_id=actor_ids["interceptor"],
                    target_id=actor_ids["target"],
                ),
            )

        for _name, created, drone in self._runtime_drones():
            if created:
                self._try_call(drone.hover)

        self.view.close()
        should_cleanup = not self.config["runtime"]["keep_actors"]
        if should_cleanup and self.exit_reason.startswith("error:"):
            should_cleanup = False
            print("[WARN] keeping actors in scene for debugging because the script exited on error")

        if should_cleanup:
            for label, created, actor in self._runtime_actors():
                if not created:
                    continue
                response = self._try_call(actor.remove_raw)
                if response is not None:
                    print(f"[INFO] remove {label}:", response)

        self.runtime["client"].close()

    def _make_drone_agent(self, client, config_key):
        drone_config = self.config[config_key]
        return AgentDrone(
            client,
            drone_config["id"],
            classname=self.config["drone_class"],
            label=drone_config["label"],
            mission_role=drone_config["role"],
        )

    def _make_guidance_agent(self, client):
        guidance_config = self.config["guidance"]
        return AgentGuidance(
            client,
            actor_id=guidance_config["id"],
            classname=guidance_config["classname"],
            label=guidance_config["label"],
        )

    def _create_runtime(self):
        connection = self.config["connection"]
        client = TCPClient(
            host=connection["host"],
            port=connection["port"],
            timeout=connection["timeout"],
            auto_connect=False,
        )
        if not client.connect() or not self._ok(client.ping()):
            print("[ERROR] cannot connect to simulator")
            client.close()
            return None

        self.runtime = {
            "client": client,
            "interceptor": self._make_drone_agent(client, "interceptor"),
            "target": self._make_drone_agent(client, "target"),
            "guidance": self._make_guidance_agent(client),
        }
        return self.runtime
