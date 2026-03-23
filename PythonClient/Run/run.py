from __future__ import annotations

import math
import time

try:
    import cv2
except ImportError:
    cv2 = None

from guidance_params import build_visual_intercept_args

from params import net, sim, yolo as yoloConfig
from GraduationSIM import (
    AgentBase,
    AgentDrone,
    AgentGuidance,
    DEFAULT_DRONE_CLASS,
    Pose,
    TCPClient,
    YoloDetector,
    pick_runtime_model_path,
    resolve_paths,
)

def make_detector(modelPath):
    return YoloDetector(model_path=str(modelPath), conf=0.35, iou=0.45, imgsz=640, class_id=0, max_det=5)


def start_guidance(guidance, interceptorId, targetId, visualArgs):
    payload = dict(visualArgs) if isinstance(visualArgs, dict) else {}
    payload["InterceptorId"] = interceptorId
    payload["TargetId"] = targetId
    return guidance.visual_intercept_start(**payload)



def to_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def to_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on", "ok"}


def parse_guidance_part(value):
    data = value if isinstance(value, dict) else {}
    return {
        "valid": to_bool(data.get("valid")),
        "usedPrediction": to_bool(data.get("used_prediction")),
        "leadTime": to_float(data.get("lead_time")),
    }


def parse_guidance_state(value):
    data = value if isinstance(value, dict) else {}
    kalmanData = data.get("kalman") if isinstance(data.get("kalman"), dict) else {}
    return {
        "state": str(data.get("state", "UNKNOWN")),
        "lostCount": int(data.get("lost_count", 0) or 0),
        "latency": to_float(data.get("processing_latency")),
        "track": parse_guidance_part(data.get("track")),
        "prediction": parse_guidance_part(data.get("prediction")),
        "control": data.get("control") if isinstance(data.get("control"), dict) else {},
        "kalman": {
            "q": to_float(kalmanData.get("q")),
            "uncertainty": to_float(kalmanData.get("uncertainty")),
        },
    }


class RunAbort(RuntimeError):
    pass


def require_ok(response, message):
    if not AgentBase.is_ok(response):
        raise RunAbort(message)
    return response


class BoxPredictor:
    def __init__(self, alpha=0.74, beta=0.18):
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.box = None
        self.velocity = [0.0, 0.0, 0.0, 0.0]

    def update(self, box, deltaTime):
        if not box or len(box) != 4:
            return
        box = [float(item) for item in box]
        deltaTime = max(1e-3, float(deltaTime))
        if self.box is None:
            self.box = box
            self.velocity = [0.0, 0.0, 0.0, 0.0]
            return
        predictedBox = [self.box[index] + self.velocity[index] * deltaTime for index in range(4)]
        error = [box[index] - predictedBox[index] for index in range(4)]
        self.box = [predictedBox[index] + self.alpha * error[index] for index in range(4)]
        gain = self.beta / deltaTime
        self.velocity = [self.velocity[index] + gain * error[index] for index in range(4)]

    def predict(self, leadTime, imageWidth, imageHeight):
        if self.box is None:
            return None
        box = [self.box[index] + self.velocity[index] * max(0.0, float(leadTime)) for index in range(4)]
        x1, y1, x2, y2 = box
        x1 = max(0.0, min(float(imageWidth - 2), x1))
        y1 = max(0.0, min(float(imageHeight - 2), y1))
        x2 = max(x1 + 1.0, min(float(imageWidth - 1), x2))
        y2 = max(y1 + 1.0, min(float(imageHeight - 1), y2))
        return [x1, y1, x2, y2]


def clamp(value, low, high):
    return max(low, min(high, value))


def speed_from_values(values):
    if not isinstance(values, (list, tuple)):
        return 0.0
    x = to_float(values[0]) if len(values) > 0 else 0.0
    y = to_float(values[1]) if len(values) > 1 else 0.0
    z = to_float(values[2]) if len(values) > 2 else 0.0
    return math.sqrt(x * x + y * y + z * z)


def draw_box(frame, box, color, text, thickness=2, showCorners=False, textPosition="top"):
    if not box or len(box) != 4:
        return
    imageHeight, imageWidth = frame.shape[:2]
    x1 = max(0, min(imageWidth - 2, int(round(box[0]))))
    y1 = max(0, min(imageHeight - 2, int(round(box[1]))))
    x2 = max(x1 + 1, min(imageWidth - 1, int(round(box[2]))))
    y2 = max(y1 + 1, min(imageHeight - 1, int(round(box[3]))))
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
    if showCorners:
        for pointX, pointY in ((x1, y1), (x2, y1), (x2, y2), (x1, y2)):
            cv2.circle(frame, (pointX, pointY), 3, color, -1, cv2.LINE_AA)
    textY = min(imageHeight - 8, y2 + 18) if textPosition == "bottom" else max(20, y1 - 8)
    cv2.putText(frame, text, (max(8, x1), textY), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)


class LinePlanner:
    def __init__(self, targetPose, interceptorPose, radius=600.0):
        heading = float(getattr(targetPose, "yaw", 0.0))
        if abs(heading) <= 1e-6:
            heading = math.degrees(math.atan2(targetPose.y - interceptorPose.y, targetPose.x - interceptorPose.x))
        self.heading = math.radians(heading)
        self.radius = max(500.0, float(radius))

    def command(self, elapsedTime, baseSpeed, targetDistance):
        turnRate = 0.0 if abs(baseSpeed) <= 1e-6 else float(baseSpeed) / self.radius
        heading = self.heading + turnRate * float(elapsedTime)
        velocityX = math.cos(heading) * float(baseSpeed)
        velocityY = math.sin(heading) * float(baseSpeed)
        return velocityX, velocityY, 0.0, math.degrees(heading)


class EvadePlanner:
    def __init__(self, targetPose, interceptorPose):
        heading = float(getattr(targetPose, "yaw", 0.0))
        if abs(heading) <= 1e-6:
            heading = math.degrees(math.atan2(targetPose.y - interceptorPose.y, targetPose.x - interceptorPose.x))
        self.baseHeading = math.radians(heading)

    def command(self, elapsedTime, baseSpeed, targetDistance):
        baseSpeed = max(0.0, float(baseSpeed))
        panic = 0.0 if targetDistance <= 0.0 else clamp((20.0 - float(targetDistance)) / 12.0, 0.0, 1.0)
        heading = self.baseHeading
        heading += math.radians(18.0 + 10.0 * panic) * math.sin(0.55 * elapsedTime + 0.3)
        heading += math.radians(8.0 + 6.0 * panic) * math.sin(1.20 * elapsedTime + 0.9)
        forwardX, forwardY = math.cos(heading), math.sin(heading)
        lateralX, lateralY = -forwardY, forwardX
        forwardWeight = 1.0 + 0.18 * math.sin(0.45 * elapsedTime + 0.4)
        lateralWeight = (0.30 + 0.38 * panic) * math.sin(0.55 * elapsedTime + 0.3)
        lateralWeight += (0.08 + 0.10 * panic) * math.sin(1.20 * elapsedTime + 0.9)
        verticalWeight = 0.10 * math.sin(0.45 * elapsedTime + 0.2) + (0.06 + 0.05 * panic) * math.sin(1.00 * elapsedTime + 1.1)
        velocityX = forwardX * forwardWeight + lateralX * lateralWeight
        velocityY = forwardY * forwardWeight + lateralY * lateralWeight
        velocityZ = verticalWeight
        speed = math.sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ)
        scale = 0.0 if speed <= 1e-6 else baseSpeed / speed
        velocityX *= scale
        velocityY *= scale
        velocityZ *= scale
        yaw = math.degrees(math.atan2(velocityY, velocityX))
        return velocityX, velocityY, velocityZ, yaw


def make_planner(targetPath, targetPose, interceptorPose):
    pathName = str(targetPath or "evade").strip().lower()
    if pathName == "line":
        return LinePlanner(targetPose, interceptorPose), "line"
    return EvadePlanner(targetPose, interceptorPose), "evade"


def render_frame(frame, detection, guidanceResponse, predictor, motion):
    if detection and detection.bbox_xyxy:
        draw_box(frame, detection.bbox_xyxy, (60, 230, 90), f"YOLO {detection.conf:.2f}")
    guidanceState = parse_guidance_state(guidanceResponse)
    trackState = guidanceState["track"]
    predictionState = guidanceState["prediction"]
    controlState = guidanceState["control"]
    kalmanState = guidanceState["kalman"]
    leadTime = predictionState["leadTime"]
    if not predictionState["valid"] and trackState["usedPrediction"]:
        leadTime = trackState["leadTime"]
    if (predictionState["valid"] or trackState["usedPrediction"]) and predictor.box is not None:
        predictedBox = predictor.predict(leadTime, frame.shape[1], frame.shape[0])
        if predictedBox is not None:
            draw_box(frame, predictedBox, (0, 0, 255), f"KALMAN {leadTime:.2f}s", showCorners=True, textPosition="bottom")
    trackMode = "prediction" if trackState["usedPrediction"] else "measurement"
    text = (
        f"state={guidanceState['state']} lost={guidanceState['lostCount']} track={trackMode} pred={int(predictionState['valid'])} "
        f"ex={to_float(controlState.get('ex')):+.3f} ey={to_float(controlState.get('ey')):+.3f} "
        f"yaw={to_float(controlState.get('yaw_cmd_deg')):+.1f} lat={guidanceState['latency'] * 1000.0:.0f}ms q={kalmanState['q']:.2f} u={kalmanState['uncertainty']:.2f}"
    )
    cv2.putText(frame, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (235, 235, 235), 1, cv2.LINE_AA)
    speedText = (
        f"target {motion['targetActual']:.2f}/{motion['targetCommand']:.2f} mps  "
        f"interceptor {motion['interceptorActual']:.2f}/{motion['interceptorCommand']:.2f}/{motion['interceptorLimit']:.2f} mps"
    )
    cv2.putText(frame, speedText, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, speedText, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (235, 235, 235), 1, cv2.LINE_AA)
    cv2.imshow("run", frame)
    return (cv2.waitKey(1) & 0xFF) == ord("q")


def run():
    if cv2 is None:
        raise RuntimeError("opencv-python is required for run.py")

    runConfig = sim["run"]
    paths = resolve_paths(__file__)
    modelPath = pick_runtime_model_path(paths, configured_model=yoloConfig.get("model"))
    host = str(net.get("host", "127.0.0.1"))
    port = int(net.get("port", 9000))
    timeout = 6.0
    imageQuality = 82
    maxRunTime = 500.0
    cameraFov = 124.0
    interceptorId = str(runConfig["interceptorId"])
    targetId = str(runConfig["targetId"])
    guidanceId = str(runConfig["guidanceId"])
    interceptorPose = Pose.from_any(runConfig["interceptorPose"])
    targetPose = Pose.from_any(runConfig["targetPose"])
    interceptorAltitude = to_float(runConfig.get("interceptorAltitude", 8.0), 8.0)
    interceptorSpeed = to_float(runConfig.get("interceptorSpeed", 14.0), 14.0)
    targetAltitude = to_float(runConfig.get("targetAltitude", 8.2), 8.2)
    targetSpeed = to_float(runConfig.get("targetSpeed", 3.1), 3.1)
    guidanceArgs = build_visual_intercept_args(runConfig, interceptorSpeed)
    planner, targetPath = make_planner(runConfig.get("targetPath", "evade"), targetPose, interceptorPose)

    client = TCPClient(host=host, port=port, timeout=timeout, auto_connect=False)
    interceptor = target = guidance = None
    predictor = BoxPredictor()
    createdInterceptor = createdTarget = createdGuidance = started = False
    result = {"status": "ok", "model": str(modelPath), "targetPath": targetPath, "exit_reason": "completed"}
    motion = {
        "targetActual": 0.0,
        "targetCommand": 0.0,
        "interceptorActual": 0.0,
        "interceptorCommand": 0.0,
        "interceptorLimit": to_float(guidanceArgs.get("MaxForwardSpeed"), interceptorSpeed),
    }

    try:
        if not client.connect():
            return {"status": "error", "message": "connect failed", "model": str(modelPath)}

        interceptor = AgentDrone(client, actor_id=interceptorId, classname=DEFAULT_DRONE_CLASS, label="Interceptor", mission_role="interceptor")
        target = AgentDrone(client, actor_id=targetId, classname=DEFAULT_DRONE_CLASS, label="Target", mission_role="target")
        guidance = AgentGuidance(client, actor_id=guidanceId, classname="GuidanceActor", label="Guidance")
        AgentBase.remove_existing_actors(client, (guidanceId, interceptorId, targetId))

        require_ok(guidance.create(pose=Pose(), classname="GuidanceActor", label="Guidance"), "create guidance failed")
        createdGuidance = True
        require_ok(interceptor.create(pose=interceptorPose, classname=DEFAULT_DRONE_CLASS, label="Interceptor"), "create interceptor failed")
        createdInterceptor = True
        require_ok(target.create(pose=targetPose, classname=DEFAULT_DRONE_CLASS, label="Target"), "create target failed")
        createdTarget = True

        if not AgentBase.wait_response(lambda: guidance.get_state()):
            raise RunAbort("guidance state unavailable")
        if not AgentBase.wait_response(lambda: interceptor.get_state(frame="ue")):
            raise RunAbort("interceptor state unavailable")
        if not AgentBase.wait_response(lambda: target.get_state(frame="ue")):
            raise RunAbort("target state unavailable")

        for drone, altitude in ((interceptor, interceptorAltitude), (target, targetAltitude)):
            require_ok(drone.enable_api_control(True), f"enable api control failed for {drone.actor_id}")
            require_ok(drone.takeoff(altitude=altitude), f"takeoff command failed for {drone.actor_id}")

        detector = make_detector(modelPath)

        for drone, altitude in ((interceptor, interceptorAltitude), (target, targetAltitude)):
            if not AgentBase.wait_altitude(drone, altitude):
                raise RunAbort(f"takeoff timeout for {drone.actor_id}")
        require_ok(interceptor.hover(), "interceptor hover failed")
        require_ok(target.hover(), "target hover failed")
        time.sleep(0.18)
        require_ok(interceptor.set_camera_fov(cameraFov), "set fov failed")
        require_ok(interceptor.set_camera_angles(0.0, 0.0), "set camera angles failed")
        require_ok(guidance.reset(), "guidance reset failed")

        require_ok(start_guidance(guidance, interceptor.actor_id, target.actor_id, guidanceArgs), "guidance start failed")
        started = True

        cv2.namedWindow("run", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("run", 1080, 620)
        targetDistance = 24.0
        loopDelay = 0.0
        startTime = time.monotonic()
        lastTime = startTime
        nextStateSampleTime = startTime

        while True:
            now = time.monotonic()
            elapsedTime = now - startTime
            if elapsedTime > maxRunTime:
                result["exit_reason"] = "max_time"
                break

            velocityX, velocityY, velocityZ, yaw = planner.command(elapsedTime, targetSpeed, targetDistance)
            require_ok(
                target.move_by_velocity(
                    vx=velocityX,
                    vy=velocityY,
                    vz=velocityZ,
                    frame="ue",
                    yaw_mode="angle",
                    yaw=yaw,
                    move_pattern="forward_only",
                ),
                "target move failed",
            )
            motion["targetCommand"] = math.sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ)

            packet = interceptor.get_image_packet(image_type="scene", quality=imageQuality)
            if packet is None:
                raise RunAbort("image capture failed")
            frame = packet.decode_bgr()
            if frame is None:
                raise RunAbort("image decode failed")

            detection = detector.detect_bgr(frame)
            deltaTime = max(1e-3, now - lastTime)
            lastTime = now
            if detection and detection.bbox_xyxy:
                predictor.update(detection.bbox_xyxy, deltaTime)

            guidanceResponse = require_ok(
                guidance.visual_intercept_update(
                    has_detection=detection.has_detection,
                    cx=detection.cx,
                    cy=detection.cy,
                    area=detection.area,
                    area_ratio=detection.area_ratio,
                    conf=detection.conf,
                    dt=deltaTime,
                    processing_latency=max(0.0, time.monotonic() - now),
                    image_w=detection.image_w or packet.width or 640,
                    image_h=detection.image_h or packet.height or 480,
                    interceptor_id=interceptor.actor_id,
                    target_id=target.actor_id,
                ),
                "guidance update failed",
            )
            motion["interceptorCommand"] = speed_from_values(guidanceResponse.get("cmd_velocity"))
            if now >= nextStateSampleTime:
                interceptorState = interceptor.get_state(frame="ue")
                if AgentBase.is_ok(interceptorState):
                    motion["interceptorActual"] = speed_from_values(interceptorState.get("velocity"))
                targetState = target.get_state(frame="ue")
                if AgentBase.is_ok(targetState):
                    motion["targetActual"] = speed_from_values(targetState.get("velocity"))
                nextStateSampleTime = now + 0.12
            if render_frame(frame, detection, guidanceResponse, predictor, motion):
                result["exit_reason"] = "user_quit"
                break
            if guidanceResponse.get("target_distance") is not None:
                targetDistance = max(0.0, to_float(guidanceResponse.get("target_distance")))
            if guidanceResponse.get("captured"):
                result["exit_reason"] = "captured"
                result["captured"] = True
                break

            sleepTime = loopDelay - (time.monotonic() - now)
            if loopDelay > 0.0 and sleepTime > 0.0:
                time.sleep(sleepTime)
        return result
    except RunAbort as error:
        return {"status": "error", "message": str(error), "model": str(modelPath)}
    finally:
        if guidance is not None and createdGuidance and started:
            try:
                guidance.visual_intercept_stop(interceptor_id=interceptorId, target_id=targetId)
            except Exception:
                pass
        for drone, created in ((interceptor, createdInterceptor), (target, createdTarget)):
            if drone is not None and created:
                AgentBase.safe_hover(drone)
        for actor, created in ((target, createdTarget), (interceptor, createdInterceptor), (guidance, createdGuidance)):
            if actor is not None and created:
                AgentBase.safe_remove(actor)
        AgentBase.disconnect_client(client)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    run()














