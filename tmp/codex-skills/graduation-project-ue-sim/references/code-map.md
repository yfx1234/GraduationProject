# Code Map

## Unreal Request Path

Use this chain when tracing a Python request into Unreal:

1. `PythonClient/Run/*` caller script
2. `PythonClient/Run/GraduationSIM/*` wrapper or transport helper
3. TCP newline-delimited JSON in `TCPClient.py`
4. `USimGameInstance` server entry
5. `UCommandRouter`
6. `FCommandHandle`
7. target actor `UFUNCTION`
8. runtime controller or movement code

## Guidance Stack

The active guidance stack is currently:

- `GuidanceActor.*` as the public API facade
- `VisualInterceptController.*` as the visual state machine and control core
- `KalmanPredictor.*` as the predictor implementation
- `ITargetPredictor.h` as the remaining predictor interface boundary

If a task mentions old guidance methods, check whether it refers to deleted legacy files before adding anything back.

## Python Runtime Stack

The active Python stack is currently:

- `params.py` for shared configuration
- `run.py` for live visual intercept execution
- `Collect.py` for collection, projection labeling, and dataset utilities
- `Train.py` for training orchestration
- `GraduationSIM/AgentBase.py` for `Pose`, `ImagePacket`, generic actor operations, and helper behavior
- `GraduationSIM/AgentDrone.py` and `AgentGuidance.py` for typed wrappers
- `GraduationSIM/TCPClient.py` for transport
- `GraduationSIM/__init__.py` for exports, `resolve_paths`, and `ensure_ultralytics_import_path`

## High-Ripple Files

Treat edits to these files as high-impact:

- `Source/GraduationProject/Core/Network/FCommandHandle.*`
  Changes affect every generic actor call.
- `Source/GraduationProject/Core/Network/CommandRouter.*`
  Changes affect top-level command routing.
- `Source/GraduationProject/Guidance/VisualInterceptController.*`
  Changes affect the visual intercept behavior directly.
- `PythonClient/Run/params.py`
  Changes affect shared runtime defaults across scripts.
- `PythonClient/Run/GraduationSIM/AgentBase.py`
  Changes affect shared Python types and generic wrapper behavior.
- `PythonClient/Run/GraduationSIM/__init__.py`
  Changes affect export surfaces and local Ultralytics integration.
- `PythonClient/Run/Collect.py`
  Changes can break both collection and training, because training imports helpers from this file.

## Low-Signal Or Secondary Paths

Only read these when the task needs them:

- vendored Ultralytics internals
- generated Unreal outputs
- rendered video frame folders
- thesis document outputs under `output/doc`
- temporary doc generation scripts under `tmp/docs`
