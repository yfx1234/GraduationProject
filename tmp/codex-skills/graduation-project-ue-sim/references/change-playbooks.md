# Change Playbooks

## Remote Call Path

Use this sequence when adding or debugging a Python-to-Unreal capability:

1. Find the target Unreal actor and the exact `UFUNCTION` name.
2. Check whether Python should call it through a typed wrapper or directly through `AgentBase.call_function()`.
3. Verify the payload keys match the Unreal parameter names exactly.
4. Confirm the return value is serialized into something the Python side can parse.
5. Verify the calling script handles both transport failure and semantic failure.

Read these files in order:

- `Source/GraduationProject/Core/Network/CommandRouter.h`
- `Source/GraduationProject/Core/Network/FCommandHandle.h`
- target actor header and source
- `PythonClient/Run/GraduationSIM/AgentBase.py`
- matching Python wrapper
- runtime caller script

## Python Config And Runtime Path

Use this sequence when changing shared Python behavior:

1. Check `PythonClient/Run/params.py` first.
2. Check whether the change belongs in script-local logic or shared wrapper logic.
3. If it affects path discovery or local Ultralytics setup, inspect `GraduationSIM/__init__.py`.
4. If it affects transport or retry behavior, inspect `GraduationSIM/TCPClient.py`.
5. If it affects shared Python types or generic actor calls, inspect `GraduationSIM/AgentBase.py`.

## Visual Intercept Path

Use this sequence when tuning intercept logic:

1. Read `Source/GraduationProject/Guidance/GuidanceActor.h` to confirm the public API surface.
2. Read `Source/GraduationProject/Guidance/VisualInterceptController.h` and `.cpp` for state and control internals.
3. Read `PythonClient/Run/GraduationSIM/AgentGuidance.py` for the wrapper contract.
4. Read `PythonClient/Run/params.py` and `PythonClient/Run/run.py` for runtime parameter choices and detector outputs.

Check these failure classes before changing gains:

- wrong parameter spelling between Python and Unreal
- `Dt` or latency mismatch
- image width or height mismatch
- area versus area ratio confusion
- frame convention mismatch

## Dataset And Training Path

Use this sequence when changing the vision data pipeline:

1. Read `PythonClient/Run/Collect.py` for projection-based label generation, deduplication, and output layout.
2. Read `PythonClient/Run/Train.py` for training-time orchestration and run naming.
3. Read `PythonClient/Run/GraduationSIM/__init__.py` for path and detector utilities.
4. Confirm the runtime model lookup still lands on the intended `best.pt`.

Important repo-specific facts:

- `Collect.py` resets and rewrites dataset folders.
- `Train.py` imports dataset helpers from `Collect.py`.
- `run.py` expects trained weights under the detect runs root resolved by `resolve_paths`.
- local Ultralytics import wiring is configured by `ensure_ultralytics_import_path()`.

## Missing Paths Or Legacy Modules

Use this order when docs and code disagree:

1. Check the filesystem.
2. Check `git status --short`.
3. Determine whether the path is absent, untracked, or intentionally deleted in this branch.
4. Read older docs only after the current branch state is clear.

## Vendored Ultralytics Changes

Use this order before editing vendor code:

1. Check whether the issue can be solved in `params.py`, `Train.py`, `run.py`, or `GraduationSIM/__init__.py`.
2. If vendor edits are still required, keep the patch minimal and document the reason.
3. Re-check any project code that assumes the stock Ultralytics behavior.

## Debugging Order

Use this order for mixed Unreal and Python failures:

1. Confirm Unreal is running and the TCP server is reachable on `127.0.0.1:9000`.
2. Confirm actors were created successfully and IDs match what the Python side expects.
3. Confirm the Python wrapper is sending the expected payload shape.
4. Confirm Unreal accepted the call and returned the expected JSON shape.
5. Confirm algorithm state only after transport and payload shape are proven correct.

## Safe Assumptions

- Default to `ue` frame unless the caller explicitly needs `ned`.
- Default drone blueprint class is exported as `DEFAULT_DRONE_CLASS`.
- Treat `run.py`, `Collect.py`, and `Train.py` as the active scripts unless the live tree shows a newer replacement.
