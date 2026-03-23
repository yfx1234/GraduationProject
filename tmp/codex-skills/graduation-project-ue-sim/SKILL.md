---
name: graduation-project-ue-sim
description: Project-specific guidance for the GraduationProject Unreal Engine 5.7 plus Python simulation workspace. Use when working in D:\Xstarlab\UEProjects\GraduationProject\GraduationProject on Unreal C++ modules, PythonClient/Run scripts, params configuration, TCP or JSON actor calls, visual intercept logic, collection and training pipelines, vendored Ultralytics integration, branch migration from legacy Turret or UI code, architecture recovery, or repo-specific debugging.
---

# Graduation Project Ue Sim

## Overview

Use this skill to orient quickly inside the GraduationProject workspace and to make changes without following stale project docs. Treat the live code tree and the current git migration state as the source of truth, then use repository documents for formulas, rationale, and thesis-facing explanations.

## Start Here

1. Confirm the current workspace is this repo by checking for `GraduationProject.uproject`.
2. If the task mentions missing modules, deleted scripts, or old docs, inspect `git status --short` before assuming the path should exist.
3. Read [references/current-layout.md](references/current-layout.md) for live structure, transition state, and low-signal directories.
4. Read [references/code-map.md](references/code-map.md) for file ownership and ripple effects.
5. Read [references/change-playbooks.md](references/change-playbooks.md) for repo-specific edit workflows.
6. Read `README.md` and `Document/项目实现流程与方法详解.md` only after the live tree is clear.

## Operating Rules

- Treat the live filesystem and the current git status together as authoritative.
- Do not restore `Turret`, `UI`, `GuidanceMethods`, or `intercept_fixed_cam.py` just because older docs mention them. Verify whether the user wants those legacy paths back.
- Deprioritize generated and output-heavy paths such as `Binaries`, `Intermediate`, `DerivedDataCache`, `Saved`, `_video_frames`, `output`, and most of `tmp` unless the task explicitly concerns them.
- Treat `PythonClient/YOLO/ultralytics` as vendored third-party code. Prefer changing project wrappers, params, and training entry scripts before editing vendor internals.
- Keep Python named arguments aligned with Unreal `UFUNCTION` parameter names. The wrapper layer sends named parameters, so spelling changes break calls quietly.
- Update `PythonClient/Run/params.py` whenever runtime defaults or experiment parameters are intended to become shared configuration.
- Use project documents for formulas and rationale, not to override the live code tree.

## Primary Entry Points

- Read `Source/GraduationProject/Core/Network/CommandRouter.h` and `Source/GraduationProject/Core/Network/FCommandHandle.h` for transport and generic actor invocation.
- Read `Source/GraduationProject/Guidance/GuidanceActor.h`, `VisualInterceptController.h`, and `KalmanPredictor.h` for the current guidance stack.
- Read `Source/GraduationProject/GraduationProject.Build.cs` when changing Unreal module dependencies.
- Read `PythonClient/Run/params.py` before changing run, collect, or train defaults.
- Read `PythonClient/Run/GraduationSIM/AgentBase.py`, `AgentGuidance.py`, `AgentDrone.py`, `TCPClient.py`, and `__init__.py` before editing Python wrappers or helpers.
- Read `PythonClient/Run/run.py`, `Collect.py`, and `Train.py` for the active runtime, collection, and training pipelines.
- Read `Docs/thesis_task_and_review.md` only for thesis-facing packaging or task-book context, not runtime behavior.

## Task Playbooks

### Add or modify a remote Unreal capability

1. Update the target actor interface and keep it `BlueprintCallable` if Python must call it.
2. Verify the generic actor path in `FCommandHandle` can serialize the parameters and return value cleanly.
3. Update the matching Python wrapper in `PythonClient/Run/GraduationSIM`.
4. Update the caller script or config in `PythonClient/Run`.
5. Report the final call chain explicitly so the next editor can trace it.

### Change Python runtime behavior or shared defaults

1. Check whether the change belongs in `params.py` rather than hard-coded script constants.
2. Verify which scripts read that config today: `run.py`, `Collect.py`, and `Train.py`.
3. If the change affects exports or helpers, inspect `GraduationSIM/__init__.py` before touching import sites.
4. Keep `AgentBase.py` changes minimal because it carries shared types and core wrapper behavior.

### Tune visual intercept behavior

1. Start with `GuidanceActor` as the API surface.
2. Move immediately to `UVisualInterceptController` for the state machine, PID controllers, and Kalman-backed runtime state.
3. Check `run.py`, `params.py`, and `AgentGuidance.py` for parameter names, `Dt`, image size, area semantics, and latency handling before changing gains.
4. Keep detector output conventions and Unreal control conventions separate in your reasoning.

### Change collection or training logic

1. Treat `Collect.py` as the owner of projection-based auto-labeling, duplicate filtering, and dataset writing.
2. Treat `Train.py` as the owner of dataset preparation, weight selection, and Ultralytics training orchestration.
3. Remember that `Train.py` imports helper functions from `Collect.py`, so shared dataset logic lives there.
4. Keep path logic consistent through `resolve_paths` instead of hard-coding new directories.
5. Verify that any new output still feeds the runtime model lookup used by `run.py`.

### Touch vendored Ultralytics code

1. First prove the problem cannot be solved in `params.py`, `Train.py`, `run.py`, or `GraduationSIM/__init__.py`.
2. Limit the scope to the smallest necessary vendor files.
3. Document the local fork behavior because vendored code is harder to diff mentally later.

### Investigate missing files or module regressions

1. Check `git status --short` before assuming the repository is incomplete.
2. Distinguish between paths removed from the filesystem and paths intentionally tracked as deletions in this branch.
3. Use repository documents only to understand historical intent, not to force restoration.

## Use Project Documents Deliberately

- Read `Document/项目实现流程与方法详解.md` when the task needs formulas, control-chain rationale, or deeper reading order.
- Read `Document/数学算法流程与公式推导.md` when the task is math-heavy rather than plumbing-heavy.
- Expect some older markdown files to describe paths that are currently being removed or consolidated.
