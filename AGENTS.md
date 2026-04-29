# AGENTS.md

This repository is a local/offline C++20 simulator for hybrid rocket 6-DoF flight analysis.

## Development Rules

- Keep the physics core independent from GUI and file formats.
- Keep input/output code thin. CSV, KML, and future formats should call the core rather than own simulation logic.
- Use SI units internally.
- Use NED coordinates internally. Output altitude as a positive value.
- Keep Windows local execution as the primary target. Core and CLI should remain buildable in CI.
- Prefer standard C++ and CMake. Avoid adding dependencies unless the tradeoff is explicit.
- Run tests after implementation work:
  - `.\scripts\build_mingw.ps1` on the current Windows/MinGW workspace.
  - `ctest` when a normal CMake generator is available.

## Repository Layout

- `include/hrocket/`: public headers
- `src/`: implementation
- `tests/`: unit and regression tests
- `samples/`: sample vehicle, thrust, and wind CSV files
- `scripts/`: local developer scripts
- `.github/workflows/`: CI and release automation

## Related Project Guidance

- `skills.md` describes the project-specific development skills and implementation workflow.

## Simulation Architecture

- `simulator.*` owns integration and force/moment evaluation.
- `barrowman.*` owns Barrowman center-of-pressure calculation.
- `io.*` owns CSV/KML/summary/dispersion input and output.
- `IController` and `IActuator` are extension points for future control and actuator models.
- Descent behavior is selected by `SimulationConfig::descent_mode`.
- Wind behavior is selected by `SimulationConfig::wind_mode`.

## Git Workflow

- After each coherent feature, fix, or development milestone, stage the relevant files, commit, and push.
- Use commit messages that match the project convention:
  - `Add:` for adding files or code.
  - `Change:` for changing files or behavior.
  - `Fix:` for fixing defects.
  - `Delete:` for deleting files or code.
- Do not mix unrelated changes in one commit.
- Before committing, run the most relevant local verification command and include the result in the final report.
- If no remote is configured, commit locally and report that push was skipped because no remote exists.

## Generated Files

- Do not commit build outputs, executables, or simulation outputs.
- Keep generated local outputs under ignored paths such as `build-local/`, `out/`, or `out-*/`.
