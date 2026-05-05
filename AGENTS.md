# AGENTS

This file helps coding agents work productively in this repository.

## Project Scope
- Repository root: [sets](.)
- Primary documentation: [README.md](README.md)
- Main code areas:
  - C++ core and Python bindings: [src](src)
  - Experiment scripts: [scripts](scripts)
  - YAML configs: [configs](configs)
  - Generated outputs: [data](data), [plots](plots)

## Start Here
- Read [README.md](README.md) first for project context and baseline commands.
- For rollout/policy flows, inspect [scripts/rollout.py](scripts/rollout.py) and [scripts/policy_convergence.py](scripts/policy_convergence.py).
- For config resolution behavior, inspect [src/util/util.py](src/util/util.py#L150).
- For Python-C++ interface surface, inspect [src/bindings.cpp](src/bindings.cpp).

## Environment And Build
- Create environment:
```bash
conda env create --file environment.yml
conda activate sets
```
- Build Python bindings (required before most scripts):
```bash
cd src
mkdir -p build
cd build
cmake -DPYTHON_EXECUTABLE=$(which python) -DCMAKE_BUILD_TYPE=Release ..
make -j
```
- Rebuild whenever files in [src](src) C++ or [src/bindings.cpp](src/bindings.cpp) change.

## Run Commands
- Run from [scripts](scripts) so relative paths to ../data and ../plots stay valid.
```bash
cd scripts
python value_convergence.py
python policy_convergence.py
```

## Conventions And Pitfalls
- Scripts import bindings via `from build.bindings import ...` and expect src path setup; avoid refactoring import style unless you update all affected scripts.
- Config lookup uses basename matching in [src/util/util.py](src/util/util.py#L150). Config names must be unique across [configs](configs), or resolution fails.
- Keep generated artifacts in [data](data) and [plots](plots) as outputs; do not hand-edit files under build directories.
- There is no established unit-test suite in this repo; use focused smoke runs of target scripts after changes.

## Quick Validation After Changes
- Python-only script changes:
```bash
cd scripts
python -m py_compile rollout.py value_convergence.py policy_convergence.py
```
- Binding/API changes:
```bash
cd src/build
make -j
cd ../../scripts
python -c "import os,sys; sys.path.insert(0, os.path.join(os.getcwd(), '..', 'src')); from build.bindings import get_mdp; print('bindings ok')"
```
