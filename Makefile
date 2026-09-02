# MuJoCo Menagerie developer commands.
#
# Quick start for new contributors:
#   make install   # one-time: install pre-commit + git hook
#   make all       # run every check CI runs

.PHONY: help install check test gallery registry archives python-test build all

help:
	@echo "MuJoCo Menagerie developer commands:"
	@echo "  make install      One-time setup: install pre-commit and the git hook"
	@echo "  make check        Run lint, format, license, and XML checks (fast)"
	@echo "  make test         Run the pytest model + structural test suite (slow)"
	@echo "  make gallery      Re-render thumbnails and update the gallery in README.md"
	@echo "  make registry     Derive python/src/mujoco_menagerie/registry.json (no archives)"
	@echo "  make archives     Build the model archives + registry for a release (slow)"
	@echo "  make python-test  Test the mujoco-menagerie package (builds the registry first)"
	@echo "  make build        Build the wheel + sdist into dist/ and smoke test both"
	@echo "  make all          Run check + test + python-test (everything CI runs)"

install:
	uv tool install pre-commit
	pre-commit install

check:
	pre-commit run --all-files

test:
	pre-commit run --hook-stage manual pytest --all-files

gallery:
	uv run --no-project generate_gallery.py

registry:
	uv run --no-project build_registry.py --no-archives --allow-dirty

archives:
	uv run --no-project build_registry.py

python-test: registry
	cd python && uv run --group dev pytest -n auto

build: archives
	rm -rf dist && uv build python --out-dir dist
	uv run --isolated --no-project --with dist/*.whl python/tests/smoke_test.py
	uv run --isolated --no-project --with dist/*.tar.gz python/tests/smoke_test.py

all: check test python-test
