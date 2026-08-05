# MuJoCo Menagerie developer commands.
#
# Quick start for new contributors:
#   make install   # one-time: install pre-commit + git hook
#   make all       # run every check CI runs

.PHONY: help install check test gallery all

help:
	@echo "MuJoCo Menagerie developer commands:"
	@echo "  make install   One-time setup: install pre-commit and the git hook"
	@echo "  make check     Run lint, format, license, and XML checks (fast)"
	@echo "  make test      Run the pytest model + structural test suite (slow)"
	@echo "  make gallery   Re-render thumbnails and update the gallery in README.md"
	@echo "  make all       Run check + test (everything CI runs)"

install:
	uv tool install pre-commit
	pre-commit install

check:
	pre-commit run --all-files

test:
	pre-commit run --hook-stage manual pytest --all-files

gallery:
	uv run --no-project generate_gallery.py

all: check test
