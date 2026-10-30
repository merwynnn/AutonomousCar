.PHONY: install dev backend web cpp cpp-test test lint format check clean

PY := uv run
PORT ?= 8000
WEB_PORT ?= 5173

install:
	uv sync --all-extras
	cd web && npm install

dev:
	@trap 'kill 0' INT; \
	( $(PY) uvicorn awake_sim.server.app:app --reload --port $(PORT) ) & \
	( cd web && npm run dev -- --port $(WEB_PORT) ) & \
	wait

backend:
	$(PY) uvicorn awake_sim.server.app:app --reload --port $(PORT)

web:
	cd web && npm run dev -- --port $(WEB_PORT)

cpp:
	$(PY) python -m awake_sim.cli build-cpp

cpp-test:
	$(PY) python -m awake_sim.cli test-cpp

test:
	$(PY) pytest

lint:
	$(PY) ruff check src tests
	$(PY) mypy src

format:
	$(PY) ruff format src tests
	$(PY) ruff check --fix src tests

check: lint test
	cd web && npm run build

clean:
	rm -rf build .venv .pytest_cache .mypy_cache .ruff_cache
	rm -rf web/node_modules web/dist
	find src -name '_awake_slam*.so' -delete
