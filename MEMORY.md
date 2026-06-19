# Repository Memory & Session Log

## Core Purpose
This repository contains the `Dynamic_World_Generator_ros2_control` project, which facilitates dynamic world generation for ROS2 Control.

## Current Setup & Tasks Completed
- Python Virtual Environment (`.venv`) set up at `PROJECT_ROOT/.venv`
- Node.js LTS integrated inside `.venv`
- Graphify installed
- CodeGraph installed (noting that local/IDE MCP server may report "codegraph executable not found in $PATH" unless run within `.venv` environment context)
- RTK (Rust Token Killer) installed at `.venv/bin/rtk`
- Repomix installed and configured with `repomix.config.json` (XML style output)
- Caveman (`@juliusbrussee/caveman-code`) installed and initialized
- CRUX-Compress installed (v2.10.4)

## Active Context & Guidelines
- Follow CRUX notation for semantic compression of rules/documentation.
- When executing tasks, activate the virtual environment (`source .venv/bin/activate`) so all virtualenv binaries (like `codegraph`, `repomix`, `caveman`) are in the active `$PATH`.
