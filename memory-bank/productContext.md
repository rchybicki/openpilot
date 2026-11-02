# Product Context: Openpilot with FrogPilot

## Project Overview
[2024-05-03 00:00:00] - Initial Setup

Openpilot is an open-source driver assistance system developed for modern cars. It provides adaptive cruise control, automated lane centering, lane keeping, and other driver assistance features. The system runs on compatible hardware and interfaces with the vehicle's existing systems.

[2024-05-03 00:15:00] - FrogPilot Extension
This repository contains a customized version of openpilot called FrogPilot, which extends the standard functionality with additional features and customizations for a more refined driving experience. FrogPilot appears to add enhanced acceleration profiles, improved lead vehicle following behavior, and various quality-of-life improvements.

## System Components

### Core Components:
- **selfdrive/**: Core driving code implementation
- **common/**: Shared utilities and helper functions
- **system/**: System services and utilities
- **panda/**: Hardware interface for vehicle communication
- **cereal/**: Messaging interfaces using capnp
- **opendbc/**: Vehicle-specific CAN bus definitions

### FrogPilot Components:
- **selfdrive/frogpilot/**: FrogPilot customizations and extensions
  - **controls/**: Custom control algorithms
  - **frogpilot_functions.py**: Core utility functions
  - **frogpilot_utilities.py**: Helper utilities
  - **frogpilot_variables.py**: Configuration constants

### Key Subsystems:
- **Controls**: Path planning and vehicle control
- **Monitoring**: Driver monitoring systems
- **UI**: User interface components
- **Modeling**: Machine learning models for perception

## Technical Specifications

### Development Environment:
- Primarily Python and C++ based
- Build system: SCons
- Python version: 3.11
- Testing framework: pytest

### Code Organization:
- Modular architecture separating vehicle interfaces from control logic
- Messaging-based communication between processes
- Hardware abstraction for vehicle interfaces
- FrogPilot extensions that enhance base functionality

## Development Standards

### Code Style:
- Python: Follow ruff linting rules in pyproject.toml
- Line length: 160 characters maximum
- Error handling: Explicit over generic try/except blocks
- Imports: Use openpilot module prefix (e.g., from openpilot.selfdrive)

### Testing Standards:
- Unit tests with pytest
- Integration tests for vehicle interfaces
- Performance benchmarks for critical systems

## Current Branch and Status
- Current branch: !my-fp
- Recent changes: Reverted "disabled following clip" functionality

## FrogPilot Features
[2024-05-03 00:15:00] - Feature Analysis

### Driving Profiles:
- Eco: Optimized for efficiency with gentler acceleration
- Sport: More responsive with increased acceleration
- Sport Plus: Maximum responsiveness with aggressive acceleration

### Adaptive Following:
- Dynamic time gap adjustment based on driving conditions
- Human-like following behavior with lead acceleration adaptation
- Traffic mode with specialized following parameters for congested areas

### Customized Controls:
- Conditional experimental mode based on driving conditions
- Adjustable acceleration and deceleration profiles
- Road curvature detection and handling