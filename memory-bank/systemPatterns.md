# System Patterns

## Architectural Patterns
[2024-05-03 00:00:00] - Initial Documentation

### Process-Based Architecture
Openpilot uses a process-based architecture with distinct processes for different functionality areas (control, monitoring, sensor reading, etc.) that communicate through a messaging system.

### Pub/Sub Messaging
The codebase employs a publish-subscribe messaging pattern using capnp for serialization, allowing efficient communication between processes.

### Hardware Abstraction
Vehicle interfaces are abstracted through the panda hardware, providing a consistent API for different vehicle makes and models.

## Coding Patterns

### Module Organization
- **selfdrive/**: Core driving functionality
- **common/**: Shared utilities
- **system/**: System services
- **tools/**: Development and debugging tools

### Build System
- SCons-based build system
- Module-level SConscript files define build targets
- Consistent pattern for integrating new C/C++ modules

## Testing Patterns

### Test Organization
- Tests typically located in `tests/` directories within modules
- Unit tests for individual components
- Integration tests for system-level verification

### Testing Practices
- Pytest for Python testing
- Catch2 appears to be used for C++ testing
- Test files prefixed with `test_`

## FrogPilot Customization Patterns
[2024-05-03 00:15:00] - FrogPilot Extensions

### Extension Architecture
FrogPilot appears to extend the base openpilot functionality through a set of modules in selfdrive/frogpilot/ that enhance various aspects of vehicle control:

- **controls/lib/**: Custom control algorithms for acceleration, following, etc.
- **frogpilot_functions.py**: Core utility functions for FrogPilot features
- **frogpilot_variables.py**: Configuration constants and parameters

### Acceleration Control
The following patterns are observed in acceleration control:
- Multiple acceleration profiles (eco, sport, sport_plus)
- Adaptive acceleration based on lead vehicle behavior
- Speed-dependent acceleration limits

### Vehicle Following
Custom following behavior with these patterns:
- Dynamic time gap adjustment based on speed and conditions
- Lead vehicle acceleration mirroring for smoother following
- Traffic mode with specialized following parameters
