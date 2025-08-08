# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the **Recoil Engine** codebase - an open source real-time strategy game engine forked from Spring RTS version 105.0. It's primarily written in C++20 and uses CMake as the build system.

## Build Commands

### Basic Build
```bash
cmake .
ninja
```

### Build with Tests
```bash
cmake .
make tests
make test
```

### Docker-based Build
For cross-platform builds, Docker build scripts are available in `docker-build-v2/`:
```bash
cd docker-build-v2
./build.sh
```

### Build Configuration
The project requires:
- CMake 3.27+
- C++20 compiler (GCC 7.0+)
- Git with proper tags (ensure upstream tags are fetched)

If working with a fork, always fetch upstream tags:
```bash
git remote add upstream https://github.com/beyond-all-reason/RecoilEngine.git
git fetch --all --tags
```

## Architecture Overview

### Core Directories
- `rts/` - Main engine source code (Real-Time Strategy engine)
  - `System/` - Core engine systems, utilities, and platform abstraction
  - `Game/` - Game logic, controllers, UI components
  - `Sim/` - Simulation systems (units, physics, pathfinding)
  - `Rendering/` - Graphics rendering pipeline
  - `Lua/` - Lua scripting integration for game logic
  - `Map/` - Map loading, terrain rendering, heightmaps
  - `Net/` - Network protocol and multiplayer support
  - `ExternalAI/` - AI interface system

- `AI/` - AI implementations and interfaces
  - `Skirmish/` - Various AI implementations (CircuitAI, BARb, etc.)
  - `Interfaces/` - AI interface bindings (C, Java)
  - `Wrappers/` - Language wrappers for AI development

- `cont/` - Game content and assets
  - `LuaUI/` - Lua UI framework and widgets
  - `base/` - Core game assets and content

### Key Components
- **SpringApp** (`rts/System/SpringApp.h`) - Main application entry point
- **Game** (`rts/Game/Game.h`) - Core game state management  
- **EventHandler** (`rts/System/EventHandler.h`) - Event system for Lua integration
- **LuaHandle** (`rts/Lua/LuaHandle.h`) - Lua scripting interface base class
- **ReadMap** (`rts/Map/ReadMap.h`) - Map loading and management
- **PathFinder** - Pathfinding systems for unit movement

### Lua Integration
The engine heavily uses Lua for:
- Game rules and logic (`LuaRules`)
- UI widgets (`LuaUI`) 
- AI scripts (`LuaAI`)
- Game content configuration

### Build System Notes
- Uses CMake with custom modules in `rts/build/cmake/`
- Supports both static and shared library builds
- Cross-platform builds via Docker containers
- Unit tests use Catch2 framework

### Development Workflow
- Main branch: `master`
- Requires proper git tags for version detection
- Releases happen every 2-3 months
- Consider getting familiar with the build process before contributing
- Keep game-specific logic out of the engine core