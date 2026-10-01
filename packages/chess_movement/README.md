# Universal Package Template

A comprehensive ROS2 package template that works with our package manager.

## What This Template Provides

This is a **working example package** that demonstrates:
- How to structure both C++ and Python code in one package
- How to configure dependencies for different deployment targets
- How to set up testing, documentation, and configuration
- How to use our custom package manager extensions alongside standard ROS

## Quick Start

> TODO: add build setup

```bash
# 1. Copy this template
cp -r template your_package_name
cd your_package_name

# 2. Edit package.xml following the checklist
# 3. Replace "template" with your package name in all files
# 4. Build: #################
```

## Folder Structure

```
packages/template/
├── CMakeLists.txt           # C++ build configuration
├── docs/                    # Documentation and guides
├── examples/                # Usage examples and demos
├── include/                 # Public C++ headers
├── launch/                  # ROS launch files
├── package.xml              # Package manifest and configuration
├── README.md               # This file
├── scripts/                # Executable tools and build scripts
├── setup.cfg               # Python build configuration
├── setup.py                # Python package definition
└── test/                   # Unit tests and test data
```

### What Each Folder Contains

**`docs/`** - Documentation for your package including API docs, usage guides, and examples. Auto-generated documentation goes here.

**`examples/`** - Standalone examples showing how to use your package. Include both simple and advanced usage patterns.

**`include/`** - Public C++ header files that other packages can use if your package is a library. Only put headers here that you want to expose.

**`launch/`** - ROS launch files that start your nodes with proper configuration. These orchestrate how your package runs in the ROS ecosystem.

**`scripts/`** - Executable scripts and build utilities. Put standalone tools and helper scripts here.

**`test/`** - All your testing code including unit tests, integration tests, and test data files.

## Core Components

### package.xml - Package Configuration
The heart of your package. This file declares what your package is, what it depends on, and how our package manager should deploy it. **Start here** - it contains comprehensive documentation and a step-by-step checklist.

### CMake Build System
**`CMakeLists.txt`** - Defines how to build C++ code, link libraries, and install files. CMake is the standard build system for C++ in ROS and handles cross-compilation for different target architectures.

**Why CMake**: Handles complex dependency resolution, cross-platform compilation, and integrates seamlessly with the ROS build system.

### Python Build System
**`setup.py`** - Defines how to build and install Python code. This is the standard Python packaging mechanism that integrates with both ROS and system package managers.

**`setup.cfg`** - Additional Python build configuration including entry points, dependencies, and metadata.

**Why Python packaging**: Allows your Python code to be installed system-wide, imported by other packages, and distributed through standard channels.

### ROS Integration
**Launch files** define how your nodes start up, what parameters they use, and how they connect to other parts of the robot system.

**Why launch files**: ROS systems are distributed - launch files orchestrate multiple processes across potentially multiple machines.

## How to Learn

**Explore each file** to understand the patterns and structure. Each file contains:
- Examples of proper usage
- Comments explaining the purpose
- Links to relevant documentation

**Start with `package.xml`** for the overall configuration, then examine the build files (`CMakeLists.txt`, `setup.py`) that match your code type.

The template shows a complete working package with both C++ and Python components. Use what you need, remove what you don't.