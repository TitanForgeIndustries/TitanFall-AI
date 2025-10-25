# Titan Fall AI - Visual Studio Code Setup

## Overview
This project is now configured for Visual Studio Code with full ROS 2, C++, and Python support.

## Quick Start

1. **Install VS Code Extensions** (automated):
   ```bash
   ./setup_vscode.sh
   ```

2. **Open in VS Code**:
   ```bash
   code .
   ```
   Or open the workspace file:
   ```bash
   code .vscode/titanfall_ai.code-workspace
   ```

## VS Code Configuration Files

### `.vscode/settings.json`
- C++ IntelliSense configuration
- Python environment settings
- File associations for ROS 2 files
- Build directory exclusions
- Code formatting settings

### `.vscode/tasks.json`
Predefined tasks for common operations:
- **Build ROS2 Package** (Ctrl+Shift+B / Cmd+Shift+B)
- **Clean Build**
- **Run Gazebo Simulation**
- **Run Full Simulation**
- **List Topics**
- **Generate Compile Commands**

### `.vscode/launch.json`
Debug configurations:
- **Debug Titan Fall AI Node** - Debug C++ ROS 2 node
- **Debug with GDB** - Alternative GDB debugger
- **Attach to Process** - Attach to running process
- **Debug Python Launch File** - Debug Gazebo launch
- **Debug Full Simulation** - Debug full system

### `.vscode/c_cpp_properties.json`
- C/C++ include paths
- Compiler settings
- IntelliSense configuration
- ROS 2 headers

### `.vscode/extensions.json`
Recommended extensions:
- C/C++ Extension Pack
- CMake Tools
- Python
- YAML Language Support

## Building the Project

### Using VS Code Tasks
1. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
2. Type "Tasks: Run Task"
3. Select "Build ROS2 Package"

Or use the keyboard shortcut: `Ctrl+Shift+B` / `Cmd+Shift+B`

### Using Terminal
```bash
colcon build --packages-select titanfall_ai
source install/setup.bash
```

## Debugging

### Debug C++ Node
1. Build the project first
2. Press `F5` or go to Run → Start Debugging
3. Select "Debug Titan Fall AI Node"

### Debug Python Launch File
1. Open a launch file
2. Press `F5`
3. Select "Debug Python Launch File"

## Running Simulations

### Via Tasks
1. Press `Ctrl+Shift+P` / `Cmd+Shift+P`
2. Type "Tasks: Run Task"
3. Select:
   - "Run Gazebo Simulation" - Launch Gazebo only
   - "Run Full Simulation" - Launch full system with Rviz

### Via Terminal
```bash
# Source environment first
source install/setup.bash

# Run Gazebo simulation
ros2 launch titanfall_ai gazebo_world.launch.py

# Or run full simulation
ros2 launch titanfall_ai titanfall_sim.launch.py
```

## IntelliSense and Code Completion

### C++ IntelliSense
The project is configured with:
- ROS 2 Humble include paths
- C++17 standard
- Clang/GCC compiler support
- Compile commands database

If IntelliSense doesn't work:
1. Run "Generate Compile Commands" task
2. Reload VS Code window: `Ctrl+Shift+P` → "Reload Window"

### Python IntelliSense
Configured with:
- Python 3 interpreter
- PyLint linting
- Black formatter
- Auto-formatting on save

## Keyboard Shortcuts

| Action | Mac | Windows/Linux |
|--------|-----|---------------|
| Build | `Cmd+Shift+B` | `Ctrl+Shift+B` |
| Debug | `F5` | `F5` |
| Command Palette | `Cmd+Shift+P` | `Ctrl+Shift+P` |
| Quick Open | `Cmd+P` | `Ctrl+P` |
| Terminal | `` Ctrl+` `` | `` Ctrl+` `` |
| Find in Files | `Cmd+Shift+F` | `Ctrl+Shift+F` |

## Recommended Workflow

1. **Edit Code** - Use IntelliSense for auto-completion
2. **Build** - `Cmd+Shift+B` / `Ctrl+Shift+B`
3. **Check Errors** - View in Problems panel
4. **Debug** - Set breakpoints and press `F5`
5. **Run** - Use tasks or integrated terminal

## File Structure Navigation

- **Go to Symbol**: `Cmd+Shift+O` / `Ctrl+Shift+O`
- **Go to File**: `Cmd+P` / `Ctrl+P`
- **Go to Definition**: `F12`
- **Peek Definition**: `Alt+F12`
- **Find References**: `Shift+F12`

## Troubleshooting

### IntelliSense Not Working
1. Check `.vscode/c_cpp_properties.json` has correct paths
2. Run "Generate Compile Commands" task
3. Reload VS Code window

### Build Fails
1. Ensure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`
2. Clean build: Run "Clean Build" task
3. Check CMakeLists.txt for errors

### Debugger Won't Start
1. Build project first with debug symbols
2. Check executable path in launch.json
3. Ensure lldb or gdb is installed

### Python Extension Issues
1. Select correct Python interpreter: `Cmd+Shift+P` → "Python: Select Interpreter"
2. Install Python dependencies: `pip install -r requirements.txt`

## Additional Resources

- [VS Code C++ Documentation](https://code.visualstudio.com/docs/cpp/cpp-ide)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [CMake Tools Extension](https://github.com/microsoft/vscode-cmake-tools)
- [Python Extension](https://marketplace.visualstudio.com/items?itemName=ms-python.python)

## Contact & Support

For issues specific to this project setup, please check the main README.md or contact the project maintainers.
