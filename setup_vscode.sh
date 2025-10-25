#!/bin/bash

echo "=========================================="
echo "  Titan Fall AI - VS Code Setup"
echo "=========================================="
echo ""

# Check if VS Code is installed
if ! command -v code &> /dev/null; then
    echo "❌ Visual Studio Code is not installed or 'code' command is not in PATH"
    echo "Please install VS Code from: https://code.visualstudio.com/"
    echo "And enable 'code' command in PATH"
    exit 1
fi

echo "✅ VS Code found"
echo ""

# Install recommended extensions
echo "📦 Installing recommended VS Code extensions..."
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-python.python
code --install-extension redhat.vscode-yaml
code --install-extension twxs.cmake

echo ""
echo "✅ Extensions installed"
echo ""

# Create build directory if it doesn't exist
if [ ! -d "build" ]; then
    mkdir -p build
    echo "✅ Created build directory"
fi

# Generate compile_commands.json for IntelliSense
echo "🔨 Generating compile commands..."
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
cd ..

if [ -f "build/compile_commands.json" ]; then
    echo "✅ Compile commands generated"
else
    echo "⚠️  Could not generate compile commands (this is okay for ROS2 projects)"
fi

echo ""
echo "=========================================="
echo "  VS Code Setup Complete!"
echo "=========================================="
echo ""
echo "To open the project in VS Code, run:"
echo "  code ."
echo ""
echo "Or open the workspace file:"
echo "  code .vscode/titanfall_ai.code-workspace"
echo ""
