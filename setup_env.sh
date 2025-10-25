#!/bin/bash

# TITAN-FALL-AI Environment Setup Script
# Author: Bryce K. Courtney | Titan Forge Industries

set -e

echo "=========================================="
echo "TITAN-FALL-AI Environment Setup"
echo "Author: Bryce K. Courtney"
echo "Titan Forge Industries"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on supported OS
check_os() {
    print_status "Checking operating system..."
    
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if command -v lsb_release &> /dev/null; then
            OS_VERSION=$(lsb_release -rs)
            if [[ "$OS_VERSION" == "20.04" ]] || [[ "$OS_VERSION" == "22.04" ]]; then
                print_success "Ubuntu $OS_VERSION detected"
                OS_TYPE="ubuntu"
            else
                print_warning "Ubuntu $OS_VERSION detected (may not be fully supported)"
                OS_TYPE="ubuntu"
            fi
        else
            print_warning "Linux detected but version unknown"
            OS_TYPE="linux"
        fi
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        print_success "macOS detected"
        OS_TYPE="macos"
    else
        print_error "Unsupported operating system: $OSTYPE"
        exit 1
    fi
}

# Check if ROS 2 is installed
check_ros2() {
    print_status "Checking ROS 2 installation..."
    
    if command -v ros2 &> /dev/null; then
        ROS_VERSION=$(ros2 --version | head -n1 | awk '{print $3}')
        print_success "ROS 2 $ROS_VERSION found"
        
        # Check if ROS 2 is sourced
        if [[ -z "$ROS_DISTRO" ]]; then
            print_warning "ROS 2 not sourced. Please run: source /opt/ros/$ROS_VERSION/setup.bash"
        else
            print_success "ROS 2 $ROS_DISTRO is sourced"
        fi
    else
        print_error "ROS 2 not found. Please install ROS 2 first."
        print_status "Installation instructions: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi
}

# Check if Gazebo is installed
check_gazebo() {
    print_status "Checking Gazebo installation..."
    
    if command -v gazebo &> /dev/null; then
        GAZEBO_VERSION=$(gazebo --version | head -n1 | awk '{print $3}')
        print_success "Gazebo Classic $GAZEBO_VERSION found"
    elif command -v gz &> /dev/null; then
        GZ_VERSION=$(gz --version | head -n1 | awk '{print $3}')
        print_success "Gazebo Ignition $GZ_VERSION found"
    else
        print_warning "Gazebo not found. Simulation features may not work."
        print_status "Installation instructions: https://gazebosim.org/docs"
    fi
}

# Check if colcon is installed
check_colcon() {
    print_status "Checking colcon installation..."
    
    if command -v colcon &> /dev/null; then
        print_success "colcon found"
    else
        print_error "colcon not found. Please install colcon:"
        print_status "pip3 install colcon-common-extensions"
        exit 1
    fi
}

# Install dependencies
install_dependencies() {
    print_status "Installing dependencies..."
    
    if [[ "$OS_TYPE" == "ubuntu" ]]; then
        print_status "Installing Ubuntu packages..."
        
        # Update package list
        sudo apt update
        
        # Install ROS 2 packages
        sudo apt install -y \
            ros-humble-rclcpp \
            ros-humble-std-msgs \
            ros-humble-sensor-msgs \
            ros-humble-geometry-msgs \
            ros-humble-nav-msgs \
            ros-humble-tf2 \
            ros-humble-tf2-ros \
            ros-humble-tf2-geometry-msgs \
            ros-humble-gazebo-ros \
            ros-humble-gazebo-ros-pkgs \
            ros-humble-gazebo-ros-control \
            ros-humble-controller-manager \
            ros-humble-joint-state-broadcaster \
            ros-humble-joint-trajectory-controller \
            ros-humble-position-controllers \
            ros-humble-velocity-controllers \
            ros-humble-effort-controllers \
            python3-colcon-common-extensions \
            python3-rosdep
        
        print_success "Ubuntu packages installed"
        
    elif [[ "$OS_TYPE" == "macos" ]]; then
        print_status "Installing macOS packages..."
        
        # Check if Homebrew is installed
        if ! command -v brew &> /dev/null; then
            print_error "Homebrew not found. Please install Homebrew first."
            print_status "Installation instructions: https://brew.sh"
            exit 1
        fi
        
        # Install packages via Homebrew
        brew install cmake pkg-config
        
        print_success "macOS packages installed"
    fi
}

# Setup ROS 2 workspace
setup_workspace() {
    print_status "Setting up ROS 2 workspace..."
    
    # Create workspace if it doesn't exist
    if [[ ! -d "build" ]]; then
        print_status "Creating workspace structure..."
        mkdir -p build install log
    fi
    
    # Initialize rosdep if not already done
    if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
        print_status "Initializing rosdep..."
        sudo rosdep init
    fi
    
    rosdep update
    
    print_success "Workspace setup complete"
}

# Build the package
build_package() {
    print_status "Building Titan Fall AI package..."
    
    # Source ROS 2
    if [[ -n "$ROS_DISTRO" ]]; then
        source /opt/ros/$ROS_DISTRO/setup.bash
    fi
    
    # Install dependencies
    rosdep install --from-paths . --ignore-src -r -y
    
    # Build with colcon
    colcon build --packages-select titanfall_ai
    
    if [[ $? -eq 0 ]]; then
        print_success "Package built successfully"
    else
        print_error "Build failed"
        exit 1
    fi
}

# Create environment setup script
create_env_script() {
    print_status "Creating environment setup script..."
    
    cat > setup_env.bash << 'ENV_EOF'
#!/bin/bash
# Titan Fall AI Environment Setup
# Source this file to set up the environment

# Source ROS 2
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    source /opt/ros/humble/setup.bash
    echo "ROS 2 Humble sourced"
elif [[ -f "/opt/ros/galactic/setup.bash" ]]; then
    source /opt/ros/galactic/setup.bash
    echo "ROS 2 Galactic sourced"
fi

# Source workspace
if [[ -f "install/setup.bash" ]]; then
    source install/setup.bash
    echo "Titan Fall AI workspace sourced"
fi

# Set environment variables
export TITANFALL_AI_ROOT=$(pwd)
export GAZEBO_MODEL_PATH=$TITANFALL_AI_ROOT/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$TITANFALL_AI_ROOT/worlds:$GAZEBO_RESOURCE_PATH

echo "Titan Fall AI environment ready!"
ENV_EOF
    
    chmod +x setup_env.bash
    print_success "Environment setup script created"
}

# Main setup function
main() {
    print_status "Starting Titan Fall AI environment setup..."
    
    check_os
    check_ros2
    check_gazebo
    check_colcon
    
    # Ask user if they want to install dependencies
    read -p "Do you want to install dependencies? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        install_dependencies
    fi
    
    setup_workspace
    
    # Ask user if they want to build the package
    read -p "Do you want to build the package now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        build_package
    fi
    
    create_env_script
    
    print_success "Setup complete!"
    print_status "Next steps:"
    print_status "1. Source the environment: source setup_env.bash"
    print_status "2. Run the simulation: ros2 launch titanfall_ai titanfall_sim.launch.py"
    print_status "3. Check the README.md for more information"
}

# Run main function
main "$@"
