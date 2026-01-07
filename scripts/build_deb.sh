#!/bin/bash
# Build script for creating Debian package of ros2_data_logger
# Usage: ./scripts/build_deb.sh [--ros-distro humble|iron|jazzy] [--arch amd64|arm64|all]

set -e

# Default ROS distro and architecture (default to "all" to build both architectures)
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ARCH="${ARCH:-all}"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --ros-distro)
            ROS_DISTRO="$2"
            shift 2
            ;;
        --arch)
            ARCH="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--ros-distro humble|iron|jazzy] [--arch amd64|arm64|all]"
            exit 1
            ;;
    esac
done

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Building ros2_data_logger Debian package${NC}"
echo -e "${BLUE}ROS Distro: ${ROS_DISTRO}${NC}"
echo -e "${BLUE}Architecture: ${ARCH}${NC}"
echo -e "${BLUE}========================================${NC}"

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "${SCRIPT_DIR}")"
WORKSPACE_ROOT="$(dirname "$(dirname "${PROJECT_ROOT}")")"

echo -e "${GREEN}Project root: ${PROJECT_ROOT}${NC}"
echo -e "${GREEN}Workspace root: ${WORKSPACE_ROOT}${NC}"

# Source ROS setup
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo -e "${GREEN}Sourcing ROS ${ROS_DISTRO} setup...${NC}"
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo -e "${RED}Error: ROS ${ROS_DISTRO} is not installed${NC}"
    exit 1
fi

# Install bloom if not already installed
if ! command -v bloom-generate &> /dev/null; then
    echo -e "${GREEN}Installing bloom...${NC}"
    sudo apt-get update
    sudo apt-get install -y python3-bloom
fi

# Install rosdep dependencies
echo -e "${GREEN}Installing rosdep dependencies...${NC}"
cd "${WORKSPACE_ROOT}"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the package
echo -e "${GREEN}Building the package...${NC}"
cd "${WORKSPACE_ROOT}"
colcon build --packages-select ros2_data_logger --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Create debian directory
echo -e "${GREEN}Generating Debian package files with bloom...${NC}"
cd "${PROJECT_ROOT}"

# Create a build directory for debian packages
BUILD_DIR="${PROJECT_ROOT}/build_deb"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# Generate Debian files using bloom
bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro ${ROS_DISTRO}

# Build the Debian package
echo -e "${GREEN}Building Debian package for ${ARCH}...${NC}"
if [ "${ARCH}" == "all" ]; then
    # Build for current architecture when "all" is specified (build_and_deploy.sh handles multi-arch)
    CURRENT_ARCH="$(dpkg --print-architecture)"
    echo -e "${YELLOW}Building for current architecture: ${CURRENT_ARCH}${NC}"
    echo -e "${YELLOW}Note: Use build_and_deploy.sh to build both architectures${NC}"
    DEB_BUILD_OPTIONS="nocheck" DEB_BUILD_ARCH="${CURRENT_ARCH}" fakeroot debian/rules binary
    # Update ARCH for file organization
    ARCH="${CURRENT_ARCH}"
else
    # Build for specific architecture
    DEB_BUILD_OPTIONS="nocheck" DEB_BUILD_ARCH="${ARCH}" fakeroot debian/rules binary
fi

# Move the .deb file to a known location with architecture subfolder
DEB_OUTPUT_DIR="${PROJECT_ROOT}/debian_packages/${ARCH}"
mkdir -p "${DEB_OUTPUT_DIR}"

# Find and move the generated .deb files
find "${WORKSPACE_ROOT}" -maxdepth 2 -name "ros-${ROS_DISTRO}-ros2-data-logger*.deb" -exec mv {} "${DEB_OUTPUT_DIR}/" \;

if ls "${DEB_OUTPUT_DIR}"/ros-${ROS_DISTRO}-ros2-data-logger*.deb 1> /dev/null 2>&1; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Debian package built successfully!${NC}"
    echo -e "${GREEN}Architecture: ${ARCH}${NC}"
    echo -e "${GREEN}Location: ${DEB_OUTPUT_DIR}${NC}"
    ls -lh "${DEB_OUTPUT_DIR}"/*.deb
    echo -e "${GREEN}========================================${NC}"
else
    echo -e "${RED}Error: Failed to build Debian package${NC}"
    exit 1
fi

# Clean up bloom-generated debian directory
rm -rf debian

echo -e "${GREEN}Build complete!${NC}"
