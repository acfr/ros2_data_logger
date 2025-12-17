#!/bin/bash
# Deploy script for ros2_data_logger Debian packages to avocado.acfr.usyd.edu.au
# Usage: ./scripts/deploy.sh [--user username] [--ros-distro humble|iron|jazzy]

set -e

# Default values
DEPLOY_USER="${DEPLOY_USER:-jerome}"
DEPLOY_HOST="avocado.acfr.usyd.edu.au"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REMOTE_BASE_PATH="/data/www/EHM/datasets/ubuntu-repo"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --user)
            DEPLOY_USER="$2"
            shift 2
            ;;
        --ros-distro)
            ROS_DISTRO="$2"
            shift 2
            ;;
        --host)
            DEPLOY_HOST="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--user username] [--ros-distro humble|iron|jazzy] [--host hostname]"
            exit 1
            ;;
    esac
done

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Deploying ros2_data_logger to ${DEPLOY_HOST}${NC}"
echo -e "${BLUE}User: ${DEPLOY_USER}${NC}"
echo -e "${BLUE}ROS Distro: ${ROS_DISTRO}${NC}"
echo -e "${BLUE}========================================${NC}"

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "${SCRIPT_DIR}")"
DEB_OUTPUT_DIR="${PROJECT_ROOT}/debian_packages"

# Check if debian packages exist
if [ ! -d "${DEB_OUTPUT_DIR}" ] || [ -z "$(ls -A ${DEB_OUTPUT_DIR}/*.deb 2>/dev/null)" ]; then
    echo -e "${RED}Error: No Debian packages found in ${DEB_OUTPUT_DIR}${NC}"
    echo -e "${YELLOW}Please run ./scripts/build_deb.sh first${NC}"
    exit 1
fi

# Display packages to be deployed
echo -e "${GREEN}Packages to deploy:${NC}"
ls -lh "${DEB_OUTPUT_DIR}"/*.deb

# Create remote directory structure if it doesn't exist
echo -e "${GREEN}Setting up remote directory structure...${NC}"
ssh "${DEPLOY_USER}@${DEPLOY_HOST}" "mkdir -p ${REMOTE_BASE_PATH}/${ROS_DISTRO}"

# Copy packages to remote server
echo -e "${GREEN}Copying packages to ${DEPLOY_HOST}...${NC}"
scp "${DEB_OUTPUT_DIR}"/*.deb "${DEPLOY_USER}@${DEPLOY_HOST}:${REMOTE_BASE_PATH}/${ROS_DISTRO}/"

# Create or update the package index
echo -e "${GREEN}Creating package repository index...${NC}"
ssh "${DEPLOY_USER}@${DEPLOY_HOST}" << EOF
    cd ${REMOTE_BASE_PATH}/${ROS_DISTRO}
    
    # Install dpkg-dev if not available
    if ! command -v dpkg-scanpackages &> /dev/null; then
        echo "Warning: dpkg-dev not found. Please install it: apt-get install dpkg-dev"
        exit 1
    fi
    
    # Generate Packages file
    dpkg-scanpackages . /dev/null | gzip -9c > Packages.gz
    dpkg-scanpackages . /dev/null > Packages
    
    # Create Release file
    cat > Release << 'RELEASE_EOF'
Origin: ACFR
Label: ROS2 Data Logger Repository
Suite: stable
Codename: ${ROS_DISTRO}
Architectures: amd64 arm64
Components: main
Description: ACFR ROS2 Data Logger packages for ${ROS_DISTRO}
Date: $(date -R)
RELEASE_EOF
    
    echo "Repository updated at: ${REMOTE_BASE_PATH}/${ROS_DISTRO}"
    ls -lh
EOF

# Create an index.html for the repository
echo -e "${GREEN}Creating repository index page...${NC}"
ssh "${DEPLOY_USER}@${DEPLOY_HOST}" "cat > ${REMOTE_BASE_PATH}/index.html" << 'HTML_EOF'
<!DOCTYPE html>
<html>
<head>
    <title>ACFR ROS2 Packages Repository</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; }
        h1 { color: #333; }
        .distro { margin: 20px 0; padding: 15px; background-color: #f5f5f5; border-radius: 5px; }
        code { background-color: #e0e0e0; padding: 2px 6px; border-radius: 3px; }
        pre { background-color: #272822; color: #f8f8f2; padding: 15px; border-radius: 5px; overflow-x: auto; }
    </style>
</head>
<body>
    <h1>ACFR ROS2 Packages Repository</h1>
    <p>Welcome to the ACFR ROS2 packages repository</p>
    
    <h2>Available Distributions</h2>
    <div class="distro">
        <h3>ROS2 Jazzy</h3>
        <p><a href="jazzy/">Browse packages</a></p>
    </div>
    
    <h2>Installation Instructions</h2>
    <p>To use this repository, add it to your APT sources:</p>
    <pre>
# Add the repository
echo "deb [trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo/jazzy ./" | sudo tee /etc/apt/sources.list.d/acfr-ros2.list

# Update package list
sudo apt update

# Install packages
sudo apt install ros-jazzy-ros2-data-logger
    </pre>
    
    <h2>Packages</h2>
    <ul>
        <li><strong>ros2_data_logger</strong> - Flexible data logging for ROS2 robotic platforms</li>
    </ul>
    
    <p><em>Last updated: $(date)</em></p>
</body>
</html>
HTML_EOF

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Deployment completed successfully!${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}Repository URL: https://data.acfr.usyd.edu.au/ubuntu-repo/${ROS_DISTRO}${NC}"
echo -e "${BLUE}To install the package:${NC}"
echo -e "${YELLOW}  echo \"deb [trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo/${ROS_DISTRO} ./\" | sudo tee /etc/apt/sources.list.d/acfr-ros2.list${NC}"
echo -e "${YELLOW}  sudo apt update${NC}"
echo -e "${YELLOW}  sudo apt install ros-${ROS_DISTRO}-ros2-data-logger${NC}"
echo -e "${GREEN}========================================${NC}"
