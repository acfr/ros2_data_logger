#!/bin/bash
# Deploy script for ros2_data_logger Debian packages to avocado.acfr.usyd.edu.au
# Usage: ./scripts/deploy.sh [--user username] [--ros-distro humble|iron|jazzy] [--arch amd64|arm64|all]

set -e

# Default values
DEPLOY_USER="${DEPLOY_USER:-jjustin}"
DEPLOY_HOST="avocado.acfr.usyd.edu.au"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ARCH="${ARCH:-all}"
REMOTE_BASE_PATH="/data/www/EHM/datasets/ubuntu-repo"
CODENAME="noble"  # Ubuntu 24.04 Noble Numbat

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
        --arch)
            ARCH="$2"
            shift 2
            ;;
        --host)
            DEPLOY_HOST="$2"
            shift 2
            ;;
        --codename)
            CODENAME="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--user username] [--ros-distro humble|iron|jazzy] [--arch amd64|arm64|all] [--host hostname] [--codename noble]"
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
echo -e "${BLUE}Architecture: ${ARCH}${NC}"
echo -e "${BLUE}Codename: ${CODENAME}${NC}"
echo -e "${BLUE}========================================${NC}"

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "${SCRIPT_DIR}")"
DEB_BASE_DIR="${PROJECT_ROOT}/debian_packages"

# Determine which architectures to deploy
if [ "${ARCH}" == "all" ]; then
    ARCHS=("amd64" "arm64")
else
    ARCHS=("${ARCH}")
fi

# Create remote repository structure if it doesn't exist
echo -e "${GREEN}Setting up remote repository structure...${NC}"
ssh "${DEPLOY_USER}@${DEPLOY_HOST}" "mkdir -p ${REMOTE_BASE_PATH}/{pool/main,dists/${CODENAME}/main/{binary-amd64,binary-arm64}}"

# Deploy packages for each architecture
DEPLOYED_ARCHS=()
for DEPLOY_ARCH in "${ARCHS[@]}"; do
    DEB_OUTPUT_DIR="${DEB_BASE_DIR}/${DEPLOY_ARCH}"
    
    if [ ! -d "${DEB_OUTPUT_DIR}" ] || [ -z "$(ls -A ${DEB_OUTPUT_DIR}/*.deb 2>/dev/null)" ]; then
        echo -e "${YELLOW}Warning: No ${DEPLOY_ARCH} packages found in ${DEB_OUTPUT_DIR}${NC}"
        echo -e "${YELLOW}Skipping ${DEPLOY_ARCH}...${NC}"
        continue
    fi
    
    # Display packages to be deployed
    echo -e "${GREEN}Packages to deploy for ${DEPLOY_ARCH}:${NC}"
    ls -lh "${DEB_OUTPUT_DIR}"/*.deb
    
    # Copy packages to remote pool
    echo -e "${GREEN}Copying ${DEPLOY_ARCH} packages to ${DEPLOY_HOST}...${NC}"
    scp "${DEB_OUTPUT_DIR}"/*.deb "${DEPLOY_USER}@${DEPLOY_HOST}:${REMOTE_BASE_PATH}/pool/main/"
    
    DEPLOYED_ARCHS+=("${DEPLOY_ARCH}")
done

# Check if any packages were deployed
if [ ${#DEPLOYED_ARCHS[@]} -eq 0 ]; then
    echo -e "${RED}Error: No packages were deployed${NC}"
    exit 1
fi

# Generate repository indices and metadata
echo -e "${GREEN}Generating repository indices...${NC}"
ssh "${DEPLOY_USER}@${DEPLOY_HOST}" << 'EOF'
    set -e
    
    REMOTE_BASE_PATH="/data/www/EHM/datasets/ubuntu-repo"
    CODENAME="noble"
    
    cd ${REMOTE_BASE_PATH}
    
    # Install dpkg-dev if not available
    if ! command -v dpkg-scanpackages &> /dev/null; then
        echo "Error: dpkg-dev not found. Please install it: sudo apt-get install dpkg-dev"
        exit 1
    fi
    
    # Generate indices for both architectures
    for ARCH in amd64 arm64; do
        BINARY_DIR="dists/${CODENAME}/main/binary-${ARCH}"
        
        echo "Generating index for ${ARCH}..."
        
        # Scan packages for this architecture
        dpkg-scanpackages --arch ${ARCH} pool/main 2>/dev/null | gzip -9c > ${BINARY_DIR}/Packages.gz
        dpkg-scanpackages --arch ${ARCH} pool/main 2>/dev/null > ${BINARY_DIR}/Packages
        
        # Create Release file for this architecture
        cat > ${BINARY_DIR}/Release << RELEASE_EOF
Archive: ${CODENAME}
Component: main
Architecture: ${ARCH}
RELEASE_EOF
        
        echo "  Created ${BINARY_DIR}/Packages.gz"
    done
    
    # Create main Release file for the distribution with checksums
    cd dists/${CODENAME}
    
    # Generate the Release file with proper hashes
    cat > Release << RELEASE_EOF
Origin: ACFR
Label: ACFR ROS2 Packages
Suite: stable
Codename: ${CODENAME}
Architectures: amd64 arm64
Components: main
Description: ACFR ROS2 packages repository
Date: $(date -u +"%a, %d %b %Y %H:%M:%S UTC")
RELEASE_EOF
    
    # Generate MD5Sum section
    echo "MD5Sum:" >> Release
    for ARCH in amd64 arm64; do
        if [ -f "main/binary-${ARCH}/Packages" ]; then
            echo " $(md5sum main/binary-${ARCH}/Packages | cut -d' ' -f1) $(stat -c%s main/binary-${ARCH}/Packages) main/binary-${ARCH}/Packages" >> Release
        fi
        if [ -f "main/binary-${ARCH}/Packages.gz" ]; then
            echo " $(md5sum main/binary-${ARCH}/Packages.gz | cut -d' ' -f1) $(stat -c%s main/binary-${ARCH}/Packages.gz) main/binary-${ARCH}/Packages.gz" >> Release
        fi
    done
    
    # Generate SHA1 section
    echo "SHA1:" >> Release
    for ARCH in amd64 arm64; do
        if [ -f "main/binary-${ARCH}/Packages" ]; then
            echo " $(sha1sum main/binary-${ARCH}/Packages | cut -d' ' -f1) $(stat -c%s main/binary-${ARCH}/Packages) main/binary-${ARCH}/Packages" >> Release
        fi
        if [ -f "main/binary-${ARCH}/Packages.gz" ]; then
            echo " $(sha1sum main/binary-${ARCH}/Packages.gz | cut -d' ' -f1) $(stat -c%s main/binary-${ARCH}/Packages.gz) main/binary-${ARCH}/Packages.gz" >> Release
        fi
    done
    
    # Generate SHA256 section
    echo "SHA256:" >> Release
    for ARCH in amd64 arm64; do
        if [ -f "main/binary-${ARCH}/Packages" ]; then
            echo " $(sha256sum main/binary-${ARCH}/Packages | cut -d' ' -f1) $(stat -c%s main/binary-${ARCH}/Packages) main/binary-${ARCH}/Packages" >> Release
        fi
        if [ -f "main/binary-${ARCH}/Packages.gz" ]; then
            echo " $(sha256sum main/binary-${ARCH}/Packages.gz | cut -d' ' -f1) $(stat -c%s main/binary-${ARCH}/Packages.gz) main/binary-${ARCH}/Packages.gz" >> Release
        fi
    done
    
    cd ${REMOTE_BASE_PATH}
    echo "Repository structure updated successfully!"
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
        <h3>Ubuntu 24.04 (Noble Numbat)</h3>
        <p><a href="dists/noble/">Browse distributions</a> | <a href="pool/main/">Browse packages</a></p>
    </div>
    
    <h2>Installation Instructions</h2>
    <p>To use this repository, add it to your APT sources. The repository supports both AMD64 and ARM64 architectures.</p>
    
    <pre>
# Add the repository (works for both AMD64 and ARM64)
echo "deb [trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo noble main" | sudo tee /etc/apt/sources.list.d/acfr-ros2.list

# Update package list
sudo apt update

# Install packages (example)
sudo apt install ros-jazzy-ros2-data-logger
    </pre>
    
    <h2>Available Packages</h2>
    <ul>
        <li><strong>ros2_data_logger</strong> - Flexible data logging for ROS2 robotic platforms</li>
        <li><strong>adnav</strong> - Advanced Navigation driver and interfaces</li>
        <li><strong>roboteq-driver</strong> - Roboteq motor controller driver</li>
        <li><strong>ros2-hetronic</strong> - Hetronic remote control interfaces</li>
        <li><strong>swerve-controller</strong> - ROS2 swerve drive controller</li>
    </ul>
    
    <h2>Repository Structure</h2>
    <p>This repository follows the standard Debian repository layout:</p>
    <ul>
        <li><code>dists/noble/</code> - Distribution metadata and package indices</li>
        <li><code>pool/main/</code> - Package pool containing all .deb files</li>
    </ul>
    
    <h2>Supported Architectures</h2>
    <ul>
        <li>AMD64 (x86_64)</li>
        <li>ARM64 (aarch64)</li>
    </ul>
    
    <p><em>Last updated: $(date)</em></p>
</body>
</html>
HTML_EOF

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Deployment completed successfully!${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}Repository URL: https://data.acfr.usyd.edu.au/ubuntu-repo${NC}"
echo -e "${BLUE}To install packages:${NC}"
echo ""
echo -e "${YELLOW}# Add the repository (supports both AMD64 and ARM64)${NC}"
echo -e "${YELLOW}echo \"deb [trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo ${CODENAME} main\" | sudo tee /etc/apt/sources.list.d/acfr-ros2.list${NC}"
echo ""
echo -e "${YELLOW}# Update and install${NC}"
echo -e "${YELLOW}sudo apt update${NC}"
echo -e "${YELLOW}sudo apt install ros-${ROS_DISTRO}-ros2-data-logger${NC}"
echo -e "${GREEN}========================================${NC}"
