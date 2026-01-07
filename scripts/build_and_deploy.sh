#!/bin/bash
# Quick build and deploy script
# Usage: ./scripts/build_and_deploy.sh [--ros-distro humble|iron|jazzy] [--arch amd64|arm64|all]

set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parse arguments to check if architecture was provided
ARCH=""
ARCH_PROVIDED=false
for arg in "$@"; do
    if [[ "$prev_arg" == "--arch" ]]; then
        ARCH="$arg"
        ARCH_PROVIDED=true
    fi
    prev_arg="$arg"
done

# If no architecture provided, prompt the user
if [[ "$ARCH_PROVIDED" == false ]]; then
    echo "Select architecture to build and deploy:"
    echo "1. ARM64"
    echo "2. AMD64"
    echo "3. Both"
    echo -n "Enter option (1-3): "
    read -r option
    
    case $option in
        1)
            ARCH="arm64"
            ;;
        2)
            ARCH="amd64"
            ;;
        3)
            ARCH="all"
            ;;
        *)
            echo "Invalid option. Exiting."
            exit 1
            ;;
    esac
    
    # Add architecture to arguments
    set -- "$@" "--arch" "$ARCH"
fi

# If building for all architectures, build each separately
if [[ "${ARCH}" == "all" ]]; then
    echo "Building for all architectures..."
    
    # Build for amd64
    echo "Building for amd64..."
    MODIFIED_ARGS=()
    skip_next=false
    for arg in "$@"; do
        if [[ "$skip_next" == true ]]; then
            skip_next=false
            continue
        fi
        if [[ "$arg" == "--arch" ]]; then
            MODIFIED_ARGS+=("--arch" "amd64")
            skip_next=true
        else
            MODIFIED_ARGS+=("$arg")
        fi
    done
    "${SCRIPT_DIR}/build_deb.sh" "${MODIFIED_ARGS[@]}"
    
    # Build for arm64
    echo "Building for arm64..."
    MODIFIED_ARGS=()
    skip_next=false
    for arg in "$@"; do
        if [[ "$skip_next" == true ]]; then
            skip_next=false
            continue
        fi
        if [[ "$arg" == "--arch" ]]; then
            MODIFIED_ARGS+=("--arch" "arm64")
            skip_next=true
        else
            MODIFIED_ARGS+=("$arg")
        fi
    done
    "${SCRIPT_DIR}/build_deb.sh" "${MODIFIED_ARGS[@]}"
    
    # Deploy all architectures
    "${SCRIPT_DIR}/deploy.sh" "$@"
else
    # Build and deploy for single architecture
    "${SCRIPT_DIR}/build_deb.sh" "$@"
    "${SCRIPT_DIR}/deploy.sh" "$@"
fi
