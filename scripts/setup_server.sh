#!/bin/bash
# Server setup script for avocado.acfr.usyd.edu.au
# Run this once on the server to set up the repository hosting infrastructure
# Usage: ssh user@avocado.acfr.usyd.edu.au 'bash -s' < scripts/setup_server.sh

set -e

# Configuration
REMOTE_BASE_PATH="/data/www/EHM/datasets/ubuntu-repo"
DISTROS=("jazzy")

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Setting up ROS2 Package Repository${NC}"
echo -e "${BLUE}========================================${NC}"

# Install required packages
echo -e "${GREEN}Installing required packages...${NC}"
sudo apt-get update
sudo apt-get install -y dpkg-dev apache2 || sudo apt-get install -y dpkg-dev nginx

# Create directory structure
echo -e "${GREEN}Creating directory structure...${NC}"
sudo mkdir -p ${REMOTE_BASE_PATH}
for distro in "${DISTROS[@]}"; do
    sudo mkdir -p ${REMOTE_BASE_PATH}/${distro}
done

# Set permissions
echo -e "${GREEN}Setting permissions...${NC}"
sudo chown -R ${USER}: ${REMOTE_BASE_PATH}
sudo chmod -R 755 ${REMOTE_BASE_PATH}

# Create main index.html
echo -e "${GREEN}Creating repository index page...${NC}"
cat > ${REMOTE_BASE_PATH}/index.html << 'HTML_EOF'
<!DOCTYPE html>
<html>
<head>
    <title>ACFR ROS2 Packages Repository</title>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 40px;
            background-color: #f9f9f9;
        }
        .container {
            max-width: 900px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 { color: #2c3e50; }
        h2 { color: #34495e; margin-top: 30px; }
        h3 { color: #7f8c8d; }
        .distro { 
            margin: 20px 0; 
            padding: 15px; 
            background-color: #ecf0f1; 
            border-radius: 5px;
            border-left: 4px solid #3498db;
        }
        code { 
            background-color: #e8e8e8; 
            padding: 2px 6px; 
            border-radius: 3px;
            font-family: 'Courier New', monospace;
        }
        pre { 
            background-color: #272822; 
            color: #f8f8f2; 
            padding: 15px; 
            border-radius: 5px; 
            overflow-x: auto;
            font-family: 'Courier New', monospace;
        }
        .warning {
            background-color: #fff3cd;
            border-left: 4px solid #ffc107;
            padding: 15px;
            margin: 20px 0;
            border-radius: 5px;
        }
        a { color: #3498db; text-decoration: none; }
        a:hover { text-decoration: underline; }
        .footer {
            margin-top: 40px;
            padding-top: 20px;
            border-top: 1px solid #ddd;
            color: #7f8c8d;
            font-size: 0.9em;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 ACFR ROS2 Packages Repository</h1>
        <p>Welcome to the Australian Centre for Field Robotics (ACFR) ROS2 packages repository hosted at avocado.acfr.usyd.edu.au</p>
        
        <h2>📦 Available Distributions</h2>
        <div class="distro">
            <h3>ROS2 Humble (LTS)</h3>
            <p><strong>Ubuntu:</strong> 22.04 (Jammy)</p>
            <p><a href="humble/">📁 Browse packages</a></p>
        </div>
        <div class="distro">
            <h3>ROS2 Iron</h3>
            <p><strong>Ubuntu:</strong> 22.04 (Jammy)</p>
            <p><a href="iron/">📁 Browse packages</a></p>
        </div>
        <div class="distro">
            <h3>ROS2 Jazzy</h3>
            <p><strong>Ubuntu:</strong> 24.04 (Noble)</p>
            <p><a href="jazzy/">📁 Browse packages</a></p>
        </div>
        
        <h2>📥 Installation Instructions</h2>
        <p>To use this repository, add it to your APT sources:</p>
        <pre>
# Add the repository
echo "deb [trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo/jazzy ./" | sudo tee /etc/apt/sources.list.d/acfr-ros2.list

# Update package list
sudo apt update

# Install packages
sudo apt install ros-jazzy-ros2-data-logger
        </pre>
        
        <div class="warning">
            <strong>⚠️ Note:</strong> This repository uses <code>[trusted=yes]</code> to skip GPG signature verification. 
            Only use this on trusted networks within ACFR/USYD.
        </div>
        
        <h2>📚 Available Packages</h2>
        <ul>
            <li><strong>ros2_data_logger</strong> - Flexible data logging package for ROS2 robotic platforms</li>
        </ul>
        
        <h2>🔧 For Developers</h2>
        <p>To build and deploy packages to this repository:</p>
        <pre>
# Clone the repository
git clone https://github.com/acfr/ros2_data_logger.git
cd ros2_data_logger

# Build Debian package
./scripts/build_deb.sh --ros-distro jazzy

# Deploy to this server
./scripts/deploy.sh --user your_username --ros-distro jazzy
        </pre>
        
        <h2>🤝 Contributing</h2>
        <p>Visit our <a href="https://github.com/acfr/ros2_data_logger">GitHub repository</a> to contribute or report issues.</p>
        
        <div class="footer">
            <p><strong>Maintained by:</strong> Australian Centre for Field Robotics, University of Sydney</p>
            <p><strong>Last updated:</strong> <span id="timestamp"></span></p>
            <script>
                document.getElementById('timestamp').textContent = new Date().toLocaleString();
            </script>
        </div>
    </div>
</body>
</html>
HTML_EOF

# Configure web server (if not already configured)
if command -v apache2 &> /dev/null; then
    echo -e "${GREEN}Configuring Apache...${NC}"
    # Enable directory listing for package browsing
    sudo tee /etc/apache2/conf-available/ros2-packages.conf > /dev/null << 'APACHE_EOF'
<Directory /data/www/EHM/datasets/ubuntu-repo>
    Options +Indexes +FollowSymLinks
    AllowOverride None
    Require all granted
    IndexOptions FancyIndexing NameWidth=* DescriptionWidth=*
</Directory>
APACHE_EOF
    sudo a2enconf ros2-packages
    sudo systemctl reload apache2
elif command -v nginx &> /dev/null; then
    echo -e "${GREEN}Configuring Nginx...${NC}"
    sudo tee /etc/nginx/sites-available/ros2-packages > /dev/null << 'NGINX_EOF'
server {
    listen 80;
    listen 443 ssl;
    server_name data.acfr.usyd.edu.au;
    
    location /ubuntu-repo {
        alias /data/www/EHM/datasets/ubuntu-repo;
        autoindex on;
        autoindex_exact_size off;
        autoindex_localtime on;
    }
}
NGINX_EOF
    sudo ln -sf /etc/nginx/sites-available/ros2-packages /etc/nginx/sites-enabled/
    sudo systemctl reload nginx
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Server setup complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}Repository URL: https://data.acfr.usyd.edu.au/ubuntu-repo/${NC}"
echo -e "${YELLOW}You can now deploy packages using:${NC}"
echo -e "${YELLOW}  ./scripts/deploy.sh${NC}"
echo -e "${GREEN}========================================${NC}"
