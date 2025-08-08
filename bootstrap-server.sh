#!/bin/bash

# Tactile Teleop Server Bootstrap Script
# Installs all prerequisites for running the deployment system
# Usage: ./bootstrap-server.sh [domain] [email]

set -e

echo "=== Tactile Teleop Server Bootstrap ==="
echo "Installing prerequisites for deployment..."
echo

# Parse arguments
DOMAIN=${1:-}
EMAIL=${2:-}

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    print_error "This script must be run as root (use sudo)"
    echo "Usage: sudo ./bootstrap-server.sh [domain] [email]"
    exit 1
fi

print_status "Updating system packages..."
apt-get update

print_status "Installing essential build tools..."
apt-get install -y \
    make \
    curl \
    wget \
    git \
    software-properties-common \
    ca-certificates \
    gnupg \
    lsb-release

print_success "Essential tools installed"

# Install Docker if not already installed
if ! command -v docker &> /dev/null; then
    print_status "Installing Docker..."
    
    # Add Docker's official GPG key
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    
    # Add Docker repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null
    
    # Install Docker
    apt-get update
    apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    
    # Start and enable Docker
    systemctl start docker
    systemctl enable docker
    
    print_success "Docker installed and started"
else
    print_success "Docker already installed"
fi

# Install Docker Compose (standalone) if not available
if ! command -v docker-compose &> /dev/null; then
    print_status "Installing Docker Compose..."
    
    # Get latest version
    DOCKER_COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | grep 'tag_name' | cut -d\" -f4)
    
    # Download and install
    curl -L "https://github.com/docker/compose/releases/download/${DOCKER_COMPOSE_VERSION}/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    chmod +x /usr/local/bin/docker-compose
    
    print_success "Docker Compose installed"
else
    print_success "Docker Compose already installed"
fi

# Install certbot for SSL certificates
if ! command -v certbot &> /dev/null; then
    print_status "Installing Certbot for SSL certificates..."
    apt-get install -y certbot python3-certbot-nginx
    print_success "Certbot installed"
else
    print_success "Certbot already installed"
fi

# Configure firewall
print_status "Configuring firewall..."
if command -v ufw &> /dev/null; then
    ufw allow 22/tcp      # SSH
    ufw allow 80/tcp      # HTTP
    ufw allow 443/tcp     # HTTPS
    ufw --force enable
    print_success "Firewall configured (ports 22, 80, 443 open)"
else
    print_warning "UFW not available, skipping firewall configuration"
fi

# Create docker group and add current user (if not root session)
if [ -n "${SUDO_USER:-}" ]; then
    usermod -aG docker $SUDO_USER
    print_success "Added $SUDO_USER to docker group"
fi

print_status "Verifying installations..."

# Check versions
echo
echo "=== Installation Summary ==="
echo "Make: $(make --version | head -n1)"
echo "Docker: $(docker --version)"
echo "Docker Compose: $(docker-compose --version)"
echo "Certbot: $(certbot --version)"
echo

print_success "All prerequisites installed successfully!"
echo

# Provide next steps
echo "=== Next Steps ==="
echo
echo "1. If you're not already in the project directory:"
echo "   cd /path/to/tactile-teleop"
echo
echo "2. For development deployment (HTTP only):"
echo "   make deploy-dev"
echo
echo "3. For production deployment (with SSL auto-setup):"
if [ -n "$DOMAIN" ] && [ -n "$EMAIL" ]; then
    echo "   make deploy-prod-ssl"
    echo "   # Your domain: $DOMAIN"
    echo "   # Your email: $EMAIL"
else
    echo "   make ssl-setup DOMAIN=yourdomain.com EMAIL=admin@yourdomain.com"
    echo "   make deploy-prod"
    echo
    echo "   Or run this script with domain and email:"
    echo "   sudo ./bootstrap-server.sh yourdomain.com admin@yourdomain.com"
fi
echo
echo "4. Check deployment status:"
echo "   make status"
echo
echo "5. View logs:"
echo "   make logs"
echo

# If domain and email provided, offer to set up SSL
if [ -n "$DOMAIN" ] && [ -n "$EMAIL" ]; then
    echo "=== SSL Setup ==="
    read -p "Would you like to set up SSL certificates for $DOMAIN now? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_status "Setting up SSL certificates..."
        if ./src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh "$DOMAIN" "$EMAIL"; then
            print_success "SSL certificates configured for $DOMAIN"
            echo
            echo "Now you can deploy with:"
            echo "make deploy-prod"
        else
            print_warning "SSL setup failed. You can try again later with:"
            echo "make ssl-setup DOMAIN=$DOMAIN EMAIL=$EMAIL"
        fi
    fi
fi

# Final reminder about docker group
if [ -n "${SUDO_USER:-}" ]; then
    echo
    print_warning "Note: $SUDO_USER was added to the docker group."
    print_warning "Please log out and log back in for the changes to take effect,"
    print_warning "or run 'newgrp docker' to apply the group membership immediately."
fi

print_success "Bootstrap complete! You can now use 'make deploy-prod' or other make commands."