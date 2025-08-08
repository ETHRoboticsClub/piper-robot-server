#!/bin/bash

# Tactile Teleop Deployment Setup Script
# This script configures the nginx and systemd service files for deployment on any Ubuntu server

set -e

echo "=== Tactile Teleop Deployment Setup ==="
echo

# Detect current configuration
CURRENT_USER=${USER:-$(whoami)}
PROJECT_ROOT=$(pwd)
SERVER_NAME=${1:-localhost}

# Find conda environment path
if [ -z "$CONDA_PREFIX" ]; then
    # Try to find conda installation
    if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/miniconda3/etc/profile.d/conda.sh"
        if conda activate tactile-teleop 2>/dev/null; then
            CONDA_ENV_PATH="$CONDA_PREFIX"
            conda deactivate
        else
            echo "Error: tactile-teleop conda environment not found"
            echo "Please create the environment first with: conda env create -f environment.yml"
            exit 1
        fi
    elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/anaconda3/etc/profile.d/conda.sh"
        if conda activate tactile-teleop 2>/dev/null; then
            CONDA_ENV_PATH="$CONDA_PREFIX"
            conda deactivate
        else
            echo "Error: tactile-teleop conda environment not found"
            echo "Please create the environment first with: conda env create -f environment.yml"
            exit 1
        fi
    else
        echo "Error: Conda installation not found"
        echo "Please install conda and create the tactile-teleop environment"
        exit 1
    fi
else
    CONDA_ENV_PATH="$CONDA_PREFIX"
fi

echo "Detected configuration:"
echo "  User: $CURRENT_USER"
echo "  Project root: $PROJECT_ROOT"
echo "  Server name: $SERVER_NAME"
echo "  Conda environment: $CONDA_ENV_PATH"
echo

# Create nginx directory
DEPLOY_DIR="$PROJECT_ROOT/src/tactile_teleop/web_server/nginx"
mkdir -p "$DEPLOY_DIR"

# Generate nginx configuration files and systemd service
echo "Generating nginx configuration..."
cd "$PROJECT_ROOT/src/tactile_teleop/web_server"
DOMAIN_NAME="$SERVER_NAME" ./nginx/configure-nginx.sh "$PROJECT_ROOT/development.env" direct

echo "Generating systemd service..."
sed -e "s|{{USER}}|$CURRENT_USER|g" \
    -e "s|{{PROJECT_ROOT}}|$PROJECT_ROOT|g" \
    -e "s|{{CONDA_ENV_PATH}}|$CONDA_ENV_PATH|g" \
    "$DEPLOY_DIR/tactile-teleop.service.template" > "$DEPLOY_DIR/tactile-teleop.service"

echo "Configuration files generated in: $DEPLOY_DIR/"

# Print deployment instructions
echo
echo "=== Deployment Instructions ==="
echo
echo "1. Install nginx:"
echo "   sudo apt update && sudo apt install nginx"
echo
echo "2. Deploy tactile-teleop nginx configuration:"
echo "   sudo cp $DEPLOY_DIR/nginx-tactile-teleop.conf /etc/nginx/sites-available/tactile-teleop"
echo "   sudo ln -sf /etc/nginx/sites-available/tactile-teleop /etc/nginx/sites-enabled/"
echo "   sudo nginx -t"
echo
echo "3. Allow HTTPS traffic:"
echo "   sudo ufw allow 'Nginx Full'"
echo "   sudo ufw delete allow 'Nginx HTTP'"

echo "4. Obtain an SSL Certificate:"
echo "   sudo certbot --nginx -d $SERVER_NAME -d www.$SERVER_NAME"
echo
echo "5. Deploy systemd service:"
echo "   sudo cp $DEPLOY_DIR/tactile-teleop.service /etc/systemd/system/"
echo "   sudo systemctl daemon-reload"
echo "   sudo systemctl enable tactile-teleop"
echo
echo "6. Start services:"
echo "   sudo systemctl start tactile-teleop"
echo "   sudo systemctl restart nginx"
echo
echo "7. Verify deployment:"
echo "   sudo systemctl status tactile-teleop"
echo "   sudo systemctl status nginx"
echo "   curl -k https://$SERVER_NAME/health"
echo
echo "=== Custom Server Name ==="
echo "To use a custom domain/IP, run:"
echo "  ./deploy-setup.sh your-domain.com"
echo
echo "Done! Your server will be available at: https://$SERVER_NAME"