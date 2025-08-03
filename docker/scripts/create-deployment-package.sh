#!/bin/bash

# Create deployment package for remote VM
# Usage: ./create-deployment-package.sh [VM_IP] [VM_USER]

set -e

VM_IP=${1:-"your-vm-ip"}
VM_USER=${2:-"user"}

echo "=== Creating Tactile Teleop Deployment Package ==="
echo

# Create deployment package
echo "📦 Creating deployment package..."
tar -czf tactile-teleop-deployment.tar.gz \
    src/ \
    docker-compose.yml \
    docker-compose.override.yml \
    docker/ \
    requirements.txt \
    pyproject.toml \
    .dockerignore

echo "✅ Package created: tactile-teleop-deployment.tar.gz"
echo

if [ "$VM_IP" != "your-vm-ip" ]; then
    echo "🚀 Transferring to remote VM: $VM_USER@$VM_IP"
    scp tactile-teleop-deployment.tar.gz $VM_USER@$VM_IP:/home/$VM_USER/
    echo "✅ Files transferred successfully!"
    echo
    echo "Next steps on the remote VM:"
    echo "  ssh $VM_USER@$VM_IP"
    echo "  tar -xzf tactile-teleop-deployment.tar.gz"
    echo "  cd tactile-teleop-deployment/"
    echo "  chmod +x docker/scripts/docker-build.sh"
    echo "  ./docker/scripts/docker-build.sh prod"
    echo
    echo "Access your deployment at: https://$VM_IP:8443/"
else
    echo "📋 To transfer to your VM:"
    echo "  scp tactile-teleop-deployment.tar.gz user@your-vm-ip:/home/user/"
    echo
    echo "📋 Then on the remote VM:"
    echo "  tar -xzf tactile-teleop-deployment.tar.gz"
    echo "  ./docker/scripts/docker-build.sh prod"
fi

echo
echo "📚 See docker/README.md for detailed instructions"