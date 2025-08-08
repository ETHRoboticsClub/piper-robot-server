# Quick Server Setup Guide

This guide will get you from a fresh Ubuntu server to a running Tactile Teleop deployment in just a few commands.

## Prerequisites

- Fresh Ubuntu 20.04+ server
- Root access (via `sudo`)
- Domain name pointing to your server (for SSL)
- Ports 80 and 443 open in firewall

## Quick Setup (3 Commands)

### 1. Clone Repository

```bash
git clone <your-repo-url> tactile-teleop
cd tactile-teleop
```

### 2. Install Prerequisites

```bash
# Install Docker, make, certbot, and configure firewall
sudo ./bootstrap-server.sh yourdomain.com admin@yourdomain.com
```

### 3. Deploy Production

```bash
# Deploy with SSL certificates
make deploy-prod
```

That's it! Your system should be running at `https://yourdomain.com/`

## Alternative: Manual Steps

If the bootstrap script doesn't work, you can install manually:

```bash
# Install prerequisites
sudo apt update
sudo apt install -y make docker.io docker-compose git certbot python3-certbot-nginx

# Start Docker
sudo systemctl start docker
sudo systemctl enable docker
sudo usermod -aG docker $USER

# Configure firewall
sudo ufw allow 80/tcp
sudo ufw allow 443/tcp
sudo ufw enable

# Log out and back in for docker group changes
# Then deploy
make deploy-prod
```

## Troubleshooting

### Command 'make' not found

```bash
sudo apt install make
```

### Docker permission denied

```bash
sudo usermod -aG docker $USER
# Then log out and back in
```

### SSL certificate issues

```bash
# Set up SSL manually
make ssl-setup DOMAIN=yourdomain.com EMAIL=admin@yourdomain.com
make deploy-prod
```

### Check deployment status

```bash
make status
make logs
```

## What the Bootstrap Script Does

1. **Updates system packages**
2. **Installs essential tools**: make, curl, wget, git
3. **Installs Docker**: Latest version with compose plugin
4. **Installs Docker Compose**: Standalone version for compatibility
5. **Installs Certbot**: For SSL certificate management
6. **Configures firewall**: Opens ports 80, 443
7. **Sets up Docker group**: Adds user to docker group
8. **Optionally sets up SSL**: If domain and email provided

## Next Steps

After successful deployment:

- Access your VR interface at `https://yourdomain.com/`
- Check health at `https://yourdomain.com/health`
- View logs with `make logs`
- Update with `git pull && make deploy-prod`

For detailed configuration options, see [DEPLOYMENT.md](DEPLOYMENT.md).
