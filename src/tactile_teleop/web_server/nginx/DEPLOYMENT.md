# Tactile Teleop Production Deployment Guide

Production deployment of the Tactile Teleop system on Ubuntu servers with two deployment methods: Docker containers or direct systemd services.

## Architecture Overview

```
Internet → Nginx (HTTPS/HTTP) → FastAPI Server
                ↓
            Static Files + VR Interface
```

- **Nginx**: HTTPS termination, static file serving, reverse proxy
- **FastAPI**: API endpoints, health checks, WebRTC signaling
- **LiveKit**: Real-time video/audio streaming for VR teleoperation

## Prerequisites

- Ubuntu/Debian Linux system (tested on Ubuntu 20.04+)
- Sudo privileges for installing services
- Domain name (for production SSL certificates)

**For Docker Deployment:**

- Docker and Docker Compose installed

**For Direct Deployment:**

- Conda/Miniconda installed
- Conda environment `tactile-teleop` configured with all dependencies

## Deployment Methods

Choose between two production deployment methods:

### Method 1: Docker Deployment (Recommended)

**Pros**: Easy setup, consistent environment, automatic SSL
**Cons**: Requires Docker knowledge

```bash
# 1. Install Docker
sudo apt update && sudo apt install -y docker.io docker-compose git
sudo systemctl start docker && sudo systemctl enable docker
sudo usermod -aG docker $USER  # logout/login required

# 2. Clone and setup
git clone <repository-url> telegrip && cd telegrip

# 3. Configure environment
cp production.env.example production.env
# Edit production.env with your domain name

# 4. Deploy with automatic SSL
make certbot-prod DOMAINS="yourdomain.com www.yourdomain.com" EMAIL="admin@yourdomain.com"
make deploy-prod
```

### Method 2: Direct Ubuntu Deployment

**Pros**: Traditional systemd service, direct system integration  
**Cons**: Manual dependency management

```bash
# 1. Install system dependencies
sudo apt update && sudo apt install -y nginx git curl

# 2. Install Miniconda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b
source ~/miniconda3/etc/profile.d/conda.sh

# 3. Clone and setup project
git clone <repository-url> telegrip && cd telegrip
# Create 'tactile-teleop' conda environment with dependencies

# 4. Run automated deployment setup
./deploy-setup.sh yourdomain.com

# 5. Follow printed instructions to install nginx config and systemd service
```

## SSL Certificate Setup

### Automatic SSL (Docker - Recommended)

Docker deployment includes automatic Let's Encrypt SSL:

```bash
# Test certificate generation (staging)
make certbot-test DOMAINS="yourdomain.com www.yourdomain.com" EMAIL="admin@yourdomain.com"

# Generate production certificates
make certbot-prod DOMAINS="yourdomain.com www.yourdomain.com" EMAIL="admin@yourdomain.com"

# Deploy with HTTPS
make deploy-prod
```

### Manual SSL (Direct Deployment)

For direct Ubuntu deployment with Let's Encrypt:

```bash
# Install certbot
sudo apt install certbot

# Generate certificates
sudo bash src/tactile_teleop/web_server/register_ssl.sh \
  --domains "yourdomain.com www.yourdomain.com" \
  --email "admin@yourdomain.com" \
  --data-path "/etc/letsencrypt" \
  --staging 0

# Regenerate nginx config with SSL enabled
DOMAIN_NAME=yourdomain.com USE_SSL=true ./src/tactile_teleop/web_server/configure-nginx.sh production.env

# Install and restart services
sudo cp src/tactile_teleop/web_server/nginx/nginx-tactile-teleop.conf /etc/nginx/sites-available/tactile-teleop
sudo ln -sf /etc/nginx/sites-available/tactile-teleop /etc/nginx/sites-enabled/
sudo nginx -t && sudo systemctl reload nginx
```

## Verification

### Docker Deployment

```bash
# Check container status
docker-compose ps

# Test endpoints
curl -k https://yourdomain.com/health
curl -k https://yourdomain.com/

# View logs
docker-compose logs -f
```

### Direct Deployment

```bash
# Check services
sudo systemctl status tactile-teleop nginx

# Test endpoints
curl -k https://yourdomain.com/health
curl -k https://yourdomain.com/

# View logs
sudo journalctl -u tactile-teleop -f
sudo tail -f /var/log/nginx/error.log
```

## Configuration

The system uses **unified nginx templates** that automatically generate appropriate configurations:

- **Development**: HTTP-only with static file serving
- **Production**: HTTPS with Let's Encrypt, HTTP→HTTPS redirects
- **Templates**: `nginx.conf.dev.template` and `nginx.conf.prod.template`
- **Output**: `nginx-tactile-teleop.conf` (generated, don't edit directly)

### Environment Files

**Docker**: Uses `development.env` or `production.env`
**Direct**: Uses environment variables or `.env` files

Key settings:

- `DOMAIN_NAME`: Your domain (e.g., `teleop.example.com`)
- `USE_SSL`: Enable HTTPS mode (`true`/`false`)
- `LIVEKIT_API_KEY`, `LIVEKIT_API_SECRET`, `LIVEKIT_URL`: WebRTC configuration

## Troubleshooting

### Docker Issues

```bash
# Services won't start
docker-compose ps
docker-compose logs

# Port conflicts
sudo lsof -i :8080 -i :8443

# SSL certificate warnings (normal for self-signed)
# Click "Advanced" → "Proceed" in browser
```

### Direct Deployment Issues

```bash
# Service won't start
sudo systemctl status tactile-teleop
sudo journalctl -u tactile-teleop --no-pager

# Nginx 502 errors
ls -la /tmp/tactile-teleop.sock
sudo systemctl restart tactile-teleop

# SSL issues
sudo certbot renew --dry-run
```

### Log Locations

**Docker**: `docker-compose logs -f`
**Direct**: `sudo journalctl -u tactile-teleop -f` and `/var/log/nginx/error.log`

## Management Commands

### Docker

```bash
# Start/stop services
make deploy-prod        # Start production
make stop              # Stop all services
make logs              # View logs
make restart-nginx     # Restart nginx only

# SSL certificate management
make certbot-test DOMAINS="..." EMAIL="..."     # Test certificates
make certbot-prod DOMAINS="..." EMAIL="..."     # Production certificates
make clean-certs                                # Remove certificates
```

### Direct Deployment

```bash
# Service management
sudo systemctl start|stop|restart tactile-teleop
sudo systemctl reload nginx

# Update after code changes
sudo systemctl restart tactile-teleop

# Certificate renewal (Let's Encrypt auto-renews)
sudo certbot renew
```

---

**🚀 Access your VR teleoperation system at: `https://yourdomain.com/`**
