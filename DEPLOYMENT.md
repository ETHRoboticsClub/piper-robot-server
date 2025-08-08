# Tactile Teleop Deployment Guide

Production deployment of the Tactile Teleop VR teleoperation system with automatic SSL certificate management.

## Quick Start

### Development (Local Testing)

```bash
make deploy-dev
# Access at: http://localhost:8080/
```

### Production (With SSL)

```bash
# Set up SSL certificates and deploy
make ssl-setup DOMAIN=yourdomain.com EMAIL=admin@yourdomain.com
make deploy-prod

# Or deploy with automatic SSL setup
make deploy-prod-ssl
# Access at: https://yourdomain.com/
```

## Architecture

```
Internet → Nginx (HTTPS/HTTP) → FastAPI Container
                ↓
            Static Files + VR Interface
```

- **Nginx**: HTTPS termination, static files, reverse proxy
- **FastAPI**: API endpoints, WebRTC signaling, health checks
- **Docker**: Containerized deployment with automatic SSL integration

## Prerequisites

### System Requirements

- Ubuntu/Debian Linux system (tested on Ubuntu 20.04+)
- Docker and Docker Compose installed
- Domain name pointing to your server (for SSL certificates)
- Ports 80 and 443 accessible from the internet

### Installation

```bash
# Install Docker
sudo apt update && sudo apt install -y docker.io docker-compose git
sudo systemctl start docker && sudo systemctl enable docker
sudo usermod -aG docker $USER  # logout/login required

# Clone repository
git clone <repository-url> tactile-teleop && cd tactile-teleop
```

## Configuration

### Environment Files

**1. Domain Configuration** (`production.env`):

```bash
DOMAIN_NAME=yourdomain.com
NGINX_HTTP_PORT=80
NGINX_HTTPS_PORT=443
LETSENCRYPT_EMAIL=admin@yourdomain.com
```

**2. API Secrets** (`.env`):

```bash
# LiveKit WebRTC Configuration
LIVEKIT_API_KEY="your-api-key"
LIVEKIT_API_SECRET="your-secret-key"
LIVEKIT_URL="ws://your-livekit-server:7880"
```

## Deployment Methods

### Docker Deployment (Recommended)

The system automatically detects and uses Let's Encrypt certificates from `/etc/letsencrypt/` when available.

#### Development Environment

```bash
make deploy-dev
```

- HTTP only on port 8080
- Live code reload
- Direct FastAPI access on port 8000

#### Production Environment

```bash
# Deploy with SSL auto-detection
make deploy-prod

# Deploy with forced SSL setup
make deploy-prod-ssl
```

- HTTPS with Let's Encrypt certificates
- Automatic HTTP→HTTPS redirects
- Production optimizations

### Direct System Deployment

For deployment without Docker containers:

```bash
# Generate system configuration files
make setup-direct DOMAIN=yourdomain.com

# Follow the printed instructions to:
# 1. Install nginx configuration
# 2. Deploy systemd service
# 3. Set up SSL certificates
# 4. Start services
```

## SSL Certificate Management

### Automatic SSL Setup

```bash
# Interactive SSL certificate setup
make ssl-setup DOMAIN=yourdomain.com EMAIL=admin@yourdomain.com

# Test SSL setup (staging certificates)
make ssl-test DOMAIN=yourdomain.com EMAIL=admin@yourdomain.com
```

### Manual SSL Setup

```bash
# Install certbot
sudo apt update && sudo apt install certbot python3-certbot-nginx

# Obtain certificates
sudo certbot --nginx -d yourdomain.com -d www.yourdomain.com

# Deploy with SSL enabled
make deploy-prod
```

### Certificate Renewal

```bash
# Renew SSL certificates
make ssl-renew

# Certificates auto-renew via certbot cron job
# Docker containers automatically reload certificates every 6 hours
```

## Management Commands

### Deployment

```bash
make deploy-dev          # Development environment (HTTP)
make deploy-prod         # Production with SSL auto-detection
make deploy-prod-ssl     # Production with forced SSL setup
```

### SSL Management

```bash
make ssl-setup DOMAIN=... EMAIL=...    # Interactive SSL setup
make ssl-test DOMAIN=... EMAIL=...     # Test certificates (staging)
make ssl-renew                          # Renew certificates
```

### Service Management

```bash
make status              # Show deployment status and health
make logs                # View service logs
make stop                # Stop all services
make clean               # Clean up containers and resources
make restart-nginx       # Restart nginx service only
```

## Monitoring & Verification

### Health Checks

```bash
# Check deployment status
make status

# Test endpoints manually
curl -k https://yourdomain.com/health
curl -k https://yourdomain.com/

# Check container status
docker-compose ps
```

### Access Points

| Service           | URL                             | Description                |
| ----------------- | ------------------------------- | -------------------------- |
| **Web Interface** | `https://yourdomain.com/`       | VR teleoperation interface |
| **Health Check**  | `https://yourdomain.com/health` | Service health status      |
| **Development**   | `http://localhost:8080/`        | Development environment    |

## SSL Certificate Details

### Certificate Sources

1. **Let's Encrypt** (Recommended): Free, automatic renewal
2. **Self-signed** (Development): Generated automatically for local testing

### Certificate Flow

1. **System Certificates**: Let's Encrypt stores certificates in `/etc/letsencrypt/`
2. **Direct Mount**: Certificates mounted directly into nginx container
3. **Automatic Renewal**: Certbot renews certificates, nginx reloads automatically

### Security Features

- ✅ **HTTPS enforced** with automatic redirects
- ✅ **Let's Encrypt SSL** certificates with auto-renewal
- ✅ **Security headers** (HSTS, XSS protection, CSP)
- ✅ **TLS 1.2+** with secure cipher suites
- ✅ **Container isolation** and private networks

## Troubleshooting

### Common Issues

**Services won't start:**

```bash
make status
make logs
docker-compose ps
```

**Port conflicts:**

```bash
sudo lsof -i :80 -i :443
```

**SSL certificate warnings:**

```bash
# For self-signed certificates (development), click "Advanced" → "Proceed" in browser
# For production, ensure DNS points to your server and certificates are valid
```

**Certificate issues:**

```bash
# Check if certificates exist
ls -la /etc/letsencrypt/live/yourdomain.com/

# Validate certificate
openssl x509 -in /etc/letsencrypt/live/yourdomain.com/fullchain.pem -text -noout

# Test certificate connection
openssl s_client -servername yourdomain.com -connect yourdomain.com:443
```

**Container issues:**

```bash
# Restart all services
docker-compose restart

# Check nginx configuration
docker exec -it tactile-teleop-nginx nginx -t

# Access container for debugging
docker exec -it tactile-teleop-nginx sh
docker exec -it tactile-teleop-api bash
```

### DNS Configuration

Ensure your DNS has these A records:

- `yourdomain.com` → your server IP
- `www.yourdomain.com` → your server IP

### Firewall Configuration

```bash
# Open required ports
sudo ufw allow 80/tcp
sudo ufw allow 443/tcp
sudo ufw reload
```

## Migration & Updates

### From Self-Signed to Let's Encrypt

```bash
# System automatically detects and uses Let's Encrypt certificates
# No manual migration required
make ssl-setup DOMAIN=yourdomain.com EMAIL=admin@yourdomain.com
make deploy-prod
```

### Updating the Application

```bash
# Pull latest changes
git pull

# Rebuild and restart
make deploy-prod
```

## Environment Variables Reference

### SSL Configuration

- `DOMAIN_NAME`: Domain name for certificates (default: teleop.tactilerobotics.ai)
- `LETSENCRYPT_EMAIL`: Email for Let's Encrypt notifications
- `AUTO_SSL`: Automatically set up SSL without prompts (default: false)
- `SSL_ENABLED`: Enable SSL in nginx container (auto-detected)

### Nginx Configuration

- `NGINX_HTTP_PORT`: HTTP port (default: 8080 dev, 80 prod)
- `NGINX_HTTPS_PORT`: HTTPS port (default: 8443 dev, 443 prod)

### Application Configuration

- `LIVEKIT_API_KEY`: LiveKit API key for WebRTC
- `LIVEKIT_API_SECRET`: LiveKit API secret
- `LIVEKIT_URL`: LiveKit server URL

## CI/CD Integration

For automated deployments:

```bash
# Set environment variables
export DOMAIN_NAME=yourdomain.com
export LETSENCRYPT_EMAIL=admin@yourdomain.com
export AUTO_SSL=true

# Deploy without user interaction
make deploy-prod-ssl
```

---

**🚀 Access your VR teleoperation system at: `https://yourdomain.com/`**
