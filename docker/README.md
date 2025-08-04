# Tactile Teleop Docker Production Deployment

Docker containerization for the Tactile Teleop VR teleoperation system with nginx reverse proxy, FastAPI backend, and automatic SSL certificate management.

## 🚀 Production Deployment on Ubuntu

### Prerequisites

```bash
# Install Docker on Ubuntu server
sudo apt update && sudo apt install -y docker.io docker-compose git
sudo systemctl start docker && sudo systemctl enable docker
sudo usermod -aG docker $USER  # logout/login required
```

### Quick Deployment

```bash
# 1. Clone repository
git clone <your-repo-url> telegrip && cd telegrip

# 2. Configure domain (edit production.env)
cp production.env.example production.env
nano production.env  # Set DOMAIN_NAME=yourdomain.com

# 3. Deploy with automatic SSL
make certbot-prod DOMAINS="yourdomain.com www.yourdomain.com" EMAIL="admin@yourdomain.com"
make deploy-prod

# Access at: https://yourdomain.com/
```

### SSL Certificates

The system supports both Let's Encrypt (production) and self-signed (development) certificates:

```bash
# Test SSL setup (staging certificates)
make certbot-test DOMAINS="yourdomain.com" EMAIL="admin@yourdomain.com"

# Production SSL setup
make certbot-prod DOMAINS="yourdomain.com www.yourdomain.com" EMAIL="admin@yourdomain.com"

# Deploy with HTTPS
make deploy-prod
```

## 🏗️ Architecture

```
Internet → Nginx (HTTPS/HTTP) → FastAPI Container
                ↓
            Static Files + VR Interface
```

- **Nginx**: HTTPS termination, static files, reverse proxy
- **FastAPI**: API endpoints, WebRTC signaling, health checks
- **Docker Network**: Internal container communication
- **Certbot**: Automatic SSL certificate management

## 📋 Management Commands

### Deployment & SSL

```bash
# Production deployment
make deploy-prod                                    # Deploy with HTTPS
make deploy-dev                                     # Deploy development (HTTP)

# SSL certificate management
make certbot-test DOMAINS="..." EMAIL="..."        # Test SSL certificates
make certbot-prod DOMAINS="..." EMAIL="..."        # Production SSL certificates
make clean-certs                                    # Remove SSL certificates
```

### Service Management

```bash
# Docker service control
make stop                        # Stop all services
make logs                        # View service logs
make restart-nginx              # Restart nginx only

# Manual Docker commands
docker-compose ps               # Check container status
docker-compose logs -f          # Follow logs
docker-compose restart          # Restart all services
```

## 🌐 Access Points

| Service           | URL                             | Description                |
| ----------------- | ------------------------------- | -------------------------- |
| **Web Interface** | `https://yourdomain.com/`       | VR teleoperation interface |
| **Health Check**  | `https://yourdomain.com/health` | Service health status      |
| **HTTP Redirect** | `http://yourdomain.com/`        | Redirects to HTTPS         |

## 🔧 Configuration

### Environment Files

Create required configuration files:

**1. Domain Configuration** (`production.env`):

```bash
DOMAIN_NAME=yourdomain.com
NGINX_HTTP_PORT=80
NGINX_HTTPS_PORT=443
COMPOSE_PROJECT_NAME=tactile-teleop
LETSENCRYPT_EMAIL=admin@yourdomain.com
```

**2. API Secrets** (`.env`):

```bash
# LiveKit WebRTC Configuration
LIVEKIT_API_KEY="your-api-key"
LIVEKIT_API_SECRET="your-secret-key"
LIVEKIT_URL="ws://your-livekit-server:7880"
```

## 🔍 Monitoring & Verification

### Health Checks

```bash
# Check container status
docker-compose ps

# Test endpoints
curl -k https://yourdomain.com/health
curl -k https://yourdomain.com/

# Check logs
make logs
# Or: docker-compose logs -f
```

### Debugging

```bash
# Access containers
docker exec -it tactile-teleop-api bash
docker exec -it tactile-teleop-nginx sh

# Resource monitoring
docker stats
```

## 🚨 Troubleshooting

### Common Issues

```bash
# Services won't start
docker-compose ps
docker-compose logs

# Port conflicts
sudo lsof -i :80 -i :443

# nginx 502 errors
docker-compose restart

# SSL certificate warnings (normal for self-signed)
# Click "Advanced" → "Proceed" in browser

# Permission errors
sudo chown -R $USER:$USER .
```

## 🌍 Production Deployment

### Remote Server Setup

```bash
# 1. On remote Ubuntu server
sudo apt update && sudo apt install -y docker.io docker-compose git
sudo systemctl start docker && sudo systemctl enable docker
sudo usermod -aG docker $USER  # logout/login required

# 2. Clone and configure
git clone <your-repo> telegrip && cd telegrip
cp production.env.example production.env
nano production.env  # Set your domain

# 3. Deploy with SSL
make certbot-prod DOMAINS="yourdomain.com www.yourdomain.com" EMAIL="admin@yourdomain.com"
make deploy-prod
```

### Firewall Configuration

```bash
# Open ports for HTTPS/HTTP
sudo ufw allow 80/tcp
sudo ufw allow 443/tcp
sudo ufw reload
```

## 🔒 Security Features

- ✅ **HTTPS enforced** with automatic redirects
- ✅ **Let's Encrypt SSL** certificates
- ✅ **Security headers** (HSTS, XSS protection)
- ✅ **Container isolation** and non-root execution
- ✅ **Private Docker networks**

## 📊 Advantages

✅ **Simple Deployment**: One-command setup with `make deploy-prod`  
✅ **Automatic SSL**: Let's Encrypt integration  
✅ **Container Isolation**: No dependency conflicts  
✅ **Easy Updates**: `docker-compose restart`  
✅ **Built-in Monitoring**: Health checks and logging

---

**🚀 Access your VR teleoperation system at: `https://yourdomain.com/`**
