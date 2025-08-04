# Tactile Teleop Docker Deployment

Complete Docker containerization for the Tactile Teleop VR teleoperation system with nginx reverse proxy and FastAPI backend.

## 📁 Directory Structure

```
docker/
├── README.md                    # This file - comprehensive Docker guide
├── Dockerfile                   # FastAPI application container
├── nginx.dockerfile            # Nginx reverse proxy container
├── docker-entrypoint.py        # Custom FastAPI entrypoint for containers
├── config/
│   └── nginx.conf              # Nginx configuration for containers
└── scripts/
    ├── docker-build.sh         # Automated build and deployment script
    └── create-deployment-package.sh  # Remote deployment packager
```

## 🚀 Quick Start

### Local Development

```bash
# Build and start all services
./docker/scripts/docker-build.sh dev

# Or manually
docker-compose up -d

# Access the interface
open https://localhost:8443/
```

### Production Deployment

```bash
# Build production setup
./docker/scripts/docker-build.sh prod

# Access the interface
open https://localhost:8443/
```

## 🏗️ Architecture Overview

```
Internet → Nginx Container → Docker Network → FastAPI Container
         (Port 8443/8080)     (Internal)      (Port 8000)
              ↓
          Static Files
```

- **Nginx Container**: HTTPS termination, static file serving, reverse proxy
- **FastAPI Container**: API endpoints, health checks, behind-proxy mode
- **Docker Network**: Internal container communication
- **Docker Volumes**: SSL certificate persistence

## 📋 Available Commands

### Build Scripts

```bash
# Development mode (live reload + direct FastAPI access)
./docker/scripts/docker-build.sh dev

# Production mode (optimized)
./docker/scripts/docker-build.sh prod

# Force rebuild of images
./docker/scripts/docker-build.sh prod rebuild
```

### Service Management

```bash
# Start services
docker-compose up -d

# Stop services
docker-compose down

# Restart services
docker-compose restart

# View logs
docker-compose logs -f

# Check status
docker-compose ps
```

### Remote Deployment

```bash
# Create deployment package for remote VM
./docker/scripts/create-deployment-package.sh

# With specific VM details
./docker/scripts/create-deployment-package.sh your-vm-ip your-username
```

## 🌐 Access Points

| Service            | URL                             | Description                     |
| ------------------ | ------------------------------- | ------------------------------- |
| **Web Interface**  | `https://localhost:8443/`       | Main VR teleoperation interface |
| **Health Check**   | `https://localhost:8443/health` | Service health status           |
| **HTTP Redirect**  | `http://localhost:8080/`        | Redirects to HTTPS              |
| **Direct FastAPI** | `http://localhost:8000/health`  | Development mode only           |

## 🔧 Configuration

### Environment Variables

Create a `.env` file in the project root:

```bash
# .env
COMPOSE_PROJECT_NAME=tactile-teleop
NGINX_HTTP_PORT=8080
NGINX_HTTPS_PORT=8443
FASTAPI_LOG_LEVEL=info
```

### Custom Ports

Edit `docker-compose.yml`:

```yaml
ports:
  - '9080:80' # HTTP on port 9080
  - '9443:443' # HTTPS on port 9443
```

### Custom SSL Certificates

```bash
# Create SSL directory
mkdir -p ssl/

# Add your certificates
cp your-cert.pem ssl/cert.pem
cp your-key.pem ssl/key.pem

# Uncomment SSL volume mounts in docker-compose.yml
# Restart services
docker-compose down && docker-compose up -d
```

## 🔍 Monitoring & Debugging

### Health Checks

```bash
# Check container health
docker-compose ps

# Test endpoints
curl -k https://localhost:8443/health
curl -k https://localhost:8443/

# Check logs
docker-compose logs -f fastapi
docker-compose logs -f nginx
```

### Container Access

```bash
# Access FastAPI container
docker exec -it tactile-teleop-api bash

# Access nginx container
docker exec -it tactile-teleop-nginx sh

# Check network connectivity
docker network inspect telegrip_tactile-network
```

### Performance Monitoring

```bash
# Container resource usage
docker stats

# Container processes
docker exec tactile-teleop-api ps aux
```

## 🚨 Troubleshooting

### Common Issues

**Services won't start:**

```bash
# Check Docker daemon
sudo systemctl status docker

# Check logs
docker-compose logs

# Rebuild images
docker-compose build --no-cache
```

**Port conflicts:**

```bash
# Find processes using ports
sudo lsof -i :8080
sudo lsof -i :8443

# Change ports in docker-compose.yml
```

**nginx 502 Bad Gateway:**

```bash
# Check FastAPI health
docker-compose ps
docker exec tactile-teleop-nginx nslookup fastapi
```

**SSL Certificate warnings:**

```bash
# This is normal for self-signed certificates
# Click "Advanced" → "Proceed to localhost (unsafe)" in browser
```

**Permission errors:**

```bash
# Fix ownership
sudo chown -R $USER:$USER .

# Fix SSL permissions
chmod 644 ssl/cert.pem 2>/dev/null || true
chmod 600 ssl/key.pem 2>/dev/null || true
```

## 🌍 Remote VM Deployment

### Prerequisites on Remote VM

```bash
# Install Docker and Docker Compose
sudo apt update
sudo apt install -y docker.io docker-compose git

# Start and enable Docker
sudo systemctl start docker
sudo systemctl enable docker

# Add your user to docker group (then logout/login)
sudo usermod -aG docker $USER
```

### Deployment Steps

#### 1. Transfer Files to Remote VM

From your local machine:

```bash
# Create deployment package using the script
./docker/scripts/create-deployment-package.sh your-vm-ip your-username

# Or manually transfer files
tar -czf tactile-teleop-docker.tar.gz \
    src/ \
    docker-compose.yml \
    docker-compose.override.yml \
    docker/ \
    requirements.txt \
    pyproject.toml \
    .dockerignore

# Copy to remote VM (replace with your VM details)
scp tactile-teleop-docker.tar.gz user@your-vm-ip:/home/user/

# SSH to remote VM
ssh user@your-vm-ip
```

#### 2. Setup on Remote VM

```bash
# Extract files
tar -xzf tactile-teleop-docker.tar.gz
cd tactile-teleop-docker/

# Make build script executable
chmod +x docker/scripts/docker-build.sh

# Deploy production setup
./docker/scripts/docker-build.sh prod
```

#### 3. Configure Firewall (if needed)

```bash
# Open required ports
sudo ufw allow 8080/tcp  # HTTP
sudo ufw allow 8443/tcp  # HTTPS
sudo ufw reload
```

#### 4. Access Your Deployment

- **Web Interface**: `https://your-vm-ip:8443/`
- **Health Check**: `https://your-vm-ip:8443/health`

### Custom Domain Setup (Optional)

If you have a domain name pointing to your VM:

```bash
# Stop current services
docker-compose down

# Edit nginx configuration for your domain
nano docker/config/nginx.conf
# Change: server_name _;
# To:     server_name your-domain.com;

# Restart services
docker-compose up -d
```

### Production SSL with Let's Encrypt

```bash
# Install certbot for Let's Encrypt
sudo apt install certbot

# Get certificate (replace your-domain.com)
sudo certbot certonly --standalone -d your-domain.com

# Copy certificates to project
mkdir -p ssl/
sudo cp /etc/letsencrypt/live/your-domain.com/fullchain.pem ssl/cert.pem
sudo cp /etc/letsencrypt/live/your-domain.com/privkey.pem ssl/key.pem
sudo chown $USER:$USER ssl/*

# Update nginx config for your domain
nano docker/config/nginx.conf
# Change: server_name _;
# To:     server_name your-domain.com;

# Uncomment SSL volume mounts in docker-compose.yml
# Restart services
docker-compose down && docker-compose up -d
```

### Remote Management Commands

```bash
# Check service status
docker-compose ps

# View logs
docker-compose logs -f

# Restart services
docker-compose restart

# Stop services
docker-compose down

# Update after code changes
git pull  # if using git
docker-compose build
docker-compose up -d
```

## 🔒 Security Considerations

- ✅ **HTTPS enforced** with HTTP→HTTPS redirects
- ✅ **Security headers** configured (HSTS, XSS protection, etc.)
- ✅ **Non-root containers** for improved security
- ✅ **Private networks** for inter-container communication
- ✅ **Self-signed certificates** for development (replace with CA certs for production)

## ⚡ Performance Optimization

### Resource Limits

Add to `docker-compose.yml`:

```yaml
services:
  fastapi:
    deploy:
      resources:
        limits:
          memory: 512M
          cpus: '0.5'
  nginx:
    deploy:
      resources:
        limits:
          memory: 128M
          cpus: '0.2'
```

### Caching

Static assets are cached for 1 year via nginx configuration.

### Scaling

For high availability:

```yaml
services:
  fastapi:
    scale: 3 # Run multiple FastAPI instances
```

## 🔄 Development vs Production

| Feature            | Development Mode             | Production Mode                |
| ------------------ | ---------------------------- | ------------------------------ |
| **Code Changes**   | Live reload via volume mount | Rebuilt into image             |
| **FastAPI Access** | Direct port 8000 exposed     | Only via nginx proxy           |
| **Logging**        | Debug level                  | Info level                     |
| **SSL**            | Self-signed (warnings OK)    | Should use proper certificates |
| **Static Files**   | Live mounted from host       | Copied into nginx image        |

## 📊 Advantages of Docker Deployment

✅ **Portability**: Runs consistently on any Docker host  
✅ **Isolation**: No system dependency conflicts  
✅ **Scalability**: Easy horizontal scaling  
✅ **Consistency**: Same environment dev→staging→prod  
✅ **Rollback**: Easy version management with image tags  
✅ **Monitoring**: Built-in health checks and logging  
✅ **Security**: Container isolation and non-root execution

## 🆚 Docker vs Traditional Deployment

| Aspect               | Docker                       | Traditional (systemd/nginx)     |
| -------------------- | ---------------------------- | ------------------------------- |
| **Setup Complexity** | Simple (`docker-compose up`) | Complex (multiple manual steps) |
| **Dependencies**     | Self-contained               | System-wide conflicts possible  |
| **Portability**      | Run anywhere                 | OS/distro specific              |
| **Updates**          | Rebuild image                | Manual service management       |
| **Scaling**          | Built-in orchestration       | Manual load balancer setup      |
| **Isolation**        | Complete container isolation | Shared system resources         |

---

## 🎯 Next Steps

1. **Local Development**: `./docker/scripts/docker-build.sh dev`
2. **Production Testing**: `./docker/scripts/docker-build.sh prod`
3. **Remote Deployment**: `./docker/scripts/create-deployment-package.sh`
4. **Custom Domain**: Update nginx config and add SSL certificates
5. **Monitoring**: Set up log aggregation and metrics collection

**Access your VR teleoperation system at: `https://localhost:8443/`** 🚀
