# Tactile Teleop Unified Deployment System
# Consolidated deployment interface using docker-build.sh and deploy-setup.sh

.PHONY: help bootstrap deploy-dev deploy-prod deploy-prod-ssl setup-direct deploy-direct ssl-setup ssl-test ssl-renew clean status logs restart-nginx

help: ## Show available commands
	@echo "Tactile Teleop Deployment Commands"
	@echo "=================================="
	@echo ""
	@echo "Server Setup:"
	@echo "  bootstrap DOMAIN=... EMAIL=...  Install prerequisites (Docker, make, etc.)"
	@echo ""
	@echo "Docker Deployments (Primary):"
	@echo "  deploy-dev              Deploy development environment (HTTP)"
	@echo "  deploy-prod             Deploy production with SSL auto-detection"
	@echo "  deploy-prod-ssl         Deploy production with forced SSL setup"
	@echo ""
	@echo "Direct/Bare-metal Deployments:"
	@echo "  setup-direct DOMAIN=... Setup direct deployment configs"
	@echo "  deploy-direct DOMAIN=... Deploy to system nginx/systemd"
	@echo ""
	@echo "SSL Certificate Management:"
	@echo "  ssl-setup DOMAIN=... EMAIL=...  Interactive SSL certificate setup"
	@echo "  ssl-test DOMAIN=... EMAIL=...   Test SSL certificates (staging)"
	@echo "  ssl-renew                        Renew SSL certificates"
	@echo ""
	@echo "Utilities:"
	@echo "  clean                   Clean up containers and certificates"
	@echo "  status                  Show deployment status"
	@echo "  logs                    Show service logs"
	@echo "  restart-nginx           Restart nginx service"
	@echo ""
	@echo "Examples:"
	@echo "  sudo make bootstrap DOMAIN=yourdomain.com EMAIL=admin@yourdomain.com"
	@echo "  make deploy-dev"
	@echo "  make deploy-prod"
	@echo "  make ssl-setup DOMAIN=teleop.tactilerobotics.ai EMAIL=admin@tactilerobotics.ai"
	@echo "  make setup-direct DOMAIN=teleop.tactilerobotics.ai"

# Server Setup
bootstrap: ## Install prerequisites (Docker, make, certbot, etc.)
	@if [ "$$(id -u)" -ne 0 ]; then \
		echo "Error: Bootstrap must be run as root"; \
		echo "Usage: sudo make bootstrap DOMAIN=yourdomain.com EMAIL=admin@yourdomain.com"; \
		exit 1; \
	fi
	@echo "🚀 Installing server prerequisites..."
	./bootstrap-server.sh "$(DOMAIN)" "$(EMAIL)"

# Docker Deployments (Primary)
deploy-dev: ## Deploy development environment (HTTP only)
	@echo "🔧 Deploying development environment..."
	./docker/scripts/docker-build.sh dev

deploy-prod: ## Deploy production with SSL auto-detection
	@echo "🚀 Deploying production environment with SSL auto-detection..."
	./docker/scripts/docker-build.sh prod

deploy-prod-ssl: ## Deploy production with forced SSL setup
	@echo "🚀 Deploying production environment with automatic SSL setup..."
	AUTO_SSL=true ./docker/scripts/docker-build.sh prod

# Direct/Bare-metal Deployments
setup-direct: ## Generate configs for direct deployment
	@if [ -z "$(DOMAIN)" ]; then \
		echo "Error: DOMAIN is required"; \
		echo "Usage: make setup-direct DOMAIN=yourdomain.com"; \
		exit 1; \
	fi
	@echo "⚙️  Generating direct deployment configs for $(DOMAIN)..."
	./deploy-setup.sh "$(DOMAIN)" true

deploy-direct: ## Deploy to system nginx/systemd (requires setup-direct first)
	@if [ -z "$(DOMAIN)" ]; then \
		echo "Error: DOMAIN is required"; \
		echo "Usage: make deploy-direct DOMAIN=yourdomain.com"; \
		exit 1; \
	fi
	@echo "🏗️  Deploying to system nginx/systemd..."
	@echo "Note: This requires manual steps. See output from setup-direct command."
	@echo "Run 'make setup-direct DOMAIN=$(DOMAIN)' first if you haven't already."

# SSL Certificate Management
ssl-setup: ## Interactive SSL certificate setup
	@if [ -z "$(DOMAIN)" ] || [ -z "$(EMAIL)" ]; then \
		echo "Error: DOMAIN and EMAIL are required"; \
		echo "Usage: make ssl-setup DOMAIN=yourdomain.com EMAIL=admin@yourdomain.com"; \
		exit 1; \
	fi
	@echo "🔐 Setting up SSL certificates for $(DOMAIN)..."
	sudo ./src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh "$(DOMAIN)" "$(EMAIL)"

ssl-test: ## Test SSL certificates (staging mode)
	@if [ -z "$(DOMAIN)" ] || [ -z "$(EMAIL)" ]; then \
		echo "Error: DOMAIN and EMAIL are required"; \
		echo "Usage: make ssl-test DOMAIN=yourdomain.com EMAIL=admin@yourdomain.com"; \
		exit 1; \
	fi
	@echo "🧪 Testing SSL certificate setup for $(DOMAIN) (staging mode)..."
	sudo ./src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh "$(DOMAIN)" "$(EMAIL)" --staging

ssl-renew: ## Renew SSL certificates
	@echo "🔄 Renewing SSL certificates..."
	sudo certbot renew
	@if docker-compose ps | grep -q nginx; then \
		echo "🔄 Restarting nginx container to reload certificates..."; \
		docker-compose restart nginx; \
	fi

# Utilities
clean: ## Clean up containers and certificates
	@echo "🧹 Cleaning up containers..."
	docker-compose down --volumes --remove-orphans
	@echo "🗑️  Removing tactile-teleop images..."
	@docker images | grep tactile-teleop | awk '{print $$3}' | xargs -r docker rmi -f 2>/dev/null || true
	@echo "🧽 Cleaning up unused Docker resources..."
	docker system prune -f --volumes

status: ## Show deployment status
	@echo "📊 Deployment Status:"
	@echo "===================="
	@if docker-compose ps | grep -q Up; then \
		docker-compose ps; \
		echo ""; \
		echo "🔍 Health Checks:"; \
		HTTP_PORT=$$(docker-compose ps | grep nginx | grep -o '0\.0\.0\.0:[0-9]*->80' | cut -d':' -f2 | cut -d'-' -f1 || echo "8080"); \
		HTTPS_PORT=$$(docker-compose ps | grep nginx | grep -o '0\.0\.0\.0:[0-9]*->443' | cut -d':' -f2 | cut -d'-' -f1 || echo "8443"); \
		echo "Detected ports - HTTP: $$HTTP_PORT, HTTPS: $$HTTPS_PORT"; \
		if curl -k -f -s https://localhost:$$HTTPS_PORT/health > /dev/null 2>&1; then \
			echo "✅ HTTPS endpoint: Healthy"; \
		else \
			echo "❌ HTTPS endpoint: Unhealthy"; \
		fi; \
		if curl -f -s http://localhost:$$HTTP_PORT/health > /dev/null 2>&1; then \
			echo "✅ HTTP endpoint: Healthy"; \
		else \
			echo "❌ HTTP endpoint: Unhealthy"; \
		fi; \
	else \
		echo "No containers running. Use 'make deploy-dev' or 'make deploy-prod' to start."; \
	fi

logs: ## Show service logs
	@echo "📋 Service logs (press Ctrl+C to exit):"
	docker-compose logs -f

restart-nginx: ## Restart nginx service
	@echo "🔄 Restarting nginx service..."
	docker-compose restart nginx

stop: ## Stop all services
	@echo "🛑 Stopping all services..."
	docker-compose down