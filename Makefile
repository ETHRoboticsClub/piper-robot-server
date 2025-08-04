# Tactile Teleop SSL Certificate Management
# Based on register_ssl.sh script

.PHONY: help certbot-test certbot-prod deploy-dev deploy-prod clean-certs

help: ## Show available commands
	@echo "Available commands:"
	@echo ""
	@echo "  certbot-test DOMAINS=\"domain.com www.domain.com\" EMAIL=admin@domain.com"
	@echo "    Test SSL certificate generation (staging mode)"
	@echo ""
	@echo "  certbot-prod DOMAINS=\"domain.com www.domain.com\" EMAIL=admin@domain.com"
	@echo "    Generate production SSL certificates"
	@echo ""
	@echo "  deploy-dev"
	@echo "    Deploy with development configuration (HTTP only)"
	@echo ""
	@echo "  deploy-prod"
	@echo "    Deploy with production configuration (HTTPS with Let's Encrypt)"
	@echo ""
	@echo "  clean-certs"
	@echo "    Remove all SSL certificates and start fresh"

certbot-test: ## Test SSL certificate generation (staging)
	@if [ -z "$(DOMAINS)" ] || [ -z "$(EMAIL)" ]; then \
		echo "Error: DOMAINS and EMAIL are required"; \
		echo "Usage: make certbot-test DOMAINS=\"domain.com www.domain.com\" EMAIL=admin@domain.com"; \
		exit 1; \
	fi
	sudo bash src/tactile_teleop/web_server/register_ssl.sh \
		--domains "$(DOMAINS)" \
		--email "$(EMAIL)" \
		--data-path "./certbot" \
		--staging 1

certbot-prod: ## Generate production SSL certificates
	@if [ -z "$(DOMAINS)" ] || [ -z "$(EMAIL)" ]; then \
		echo "Error: DOMAINS and EMAIL are required"; \
		echo "Usage: make certbot-prod DOMAINS=\"domain.com www.domain.com\" EMAIL=admin@domain.com"; \
		exit 1; \
	fi
	sudo bash src/tactile_teleop/web_server/register_ssl.sh \
		--domains "$(DOMAINS)" \
		--email "$(EMAIL)" \
		--data-path "./certbot" \
		--staging 0

deploy-dev: ## Deploy with development configuration (HTTP only)
	@echo "Deploying with development configuration (HTTP only)..."
	docker-compose up -d --build

deploy-prod: ## Deploy with production configuration (HTTPS)
	@echo "Deploying with production configuration (HTTPS)..."
	docker-compose -f docker-compose.yml -f docker-compose.ssl.yml up -d --build

clean-certs: ## Remove all SSL certificates
	sudo rm -rf ./certbot/conf/live/
	sudo rm -rf ./certbot/conf/archive/
	sudo rm -rf ./certbot/conf/renewal/
	@echo "SSL certificates removed. Run certbot-test or certbot-prod to regenerate."

stop: ## Stop all services
	docker-compose down

logs: ## Show logs for all services
	docker-compose logs -f

restart-nginx: ## Restart nginx service
	docker-compose restart nginx