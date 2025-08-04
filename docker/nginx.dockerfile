# Nginx reverse proxy for Tactile Teleop
FROM nginx:alpine

LABEL maintainer="Tactile Robotics <info@tactilerobotics.ai>"
LABEL description="Nginx reverse proxy for Tactile Teleop"

# Remove default nginx config
RUN rm /etc/nginx/conf.d/default.conf

# Copy nginx configuration (development config by default)
COPY src/tactile_teleop/web_server/nginx/nginx.conf.dev /etc/nginx/conf.d/tactile-teleop.conf

# Create nginx user directories
RUN mkdir -p /var/cache/nginx /var/log/nginx && \
    chown -R nginx:nginx /var/cache/nginx /var/log/nginx

# Copy static files
COPY src/tactile_teleop/web_server/web-ui /usr/share/nginx/html

# SSL certificates will be managed by certbot

EXPOSE 80 443

CMD ["nginx", "-g", "daemon off;"]