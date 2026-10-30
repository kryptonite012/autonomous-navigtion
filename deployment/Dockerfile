# Backend + built dashboard. Suitable for simulation mode on any machine and
# for hardware mode on a Pi (devices are passed through in docker-compose.yml).
FROM node:22-bookworm-slim AS web
WORKDIR /src/frontend
COPY frontend/package.json frontend/package-lock.json ./
RUN npm ci --no-audit --no-fund
COPY frontend/ ./
RUN npm run build

FROM python:3.11-slim-bookworm
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/nexus
COPY requirements.txt requirements-pi.txt ./
ARG WITH_PI=false
RUN pip install --no-cache-dir -r requirements.txt \
    && if [ "$WITH_PI" = "true" ]; then pip install --no-cache-dir -r requirements-pi.txt; fi
COPY robot/ robot/
COPY backend/ backend/
COPY config/ config/
COPY scripts/ scripts/
COPY --from=web /src/frontend/dist frontend/dist
RUN useradd -r -u 1001 nexus && mkdir -p data backups && chown -R nexus data backups config
USER nexus
EXPOSE 8000
HEALTHCHECK --interval=30s --timeout=3s CMD python -c "import urllib.request;urllib.request.urlopen('http://127.0.0.1:8000/api/ping')"
CMD ["python", "-m", "backend.main"]
