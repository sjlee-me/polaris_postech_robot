FROM python:3.11-slim

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
    libglib2.0-0 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
    "PyYAML>=6.0" \
    "paho-mqtt>=2.0.0" \
    "aiohttp>=3.9.0" \
    "aiortc>=1.6.0" \
    "numpy>=1.24.0" \
    "opencv-python-headless>=4.8.0"

COPY config.yml .
COPY test.py .
COPY core ./core

ENV PYTHONUNBUFFERED=1

CMD ["python", "-u", "test.py"]
