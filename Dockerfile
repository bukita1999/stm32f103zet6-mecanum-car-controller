# 使用Python 3.13作为基础镜像
FROM python:3.13-slim

# 设置环境变量
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1
ENV DEBIAN_FRONTEND=noninteractive

# 安装系统依赖（需要root权限）
RUN apt-get update && apt-get install -y \
    # 串口通信相关依赖
    python3-dev \
    build-essential \
    # USB设备支持
    usbutils \
    udev \
    # GUI支持（用于matplotlib和tkinter）
    python3-tk \
    tk-dev \
    # X11支持（可选，用于GUI显示）
    libx11-dev \
    libxext-dev \
    libxtst-dev \
    # 字体支持
    fonts-liberation \
    # 清理缓存
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# 创建非root用户
RUN useradd --create-home --shell /bin/bash --uid 1000 app

# 设置工作目录
WORKDIR /app

# 复制requirements.txt（在root权限下）
COPY python/requirements.txt .

# 安装Python依赖（在root权限下，以确保权限正确）
RUN pip install --no-cache-dir --upgrade pip setuptools wheel

# 创建requirements.txt的清理版本（去掉tkinter，因为它是系统包）
RUN grep -v "^tkinter" requirements.txt > requirements_clean.txt

# 安装Python包
RUN pip install --no-cache-dir -r requirements_clean.txt

# 复制Python应用代码
COPY python/ ./

# 设置正确的权限
RUN chown -R app:app /app && \
    chmod +x start.sh && \
    mkdir -p /app/data && \
    chown -R app:app /app/data

# 切换到非root用户
USER app

# 设置环境变量
ENV PYTHONPATH=/app
ENV DATA_DIR=/app/data
ENV MPLBACKEND=Agg

# 暴露端口（如果需要的话）
# EXPOSE 8080

# 设置默认启动命令
CMD ["python3", "--version"]

# 添加健康检查
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python3 -c "import sys, yaml, serial, pandas; print('All dependencies OK')" || exit 1

# 添加标签
LABEL maintainer="AI Assistant"
LABEL description="STM32 Robot Control Docker Image"
LABEL version="1.0"
