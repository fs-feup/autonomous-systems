# 1. Partimos da imagem base que já usavam
FROM ros:humble-ros-base-jammy

# 2. Evita que o Linux fique à espera de inputs do utilizador (Y/n) durante as instalações
ENV DEBIAN_FRONTEND=noninteractive

# 3. Atualizar o sistema e instalar ferramentas base
RUN apt-get update && apt-get install -y \
    curl \
    unzip \
    bear \
    git \
    sudo \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 4. Criar pasta de trabalho
WORKDIR /fsfeup_ws

# 5. Copiar APENAS os scripts de dependências (para aproveitar a cache do Docker)
COPY dependencies_install.sh .
COPY src/slam/dependencies_install.sh ./src/slam/

# 6. Correr as instalações pesadas (Isto é o que demorava 10 minutos!)
RUN bash ./dependencies_install.sh
RUN bash ./src/slam/dependencies_install.sh

# 7. Configurar o ambiente do utilizador root no container
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc