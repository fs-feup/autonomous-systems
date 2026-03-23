# 1. Partimos da imagem base
FROM ros:humble-ros-base-jammy

# 2. Evita interrupções
ENV DEBIAN_FRONTEND=noninteractive

# 3. Atualizar o sistema
RUN apt-get update && apt-get install -y \
    curl unzip bear git sudo python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 4. Criar pasta de trabalho
WORKDIR /fsfeup_ws

# 5. O SEGREDO DO CACHE: Copiar só o script pesado primeiro
COPY dependencies_install.sh .
RUN bash ./dependencies_install.sh && apt-get clean && rm -rf /var/lib/apt/lists/*

# 6. AGORA SIM: Copiar o resto do código todo (com os submódulos)
COPY . .

# 7. Correr a instalação do SLAM (agora já vai encontrar a pasta ext/gtsam!)
RUN bash ./src/slam/dependencies_install.sh && apt-get clean && rm -rf /var/lib/apt/lists/*

# 8. Configurar o ambiente
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc