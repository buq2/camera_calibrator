FROM conanio/gcc9

ENV DEBIAN_FRONTEND=noninteractive
RUN sudo ln -fs /usr/share/zoneinfo/Europe/Amsterdam /etc/localtime
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    libegl1-mesa-dev libgtk2.0-dev libgl1-mesa-dev libglu1-mesa-dev \
    libxcb-icccm4-dev libxcb-image0-dev libxcb-keysyms1-dev \
    libxcb-render-util0-dev libxcb-util-dev libxcb-xinerama0-dev \
    libxcb-xkb-dev xkb-data xorg-dev && \
    sudo rm -rf /var/lib/apt/lists/*

COPY . .
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --parallel 12
