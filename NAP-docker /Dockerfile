# syntax=docker/dockerfile:1
#
# Autoware (ROS 2 Humble, Ubuntu 22.04) + vanetza-nap (socktap)
# Base:  ghcr.io/autowarefoundation/autoware:universe-devel-cuda-humble
# NAP :  https://github.com/nap-it/vanetza-nap
#
# Egyetlen stage: az Autoware image-re ráépítjük a NAP socktap-et.
# A ROS-os entrypoint NEM kerul felulirasra -> az image ugyanugy mukodik mint
# egy sima Autoware devel image, csak van benne socktap + a vanetza configok.
#
FROM ghcr.io/autowarefoundation/autoware:universe-devel-cuda-humble

# Az autoware image alapertelmezetten nem-root userben futhat -> build alatt root kell
USER root

ARG CARGO_HOME=/root/.cargo
ARG DEBIAN_FRONTEND=noninteractive
# allitsd ha mas branch/tag kell a NAP forkbol (default branch ha ures)
ARG VANETZA_REF=

ENV TZ=Europe/Budapest
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# ---------------------------------------------------------------------------
# Build + runtime fuggosegek (Jammy package nevek, NINCS bullseye-backports pin)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    mosquitto \
    libboost-date-time-dev \
    libmosquittopp-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    libasio-dev \
    libtinyxml2-dev \
    libcrypto++-dev \
    libgeographic-dev \
    libgps-dev \
    libssl-dev \
    libnl-3-dev \
    libnl-genl-3-dev \
    zlib1g-dev \
    libcurl4-openssl-dev \
    ca-certificates \
    file \
    iproute2 \
    iptables \
    bridge-utils \
    ebtables \
    tcpdump \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Rust (Zenoh-hoz)
# ---------------------------------------------------------------------------
ENV PATH="${CARGO_HOME}/bin:${PATH}"
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y \
    && cargo --version

# ---------------------------------------------------------------------------
# zenoh-c 1.4.0
# ---------------------------------------------------------------------------
WORKDIR /tmp
RUN git clone https://github.com/eclipse-zenoh/zenoh-c.git \
    && cd zenoh-c \
    && git checkout 1.4.0 \
    && mkdir build && cd build \
    && cmake .. \
        -DZENOHC_BUILD_WITH_UNSTABLE_API=true \
        -DZENOHC_BUILD_WITH_SHARED_MEMORY=true \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build . --config Release \
    && cmake --build . --target install

# ---------------------------------------------------------------------------
# zenoh-cpp 1.4.0
# ---------------------------------------------------------------------------
WORKDIR /tmp
RUN git clone https://github.com/eclipse-zenoh/zenoh-cpp.git \
    && cd zenoh-cpp \
    && git checkout 1.4.0 \
    && mkdir build && cd build \
    && cmake .. \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DZENOHCXX_ENABLE_TESTS=OFF \
        -DZENOHCXX_ENABLE_EXAMPLES=OFF \
        -Dzenohc_DIR=/usr/local/lib/cmake/zenohc \
    && cmake --build . --target install

# ---------------------------------------------------------------------------
# prometheus-cpp (pinnelt commit)
# ---------------------------------------------------------------------------
WORKDIR /tmp
RUN git clone https://github.com/jupp0r/prometheus-cpp.git \
    && cd prometheus-cpp \
    && git checkout a944ec100251019cd44d070bbf2fd22f5139d6d0 \
    && git submodule update --init --recursive \
    && cmake -B_build -DCPACK_GENERATOR=DEB -DBUILD_SHARED_LIBS=ON \
    && cmake --build _build --target package --parallel "$(nproc)" \
    && dpkg -i _build/*.deb

# ---------------------------------------------------------------------------
# Fast-DDS stack -> /usr/local
# FONTOS: a ROS Humble sajat Fast-DDS-e (2.6.x) a /opt/ros/humble alatt van.
# A NAP socktap a 2.13.2-t igenyli, ezert kulon buildeljuk /usr/local-ba.
# ---------------------------------------------------------------------------
WORKDIR /tmp
RUN git clone https://github.com/eProsima/foonathan_memory_vendor.git \
    && cmake -S foonathan_memory_vendor -B foonathan_memory_vendor/build \
        -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON \
    && cmake --build foonathan_memory_vendor/build --target install --parallel "$(nproc)"

RUN git clone https://github.com/eProsima/Fast-CDR.git \
    && cmake -S Fast-CDR -B Fast-CDR/build -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build Fast-CDR/build --target install --parallel "$(nproc)"

RUN git clone https://github.com/eProsima/Fast-DDS.git \
    && cd Fast-DDS && git checkout v2.13.2 && cd .. \
    && cmake -S Fast-DDS -B Fast-DDS/build -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build Fast-DDS/build --target install --parallel "$(nproc)"

RUN ldconfig

# ---------------------------------------------------------------------------
# Vanetza (NAP fork) + socktap
# Az RPATH-ot bedrotozzuk /usr/local/lib-re es --disable-new-dtags-szel
# DT_RPATH-ot keszitunk, hogy a socktap AKKOR is a /usr/local Fast-DDS 2.13.2-t
# toltse, ha kozben source-oltad a ROS setup.bash-t (LD_LIBRARY_PATH=/opt/ros...).
# Ez kerüli el a Fast-DDS verzio-utkozes okozta crasht.
# ---------------------------------------------------------------------------
WORKDIR /
RUN git clone --recurse-submodules https://github.com/nap-it/vanetza-nap.git /vanetza \
    && if [ -n "$VANETZA_REF" ]; then \
         cd /vanetza && git checkout "$VANETZA_REF" && git submodule update --init --recursive; \
       fi

WORKDIR /vanetza
RUN rm -f CMakeCache.txt \
    && cmake . \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH=/usr/local \
        -DCMAKE_INSTALL_RPATH=/usr/local/lib \
        -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
        -DCMAKE_EXE_LINKER_FLAGS="-Wl,--disable-new-dtags" \
    && cmake --build . --target socktap -j "$(nproc)" \
    && cp /vanetza/bin/socktap /usr/local/bin/socktap \
    && ldconfig

# Configok + NAP entrypoint kulon neven (a ROS entrypoint marad az alapertelmezett)
RUN cp /vanetza/tools/socktap/config.ini        /config.ini \
    && cp /vanetza/tools/socktap/zenoh_config.json5 /zenoh_config.json5 \
    && cp /vanetza/entrypoint.sh                 /vanetza-entrypoint.sh \
    && chmod +x /vanetza-entrypoint.sh

ENV EMBEDDED_MOSQUITTO_PORT=1883

WORKDIR /
# ENTRYPOINT-ot SZANDEKOSAN nem allitunk -> az Autoware /ros_entrypoint.sh marad,
# igy ROS source-olva indul a container. A socktap-et kezzel inditod:
#   docker exec -it <ctr> /vanetza-entrypoint.sh
#   vagy:  socktap -c /config.ini ...
