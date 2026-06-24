# ==========================================
# 1. STAGE: Build környezet
# ==========================================
FROM ghcr.io/autowarefoundation/autoware:universe-devel-cuda-humble AS build

USER root
ENV DEBIAN_FRONTEND=noninteractive

# Alapvető csomagok telepítése (Ubuntu 22.04 Jammy kompatibilis listával)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl mosquitto libboost-date-time-dev \
    libmosquittopp-dev libboost-program-options-dev libboost-system-dev \
    libasio-dev libtinyxml2-dev libcrypto++-dev libgeographic-dev \
    libgps-dev libssl-dev libnl-3-dev libnl-genl-3-dev zlib1g-dev \
    libcurl4-openssl-dev ca-certificates file \
    && rm -rf /var/lib/apt/lists/*

# Rust telepítése a Zenoh fordításához
ENV CARGO_HOME=/root/.cargo
ENV PATH="${CARGO_HOME}/bin:${PATH}"
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y

# Zenoh-C
WORKDIR /tmp
RUN git clone https://github.com/eclipse-zenoh/zenoh-c.git \
    && cd zenoh-c \
    && git checkout 1.4.0 \
    && mkdir build && cd build \
    && cmake .. -DZENOHC_BUILD_WITH_UNSTABLE_API=true -DZENOHC_BUILD_WITH_SHARED_MEMORY=true -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build . --config Release \
    && cmake --build . --target install

# Zenoh-CPP
WORKDIR /tmp
RUN git clone https://github.com/eclipse-zenoh/zenoh-cpp.git \
    && cd zenoh-cpp \
    && git checkout 1.4.0 \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DZENOHCXX_ENABLE_TESTS=OFF -DZENOHCXX_ENABLE_EXAMPLES=OFF -Dzenohc_DIR=/usr/local/lib/cmake/zenohc \
    && cmake --build . --target install

# Prometheus dependencies
WORKDIR /tmp
RUN git clone https://github.com/jupp0r/prometheus-cpp.git \
    && cd prometheus-cpp \
    && git submodule update --init \
    && git checkout a944ec100251019cd44d070bbf2fd22f5139d6d0 \
    && cmake -B_build -DCPACK_GENERATOR=DEB -DBUILD_SHARED_LIBS=ON \
    && cmake --build _build --target package --parallel $(nproc) \
    && dpkg -i _build/*.deb

# DDS dependencies (A saját verziókat /usr/local alá tesszük, hogy ne akadjon össze az Autoware FastDDS-ével)
WORKDIR /tmp
RUN git clone https://github.com/eProsima/foonathan_memory_vendor.git \
    && cd foonathan_memory_vendor && mkdir build && cd build \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON \
    && cmake --build . --target install --parallel $(nproc)

WORKDIR /tmp
RUN git clone https://github.com/eProsima/Fast-CDR.git \
    && cd Fast-CDR && mkdir build && cd build \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/ \
    && cmake --build . --target install --parallel $(nproc)

WORKDIR /tmp
RUN git clone https://github.com/eProsima/Fast-DDS.git \
    && cd Fast-DDS \
    && git checkout v2.13.2 \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/ \
    && cmake --build . --target install --parallel $(nproc)

# Vanetza-NAP klónozása és fordítása
# A CMAKE_PREFIX_PATH garantálja, hogy megtalálja a zenohcxx package configot
WORKDIR /vanetza
RUN git clone https://github.com/nap-it/vanetza-nap.git . \
    && rm -f CMakeCache.txt \
    && cmake . -DCMAKE_PREFIX_PATH=/usr/local \
    && cmake --build . --target socktap -j $(nproc) \
    && cp bin/socktap /usr/local/bin/socktap

# ==========================================
# 2. STAGE: Végső Autoware Image kiegészítése
# ==========================================
FROM ghcr.io/autowarefoundation/autoware:universe-devel-cuda-humble AS final

USER root
ENV DEBIAN_FRONTEND=noninteractive

# Futásidejű függőségek telepítése
RUN apt-get update && apt-get install -y --no-install-recommends \
    mosquitto libboost-date-time-dev libmosquittopp-dev \
    libboost-program-options-dev libboost-system-dev \
    libtinyxml2-dev libcrypto++-dev libgeographic-dev \
    libnl-3-dev libnl-genl-3-dev libssl-dev zlib1g-dev \
    iproute2 libgps-dev iptables bridge-utils ebtables \
    tcpdump libcurl4-openssl-dev ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Binárisok és könyvtárak másolása a build stage-ből
COPY --from=build /usr/local/bin/socktap /usr/local/bin/socktap
COPY --from=build /usr/local/lib/libfast* /usr/local/lib/
COPY --from=build /usr/local/lib/libfoon* /usr/local/lib/
COPY --from=build /usr/local/lib/libzenoh* /usr/local/lib/
COPY --from=build /tmp/prometheus-cpp/_build/*.deb /deps/
RUN dpkg -i /deps/*.deb && rm -rf /deps

# Linker cache frissítése, hogy a /usr/local/lib dinamikus könyvtárait felismerje a rendszer
RUN ldconfig

# Mappa előkészítése a kívülről beolvasott config fájloknak
RUN mkdir -p /vanetza_config

# Visszatérés az Autoware alapkönyvtárába
WORKDIR /autoware