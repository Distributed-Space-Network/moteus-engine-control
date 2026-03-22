FROM fedora:43                                                                                                                                                                                                                                

# Install dependencies (full set for Bazel builds)
RUN dnf -y update && \
    dnf -y install \
    git \
    gcc \
    gcc-c++ \
    make \
    python3 \
    which \
    unzip \
    zip \
    curl \
    patch \
    perl \
    tar \
    diffutils \
    findutils \
    file \
    bash \
    coreutils \
    sed \
    grep \
    gawk \
    gzip \
    bzip2 \
    xz \
    rsync \
    && dnf clean all

# Install Bazelisk
RUN curl -L https://github.com/bazelbuild/bazelisk/releases/latest/download/bazelisk-linux-amd64 \
    -o /usr/local/bin/bazel && \
    chmod +x /usr/local/bin/bazel

# Workspace directory
WORKDIR /workspace

# Clone YOUR fork + checkout test branch
RUN git clone https://github.com/Distributed-Space-Network/moteus-engine-control.git && \
    cd moteus-engine-control
#cd moteus-engine-control && \
#git checkout test

# Enter repo
WORKDIR /workspace/moteus-engine-control

# Build
RUN bazel build --config=target //:target

CMD ["/bin/bash"]