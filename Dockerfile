FROM registry.gitlab.com/polytechnique-montr-al/inf3995/20213/equipe-100/inf3995-simulation/argos:0.0.1

# Configuration for the nvidia container runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

WORKDIR /inf3995-simulation
COPY controllers/ controllers/
COPY experiments/ experiments/
COPY socket_link/ socket_link/
COPY CMakeLists.txt ./

# Compile the simulation
RUN  cd /inf3995-simulation && \ 
    mkdir build && \
    cd build && \
    cmake .. && make -j $(nproc)

CMD ["argos3", "-c", "experiments/pdr_simulation.argos"]
