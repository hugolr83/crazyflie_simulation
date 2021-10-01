echo $$
# argos3 -c experiments/pdr_simulation.argos
xhost +
docker run --net=host --privileged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix argos_sim
