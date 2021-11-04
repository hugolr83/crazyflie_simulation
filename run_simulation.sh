~/argos3/build_simulator/core/argos3 -c experiments/pdr_simulation.argos &>/dev/null & disown
psax=$(ps ax | grep argos3)
echo "pid:  ${psax}"
# xhost +
# docker run --net=host --privileged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix argos_sim
