# inf3995-simulation

ARGoS physics-based simulator

## Getting started

Build docker image :

        docker build --tag <image_name> .

Run docker image :

This is temporary for now. Need to have X on your machine
        
        xhost + 
        
        docker run -d --net=host --privileged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix <image_name>


## Development


        mkdir build
        cmake ..
        make
        cd ../ && argos3 -c experiments/pdr_simulation.argos

### Debug

Launch simulation with `run_simulation.sh` and take note of the PID output

Attach gdb to the process with `gdb -p PID` *You might need to run sudo with this command (figure out why)

Set breakpoint and start simulation
