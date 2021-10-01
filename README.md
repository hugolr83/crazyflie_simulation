# inf3995-simulation

ARGoS physics-based simulator

## Getting started

Build docker image :

        docker build --tag <image_name> .

Run docker image :

This is temporary for now. Need to have X on your machine
        
        xhost + 
        
        docker run -d --net=host --priviliged -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix


## Development


        mkdir build
        cmake ..
        make
        cd ../ && argos3 -c experiments/pdr_simulation.argos
