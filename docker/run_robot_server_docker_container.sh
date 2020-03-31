echo External Logdir is $1

docker container run \
    -it \
    --rm \
    --network=host \
    --cap-add=sys_nice \
    --name robot-server \
    --mount type=bind,source=$1,target=/external_logs \
    iamlab/robot-server:latest
