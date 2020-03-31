echo Robot Client io dir is $1

docker container run \
    -it \
    --rm \
    --network=host \
    --runtime=nvidia \
    --name robot-server \
    --mount type=bind,source=$1,target=/robot_client_io \
    iamlab/robot-client:latest
