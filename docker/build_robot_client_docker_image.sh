cd robot-client

mkdir -p .docker_tmp/docker_util_scripts
cp ../iamlab_docker_github_token.txt .docker_tmp/
cp -r ../docker_util_scripts .docker_tmp/

docker build \
    --squash \
    -t iamlab/robot-client:latest \
    -t iamlab/robot-client:$1 \
    --build-arg GITHUB_TOKEN="$(cat .docker_tmp/iamlab_docker_github_token.txt)" \
    --build-arg ROBOT_SERVER_IP="192.168.1.3" \
    .
