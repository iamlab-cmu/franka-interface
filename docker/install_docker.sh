# install docker
sudo apt-get update
sudo apt-get install -y docker.io

# enable experimental features
echo $'{\n    "experimental": true\n}' | sudo tee /etc/docker/daemon.json;
sudo service docker restart

# add user to docker group permissions
sudo usermod -aG docker $USER
