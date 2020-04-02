# Get CPU core count
n_cores=$(grep ^cpu\\scores /proc/cpuinfo | uniq |  awk '{print $4}')

[ -d build ] || mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_CXX_FLAGS="-Wno-unused-variable -Wno-unused-parameter -Wno-unused-but-set-variable" && make -j$n_cores 
cd ..