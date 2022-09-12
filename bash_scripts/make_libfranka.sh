cd libfranka

test_line_number=$(grep -n "Build tests" CMakeLists.txt | cut -f1 -d:)

# Change built test to Off
sed -i $test_line_number's/.*/option(BUILD_TESTS "Build tests" OFF)/' CMakeLists.txt

# Get CPU core count
n_cores=$(grep ^cpu\\scores /proc/cpuinfo | uniq |  awk '{print $4}')

# Build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j$n_cores

cd ../..

# Copy needed files from libfranka cmake. -n means don't copy if dest. file exists
[ -d cmake ] || mkdir cmake
cp -n libfranka/cmake/FindEigen3.cmake cmake/
cp -n libfranka/cmake/FindPoco.cmake cmake/
