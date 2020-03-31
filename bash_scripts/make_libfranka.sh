cd libfranka

# Change built test to Off
sed -i '155s/.*/option(BUILD_TESTS "Build tests" OFF)/' CMakeLists.txt

# Build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .

cd ../..

# Copy needed files from libfranka cmake. -n means don't copy if dest. file exists
[ -d cmake ] || mkdir cmake
cp -n libfranka/cmake/FindEigen3.cmake cmake/
cp -n libfranka/cmake/FindPoco.cmake cmake/