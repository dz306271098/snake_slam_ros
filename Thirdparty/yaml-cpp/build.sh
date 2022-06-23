mkdir build install
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install -DYAML_BUILD_SHARED_LIBS=ON ..
make -j8
make install
