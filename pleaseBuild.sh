#cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=ON -Bbuild/default -H.

#cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=install -Bbuild/default -H.

#cmake --build build/defaul

#sudo cmake --build build/default --target install

#DO LOCAL install:

cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=install -Bbuild/default -H.
cmake --build build/default --target install
