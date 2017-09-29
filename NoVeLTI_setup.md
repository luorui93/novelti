Ubuntu 14.04 + ROS_Indigo Build Step 

1. git clone the repos

2. copy repos to workspace/src

3. install SDL2: 
    wget http://www.libsdl.org/release/SDL2-2.0.4.zip
    unpcak
    ./configure
    make 
    sudo make install

4. install SDL2_ttf:
    wget http://www.libsdl.org/projects/SDL_ttf/release/SDL2_ttf-2.0.14.zip
    unpack
    ./configure
    make
    sudo make install
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

5. obtain CWave binary (https://zzzzzz.no-ip.org/d/f91c361317994c77ae4d/)
    libcwaveproc.a
    put it into a lib folder and update the library path in CMakeLists.txt
    SET( CWAVE_LIB_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/lib") 

6. upgrade g++ compiler to 4.9.0. The default compiler of Ubuntu 14.04 is 4.8.x
    (encounter error "undefined reference to `__cxa_throw_bad_array_new_length'")

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install g++-4.9
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.9 50

7. catkin_make