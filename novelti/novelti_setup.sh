
#! /usr/bin/env bash

wget http://www.libsdl.org/release/SDL2-2.0.8.zip
unzip SDL2-2.0.8.zip
cd SDL2-2.0.8
./configure
make 
sudo make install

wget http://www.libsdl.org/projects/SDL_ttf/release/SDL2_ttf-2.0.14.zip
unzip SDL2_ttf-2.0.14.zip
cd SDL2_ttf-2.0.14
./configure
make
sudo make install
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib