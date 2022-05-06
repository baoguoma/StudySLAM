#! /bin/sh

git pull origin master
cd build
cmake ..
make