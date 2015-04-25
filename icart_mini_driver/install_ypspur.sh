#!/bin/sh
set -ex

apt-get install git
git clone https://openspur.org/repos/yp-spur.git/
cd yp-spur
./configure
make

make install
ldconfig
cd ..
