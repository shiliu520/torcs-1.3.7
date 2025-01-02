#! /bin/sh

export CFLAGS="-fPIC -g -ggdb -O0"
export CPPFLAGS="$CFLAGS"
export CXXFLAGS="$CFLAGS"
./configure --prefix=$(pwd)/BUILD  # local install dir
make
make install
make datainstall

cd scr_client
make