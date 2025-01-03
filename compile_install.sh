#! /bin/sh

export CFLAGS="-fPIC -g -ggdb -O0 -fstack-protector-all"
export CPPFLAGS="$CFLAGS"
export CXXFLAGS="$CFLAGS"
./configure --prefix=$(pwd)/BUILD  # local install dir
make
make install
make datainstall
BUILD/bin/torcs -e > gdb_launch.sh
sed '/^Saving/d' gdb_launch.sh > temp.sh && mv temp.sh gdb_launch.sh
sed -i 's/ *$//' gdb_launch.sh

cd scr_client
make