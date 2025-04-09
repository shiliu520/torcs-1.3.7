#! /bin/sh

export CFLAGS="-fPIC -g -ggdb -O0 -fstack-protector-all"
export CPPFLAGS="$CFLAGS"
export CXXFLAGS="$CFLAGS"
./configure --prefix=$(pwd)/BUILD  # local install dir
make
make install
cp src/drivers/bt/bt.so $(pwd)/BUILD/lib/torcs/drivers/bt
cp src/modules/simu/simuv2/simuv2.so $(pwd)/BUILD/lib/torcs/modules/simu
make datainstall
# The output of BUILD/bin/torcs -e is used to generate gdb_launch.sh
# BUILD/bin/torcs -e > gdb_launch.sh
# sed '/^Saving/d' gdb_launch.sh > temp.sh && mv temp.sh gdb_launch.sh
# sed -i 's/ *$//' gdb_launch.sh

cd scr_client
make