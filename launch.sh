rm -rf scr_client/TorcsOutput.log
rm -rf scr_client/TireOutput.log
cd /ssd/open_source/torcs/torcs-1.3.7/BUILD/share/games/torcs
export LD_LIBRARY_PATH=/ssd/open_source/torcs/torcs-1.3.7/BUILD/lib/torcs/lib:/ssd/open_source/torcs/torcs-1.3.7/BUILD/lib/torcs/lib
# /ssd/open_source/torcs/torcs-1.3.7/BUILD/lib/torcs/torcs-bin -l /home/uisee/.torcs -L /ssd/open_source/torcs/torcs-1.3.7/BUILD/lib/torcs -D /ssd/open_source/torcs/torcs-1.3.7/BUILD/share/games/torcs
/ssd/open_source/torcs/torcs-1.3.7/BUILD/lib/torcs/torcs-bin -l /home/uisee/.torcs -L /ssd/open_source/torcs/torcs-1.3.7/BUILD/lib/torcs -D /ssd/open_source/torcs/torcs-1.3.7/BUILD/share/games/torcs -r ~/.torcs/config/raceman/quickrace.xml