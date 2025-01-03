#! /bin/bash

# a shell to calc stack memory of torcs-bin
pid=$(ps aux | grep "[t]orcs-bin" | grep games | awk '{print $2}')
if [ -n "$pid" ]; then
    stack_info=$(cat /proc/$pid/maps | grep "\[stack\]")
    if [ -n "$stack_info" ]; then
        start=$(echo $stack_info | cut -d'-' -f1)
        end=$(echo $stack_info | cut -d'-' -f2 | cut -d' ' -f1)
        start_dec=$(echo $(("16#$start")))
        end_dec=$(echo $(("16#$end")))
        size_in_bytes=$((end_dec - start_dec))
        size_in_mb=$(echo "scale=2; $size_in_bytes / (1024 * 1024)" | bc)
        echo "$size_in_mb MB"
    fi
fi
