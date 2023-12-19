#!/bin/bash

rm pre
make

for format in 1 2 3
do

if [ $format == 1 ];then
nets=( i0i a0i f0i b0i efz d0i eur wez meg vib bts ks bvb )
elif [ $format = 2 ];then
nets=( london )
else
nets=( victoria madrid berlin )
fi

#generate ALT heuristics
for net in ${nets[@]}
do
    ./pre -f $format -n $net -g 1
done

done
