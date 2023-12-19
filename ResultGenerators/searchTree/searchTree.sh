#!/bin/bash

INCLUDEDIR=/home/michai/Projects/THESIS/include

if [ $# -ne "2" ];
then
    echo The script takes two arguments, the array size, and the number of searches to execute
    exit
fi


g++ searchTree.cpp -I$INCLUDEDIR -lrt

SIZE=$1
ITERATIONS=$2

./a.out $SIZE $ITERATIONS

rm a.out
