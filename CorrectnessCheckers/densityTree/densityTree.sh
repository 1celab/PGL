#!/bin/bash

INCLUDEDIR=/home/michai/Projects/THESIS/include

if [ $# -ne "1" ];
then
    echo The script takes one argument, the tree size
    exit
fi


g++ densityTree.cpp -I$INCLUDEDIR

SIZE=$1

./a.out $SIZE

dot density.dot -o density.svg -Tsvg

#rm a.out density.dot 
