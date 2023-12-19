#!/bin/bash

INCLUDEDIR=/home/michai/Projects/THESIS/include

if [ $# -ne "1" ];
then
    echo The script takes one argument, the array size
    exit
fi


g++ numElementsInLeaf.cpp -I$INCLUDEDIR

SIZE=$1

./a.out $SIZE

dot out.dot -o out.svg -Tsvg

rm a.out out.dot
