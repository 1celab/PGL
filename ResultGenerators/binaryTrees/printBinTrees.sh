#!/bin/bash

INCLUDEDIR=/home/michai/Projects/THESIS/include

if [ $# -ne "1" ];
then
    echo The script takes one argument, the tree height
    exit
fi


g++ printBinTrees.cpp -I$INCLUDEDIR

HEIGHT=$1

./a.out $HEIGHT

dot bfs.dot -o BFS$HEIGHT.svg -Tsvg
dot veb.dot -o VEB$HEIGHT.svg -Tsvg

#rm a.out veb.dot bfs.dot 
