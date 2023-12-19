#!/bin/bash

f=1
maps=( NY BAY COL FLA NW NE CAL LKS E W CTR )

for map in ${maps[@]}
do

echo $map

#map='E'
#numQeries='10000'
#numLandmarks='120'
#activeLandmarks='16'
lmkSelection='3'

mkdir -p ./stats/$map/Adjacency/
mkdir -p ./stats/$map/PMA/
mkdir -p ./stats/$map/ForwardStar/

make file=queryGen.cpp

./a.out -q $numQeries -f $f -m $map

#make file=lmkGen.cpp

#./a.out -l $numLandmarks -s $lmkSelection -f $f -m $map

#make file=results.cpp

#./a.out -g 0 -a 1 -f $f -m $map

for numLandmarks in 1 2 4 6 8 10 12 14 16 18 20 22 24
do
  
  make file=lmkGen.cpp
  ./a.out -l $numLandmarks -s $lmkSelection -f 1 -m $map

  make file=results.cpp

  for la in $(seq 1 1 $numLandmarks); #{1..$numLandmarks..1};
  do
    for i in `seq 7 10`;
    do
    ./a.out -g 0 -a $i -f $f -m $map -l $la
  done
  mv ./stats/${map}/Adjacency/uniLmk.dat ./stats/${map}/Adjacency/uniLmk_${la}_${numLandmarks}.dat
  mv ./stats/${map}/Adjacency/bidSymLmk.dat ./stats/${map}/Adjacency/bidSymLmk_${la}_${numLandmarks}.dat
  mv ./stats/${map}/Adjacency/bidMaxLmk.dat ./stats/${map}/Adjacency/bidMaxLmk_${la}_${numLandmarks}.dat
  mv ./stats/${map}/Adjacency/bidAveLmk.dat ./stats/${map}/Adjacency/bidAveLmk_${la}_${numLandmarks}.dat

  mv ./stats/${map}/PMA/uniLmk.dat ./stats/${map}/PMA/uniLmk_${la}_${numLandmarks}.dat
  mv ./stats/${map}/PMA/bidSymLmk.dat ./stats/${map}/PMA/bidSymLmk_${la}_${numLandmarks}.dat
  mv ./stats/${map}/PMA/bidMaxLmk.dat ./stats/${map}/PMA/bidMaxLmk_${la}_${numLandmarks}.dat
  mv ./stats/${map}/PMA/bidAveLmk.dat ./stats/${map}/PMA/bidAveLmk_${la}_${numLandmarks}.dat
done
mv ~/Projects/Graphs/DIMACS9/$map/landmarks.dat ~/Projects/Graphs/DIMACS9/$map/landmarks_${numLandmarks}.dat
#target="$map$la.tar.gz"
#tar -czvf $target ./stats/$map/PMA/
done

done
