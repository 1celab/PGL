#!/bin/bash

numQueries=1000

#1:(ADJ) 2:(dynFs) 3(PMG) 4(stFs)
gs=3
#1:(regDij) 2:(advDij) 3:(A*) 0:(all)
alg=0
#1:(sp queries) 2:(delay updates) 3:(mixed) 4:(seperated)
mod=4
#evehicle
ev=0
#walking
wk=0

root=$HOME
sDir="./stats/"
DTM="$root/Projects/pgl/ResultGenerators/ExpDynTM/DynTM"
RTE="$root/Projects/pgl/ResultGenerators/ExpDynTM/RedTE"

mkdir -p $sDir

#remove old bin files
rm redTE dynTM

#make RTE
make -C $RTE
mv $RTE/redTE ./

#make DTM
make -C $DTM
mv $DTM/dynTM ./

echo -e 'Experimental Results' > $sDir/log

for format in 3
do

if [ $format == 1 ];then
    nets=( d0i eur )
    dDir="$root/Projects/Data/Graphs/TSPG/"
elif [ $format == 2 ];then
    nets=( london )
    dDir="$root/Projects/Data/Graphs/GTFS/"
else
    nets=( berlin londonNew )
    dDir="$root/Projects/Data/Graphs/GTFS/"
fi

make query

#generate queries
for net in ${nets[@]}
do
    #skip if the file already exists (for new queries the files must be deleted manually)
    if ! [[ -s $sDir$net.queries ]]; then
        ./queryGen -f $format -n $net -o $mod -q $numQueries
        mv $net.queries $sDir
    fi
done

#run experiments
for net in ${nets[@]}
do

    #RTE: (-a0 all or specific) (-a 0 plain Dijkstra) (-a 1 optimized TE Dijkstra) (-a 2 TE ALT) (-a 3 All)
    ./redTE -g $gs -n $net -f $format -a 0 -o 3 -q $sDir$net.queries

    #DTM:
    ./dynTM -g $gs -n $net -f $format -a 0 -o 3 -q $sDir$net.queries -e $ev -w $wk

    echo -e "\ntimetable: $net\n" >> ${sDir}log

    results=$(tail -n 2 $sDir$net.RTE.RDJ.results)
    echo -e "redTE-RDJ\n$results" >> ${sDir}log

    results=$(tail -n 2 $sDir$net.RTE.ODJ.results)
    echo -e "redTE-ODJ\n$results" >> ${sDir}log

    results=$(tail -n 2 $sDir$net.RTE.ALT.results)
    echo -e "redTE-ALT\n$results" >> ${sDir}log

    results=$(tail -n 2 $sDir$net.RTE.MDJ.results)
    echo -e "redTE-MDJ\n$results" >> ${sDir}log

    results=$(tail -n 2 $sDir$net.RTE.MALT.results)
    echo -e "redTE-MALT\n$results" >> ${sDir}log

    results=$(tail -n 2 $sDir$net.DTM.ODJ.results)
    echo -e "\ndynTM-ODJ\n$results" >> ${sDir}log

    results=$(tail -n 2 $sDir$net.DTM.ALT.results)
    echo -e "dynTM-ALT\n$results" >> ${sDir}log

    results=$(tail -n 2 $sDir$net.DTM.MDJ.results)
    echo -e "dynTM-MDJ\n$results" >> ${sDir}log

    results=$(tail -n 2 $sDir$net.DTM.MALT.results)
    echo -e "dynTM-MALT\n$results" >> ${sDir}log

    if [[ -s $sDir$net.RTE.RDJ.results ]]; then
        L=`wc -l <$sDir$net.RTE.RDJ.results`
        sed -e "$L,$ d" -e "1,3d" $sDir$net.RTE.RDJ.results > ${sDir}temp
        awk ' $1=="q" { print ( $(NF) ) }' ${sDir}temp > $sDir$net.RTE.RDJ.dists
    fi

    if [[ -s $sDir$net.RTE.ODJ.results ]]; then
        L=`wc -l <$sDir$net.RTE.ODJ.results`
        sed -e "$L,$ d" -e "1,3d" $sDir$net.RTE.ODJ.results > ${sDir}temp
        awk ' $1=="q" { print ( $(NF) ) }' ${sDir}temp > $sDir$net.RTE.ODJ.dists
    fi

    if [[ -s $sDir$net.RTE.ALT.results ]]; then
        L=`wc -l <$sDir$net.RTE.ALT.results`
        sed -e "$L,$ d" -e "1,3d" $sDir$net.RTE.ALT.results > ${sDir}temp
        awk ' $1=="q" { print ( $(NF) ) }' ${sDir}temp > $sDir$net.RTE.ALT.dists
    fi

    if [[ -s $sDir$net.RTE.MDJ.results ]]; then
        L=`wc -l <$sDir$net.RTE.MDJ.results`
        sed -e "$L,$ d" -e "1,3d" $sDir$net.RTE.MDJ.results > ${sDir}temp
        awk ' $1=="q" { print ( $(NF) ) }' ${sDir}temp > $sDir$net.RTE.MDJ.dists
    fi

    if [[ -s $sDir$net.RTE.MALT.results ]]; then
        L=`wc -l <$sDir$net.RTE.MALT.results`
        sed -e "$L,$ d" -e "1,3d" $sDir$net.RTE.MALT.results > ${sDir}temp
        awk ' $1=="q" { print ( $(NF) ) }' ${sDir}temp > $sDir$net.RTE.MALT.dists
    fi

    if [[ -s $sDir$net.DTM.ODJ.results ]]; then
        L=`wc -l <$sDir$net.DTM.ODJ.results`
        sed -e "$L,$ d" -e "1,3d" $sDir$net.DTM.ODJ.results > ${sDir}temp
        awk ' $1=="q" { print ( $(NF) ) }' ${sDir}temp > $sDir$net.DTM.ODJ.dists
    fi

    if [[ -s $sDir$net.DTM.ALT.results ]]; then
        L=`wc -l <$sDir$net.DTM.ALT.results`
        sed -e "$L,$ d" -e "1,3d" $sDir$net.DTM.ALT.results > ${sDir}temp
        awk ' $1=="q" { print ( $(NF) ) }' ${sDir}temp > $sDir$net.DTM.ALT.dists
    fi

    if [[ -s $sDir$net.DTM.MDJ.results ]]; then
        L=`wc -l <$sDir$net.DTM.MDJ.results`
        sed -e "$L,$ d" -e "1,3d" $sDir$net.DTM.MDJ.results > ${sDir}temp
        awk ' $1=="q" { print ( $(NF) ) }' ${sDir}temp > $sDir$net.DTM.MDJ.dists
    fi

    if [[ -s $sDir$net.DTM.MALT.results ]]; then
        L=`wc -l <$sDir$net.DTM.MALT.results`
        sed -e "$L,$ d" -e "1,3d" $sDir$net.DTM.MALT.results > ${sDir}temp
        awk ' $1=="q" { print ( $(NF) ) }' ${sDir}temp > $sDir$net.DTM.MALT.dists
    fi

    rm ${sDir}temp

    echo -e "\n"

    #check only sh path queries
    if [[ -s $sDir$net.RTE.RDJ.dists ]]; then

        if diff $sDir$net.RTE.RDJ.dists $sDir$net.RTE.ODJ.dists >/dev/null ; then
            echo "redTE ODJ correctness: ΟΚ! :)" >> ${sDir}log
        else
            echo "redTE ODJ correctness: Conflict! :(" >> ${sDir}log
        fi

        if diff $sDir$net.RTE.RDJ.dists $sDir$net.RTE.ALT.dists >/dev/null ; then
            echo "redTE ALT correctness: ΟΚ! :)" >> ${sDir}log
        else
            echo "redTE ALT correctness: Conflict! :(" >> ${sDir}log
        fi

        if diff $sDir$net.RTE.RDJ.dists $sDir$net.DTM.ODJ.dists >/dev/null ; then
            echo "dynTM ODJ correctness: ΟΚ! :)" >> ${sDir}log
        else
            echo "dynTM ODJ correctness: Conflict! :(" >> ${sDir}log
        fi

        if diff $sDir$net.RTE.RDJ.dists $sDir$net.DTM.ALT.dists >/dev/null ; then
            echo "dynTM ALT correctness: ΟΚ! :)" >> ${sDir}log
        else
            echo "dynTM ALT correctness: Conflict! :(" >> ${sDir}log
        fi
    fi

    echo -e "\n"
done

done
