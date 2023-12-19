#!/bin/bash
for i in $(seq 1 113)
do
    dot out"$i"s.dot -Tsvg -o out"$i"s.svg
    dot out"$i"f.dot -Tsvg -o out"$i"f.svg
done
