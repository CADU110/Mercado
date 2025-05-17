#!/bin/bash

cd cadu/

echo -e "#========================# Dataset A #========================#\n"

for i in {01..20}; do
    echo "Testando instancia: $i"
    python3 checker.py "datasets/a/instance_00${i}.txt" "output/instance_00${i}.txt"
    echo
done

echo -e "#========================# Dataset B #========================#\n"

for i in {01..15}; do
    echo "Testando instancia: $i"
    python3 checker.py "datasets/b/instance_00${i}.txt" "../output/b/instance_00${i}.txt"
    echo
done

exit 0
