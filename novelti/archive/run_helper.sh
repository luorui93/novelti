#!/bin/bash

#Arguments:
data_dir=$1
run_id=$2
pkg_dir=$3
param_names=$4
param_values=$5

#run_id=`date +"%T.%N"`
run_dir="$data_dir/run-$run_id"
mkdir -p $run_dir
cd "$pkg_dir"
commit=`git log --format="%H" -n 1`
prm_file="$run_dir/params.txt"
echo "run_id   commit    $param_names
$run_id    $commit   $param_values" > "$prm_file"
