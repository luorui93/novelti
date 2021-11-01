#! /bin/bash

file=$1
inflation_radius=$2

echo "###converting img to map, map will be saved in map folder###"
cmd="$HOME/anna_ws/src/novelti/novelti/scripts/novelti/MapTools.py img2map 205 
    <$HOME/anna_ws/src/novelti/novelti/map_localization/$file.pgm 
    >$HOME/anna_ws/src/novelti/novelti/map/$file.map"
echo $cmd
eval $cmd

echo "###inflate map###"
cmd="$HOME/anna_ws/src/novelti/novelti/scripts/novelti/MapTools.py inflate $inflation_radius 
    <$HOME/anna_ws/src/novelti/novelti/map/$file.map 
    >$HOME/anna_ws/src/novelti/novelti/map/${file}_inf${inflation_radius}.map"
echo $cmd
eval $cmd

# echo "###save inflated map as image for review###"
# cmd="$HOME/anna_ws/src/novelti/novelti/scripts/novelti/MapTools.py map2img 
#     <$HOME/anna_ws/src/novelti/novelti/map/${file}_inf${inflation_radius}.map
#     >$HOME/anna_ws/src/novelti/novelti/map/${file}_inf${inflation_radius}.pgm"
# eval $cmd

echo "###test for diagonally blocked cells###"
cmd="$HOME/anna_ws/src/novelti/novelti/scripts/novelti/MapTools.py find_diags 
    <$HOME/anna_ws/src/novelti/novelti/map/${file}_inf${inflation_radius}.map"
echo $cmd
eval $cmd

