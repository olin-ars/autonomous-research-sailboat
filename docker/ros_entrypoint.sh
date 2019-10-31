#!/bin/bash

source /opt/ros/melodic/setup.bash
echo '----------------------------------------------'
echo $'\e[35m'
echo '                 ·""""·                       '
echo '                 """"""                       '
echo '                 """"""·                      '
echo $'    ·"\e[0m|||||\e[35m""··""""\e[0m|\e[35m"·"""·""\e[0m||||\e[35m"·\e[0m   "||||·  '
echo $'  ·|II||||II|\e[35m"""""\e[0m I|· \e[0;35m··\e[0m |II| |II|  |I|  I|· '
echo '  |II·    "II|   || ||·   |II  ·II   |II|"   '
echo '  III     ·III  "I|  |I"  |IIIIII|·   . |II '
echo '  "III""""III· "III||III· |II"·|II· ·||  III '
echo '    "|IIII|"  "|||""""||| "||·  |||" "|III||· '
echo
echo $'                       Copyright (c) 2019 OARS\e[0m'
echo '----------------------------------------------'
echo
echo " > Welcome to the OARS Docker Container."
echo
exec "$@"
