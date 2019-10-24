#!/bin/bash

cp .dockin ~/.local/bin/
cp d_scripts_linux.sh ~/.local/bin/
brc=~/.local/bin
echo "source ${brc}/d_scripts_linux.sh" >> ~/.bashrc

# Setup the json parsing bash script tool
sudo apt-get install jq
