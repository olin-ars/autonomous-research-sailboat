#!/bin/bash

cp .dockin ~/.local/bin/
cp d_setup_linux.sh ~/.local/bin/
brc=~/.local/bin
echo "source ${brc}/d_setup_linux.sh" >> ~/.bashrc
