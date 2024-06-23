#!/usr/bin/env sh

######################################################################
# @author      : kistenklaus (karlsasssie@gmail.com)
# @file        : run
# @created     : Sonntag Jun 23, 2024 14:28:00 CEST
#
# @description : 
######################################################################

canzero gen mother_board src/canzero
cmake -Bbuild
make -C build
alacritty -e ./build/motherboard&



