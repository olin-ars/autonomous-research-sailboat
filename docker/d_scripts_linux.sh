#!/bin/bash

function useoars() {
	  if [ -z $(docker ps -a --format "{{.Names}}" | grep "oars") ]; then
				.dockin -s 1 -n oars -i olinoars/ros:2019-2020 -v oars-volume
    else
				.dockin -s 2 -n oars
    fi
}

function cleanoars_soft() {
		docker rm oars
}

function cleanoars_hard() {
		docker rm oars
		docker rmi oars-volume
}

function resetoars_soft() {
		docker rm oars
		useoars
}

function resetoars_hard() {
		docker rm oars
		docker rmi oars-volume
		useoars
}

function joinoars() {
		docker exec -it oars bash
}
