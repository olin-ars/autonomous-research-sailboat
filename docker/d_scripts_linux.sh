#!/bin/bash

function useoars() {
	if [ -z $(docker ps -a --format "{{.Names}}" | grep "oars") ]; then
        	.dockin -s 1 -n oars -i olinaquabots/base-setup:2019-2020 -v "/home/oliner/oars-research"
	else
		.dockin -s 2 -n oars
	fi
}

function cleanoars_soft() {
	docker rm oars
}

function cleanoars_hard() {
	docker rm oars
	docker rmi oars-research
}

function resetoars_soft() {
	docker rm oars
	useoars
}

function resetoars_hard() {
	docker rm oars
	docker rmi oars-research
	useoars
}

function joinoars() {
	docker exec -it oars bash
}
