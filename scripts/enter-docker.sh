#!/usr/bin/env bash

docker run -it \
	--mount type=bind,source=$PWD,destination=/core-vc \
	core-vc
