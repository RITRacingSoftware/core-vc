#!/usr/bin/env bash

docker run \
	`# Run interactively` \
	-it \
	`# Optional options` \
	$2 \
	`# Mount entire repository to /vc to compile from` \
	-v /$PWD:/core-vc:cached \
	`# Run as current user so builds have correct ownership` \
	--user $(id -u):$(id -g) \
	`# Run vc image generated by setup.sh` \
	vc \
	`# Run argument script` \
	$1
