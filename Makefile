build:
	colcon build \
		--merge-install \
		--symlink-install

build-docker:
	cd script && docker build -t sineliu/ros - < Dockerfile

run-docker:
	docker run -itd \
		-v /home/sine:/home/sine \
		--network=host \
		--cap-add=SYS_PTRACE \
		--security-opt=seccomp:unconfined \
		--security-opt=apparmor:unconfined \
		--name ros \
		sineliu/ros

attach-docker:
	docker exec -it --user sine ros bash

rm-docker:
	docker stop ros
	docker rm ros