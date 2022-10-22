build: FORCE
	colcon build \
		--merge-install \
		--symlink-install

clean:
	rm -rf build install log

build-docker:
	cd script && docker build -t sineliu/ros - < Dockerfile

run-docker:
	docker run -itd \
		-v /home/sine:/home/sine \
		--privileged \
		--network=host \
		--runtime nvidia \
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

FORCE: ;
