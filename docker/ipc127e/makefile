current_dir = $(shell pwd)

download-sources:
	git clone --branch feature/fleet-demo-case git@github.com:EduArt-Robotik/edu_swarm.git
	git clone --branch develop git@github.com:EduArt-Robotik/edu_robot.git

build-docker:
	docker build -t eduart-swarm-ipc127e:0.1.0 .

all: download-sources build-docker
	echo "build edu_swarm into a docker image"

clean:
	rm -rf edu_swarm
	rm -rf edu_robot
