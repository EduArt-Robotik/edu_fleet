image_name = eduart-fleet:0.3.2

download-sources:
	git clone git@github.com:EduArt-Robotik/edu_fleet.git
	git clone --branch v6.0.0 https://github.com/SICKAG/sick_lidar_localization.git	
	git clone https://gitlab.com/accerion/accerion-ros2.git

build-docker:
	docker build -t $(image_name) .

upload-docker:
	docker tag $(image_name) eduartrobotik/$(image_name)
	docker push eduartrobotik/$(image_name)

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

all: download-sources build-docker
	$(info Creating EduArt's robot library Docker image "$(image_name)".)

clean:
	rm -rf edu_swarm
	rm -rf sick_lidar_localization
