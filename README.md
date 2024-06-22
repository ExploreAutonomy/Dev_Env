# Dev_Env
How to setup dev env 



## Create a docker container with zerotier support from an image

https://docs.zerotier.com/docker/

docker run -it --cap-add=NET_ADMIN --cap-add=SYS_ADMIN --device=/dev/net/tun --name ardupilot_px4_dds_mavros 43bac3283bb3311af28efacde6f5cd3f88f167723cc6e49923ab07f0604c70a9 bash
