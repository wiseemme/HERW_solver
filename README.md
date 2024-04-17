# 2023-kinematics-cal-xarm
This is the repo for XARM DH parameter calibration algorithm 

## The environment is setup using Docker 
Below are the steps for setting up the docker environment: 

```
docker pull thejerrycheng/kinematics_cal_xarm 

docker run -it thejerrycheng/kinematics_cal_xarm

```

Command used to add a local directory to the docker image: 
``` 
docker run -it --rm -v $HOME/Desktop/kinematics_cal_xarm:/source_code --name source_code thejerrycheng/kinematics_cal_xarm bash


```
