#!/bin/bash
yolo_tf_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/head_pose_picture/yolo_model.om"

wget -nc --no-check-certificate ${yolo_tf_model} -O ../model/face_detection.om