#!/bin/bash
IMAGES_DIR=/home/user/images
ZIP_NAME=images_$(date +%Y%m%d_%H%M%S).tar.gz
EC2_USER=ubuntu
EC2_IP=54.227.65.94

chmod 400 aerial-rob.pem

echo "[+] Zipping images..."
tar -czvf $ZIP_NAME -C $IMAGES_DIR .

echo "[+] Uploading to EC2..."
scp -i aerial-rob.pem $ZIP_NAME $EC2_USER@$EC2_IP:/home/ubuntu/

echo "[+] Triggering Mast3R script on EC2..."
ssh -i aerial-rob.pem $EC2_USER@$EC2_IP "bash /home/ubuntu/trigger_mast3r.sh $ZIP_NAME"   
