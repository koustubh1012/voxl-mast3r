#!/bin/bash
IMAGES_DIR=/home/user/images
ZIP_NAME=images_$(date +%Y%m%d_%H%M%S).tar.gz
EC2_USER=ubuntu
EC2_IP=xx.xxx.xxx.xxx # Replace with your EC2 instance's public IP

chmod 400 aerial-rob.pem # Generate your own key pair and replace this with the path to your key

echo "[+] Zipping images..."
tar -czvf $ZIP_NAME -C $IMAGES_DIR .

echo "[+] Uploading to EC2..."
scp -i aerial-rob.pem $ZIP_NAME $EC2_USER@$EC2_IP:/home/ubuntu/

echo "[+] Triggering Mast3R script on EC2..."
ssh -i aerial-rob.pem $EC2_USER@$EC2_IP "bash /home/ubuntu/trigger_mast3r.sh $ZIP_NAME"   
