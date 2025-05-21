# voxl_mast3r

`voxl_mast3r` is a ROS 2 package designed for VOXL-based robotics systems to intelligently capture UAV images optimized for 3D reconstruction using [MASt3R](https://github.com/naver/MASt3R). This package supports both feature-based and telemetry-driven image selection to reduce redundancy and improve model quality.

## ✨ Features

- ROS 2-compatible nodes for use with VOXL hardware
- Image sampling based on:
  - Visual similarity using feature matching
  - Drone telemetry (position and orientation)
- Tools for system diagnostics and performance monitoring
- Seamless integration with MASt3R reconstruction pipeline

## 🧩 Requirements

- ROS 2 (tested with Humble)
- VOXL SDK and hardware
- Docker
- C++14 or later
- `px4_msgs` ROS 2 package (v1.14)

## 🐳 Installation (Docker)

The recommended way to run this package is inside a Docker container.

### Build the Docker Image

```bash
cd voxl_mast3r/docker
bash build.sh
```

### Run the Docker Container

To start the docker container, run the following bash file
```bash
bash run.sh
```
You will enter a Docker container with all dependencies pre-installed.

## Usage

### Capturing image based on Features
To use the ORB + BF based image sampler, run the following command
```bash
ros2 run voxl_mast3r capture_image.py
``` 
### Capturing image based on drone telemetry
This approach saves an image only when the UAV moves a specified distance or changes yaw beyond a threshold
```bash
ros2 run voxl_mast3r voxl_mast3r
```
Images will be saved sequentially in the /images folder inside the container. Ensure the camera topic /hires_small_color is active and publishing.

### ☁️ Cloud Integration (3D Model Generation on AWS)
We used AWS EC2 cloud server to render the 3D model on the cloud directly. To run it on the cloud. Makes sure that you have installed my [mast3r-3d-model-generation](https://github.com/koustubh1012/mast3r-3d-model-generation) repository. Once this is done, modify the .pem file and uppload_images_and_build.sh file accordingly. After the required images have been captured, run the following command

```bash
uppload_images_and_build.sh
```

## Contributing

Contributions are welcome! Please open issues or submit pull requests.

## License

This project is licensed under the MIT License.

## Authors

- Koustubh (maintainer)
