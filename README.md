[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]

# YOLOv5 ROS package

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#install-dependencies">Install dependencies</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

## About The Project

This project was created for the 2nd Smart Cities Robotics Challenge (SciRoc) 2021. One of our team's tasks was detecting people and objects via a mounted camera on the robot. Thus creating a ros package that utilizes YOLOv5.

<!-- GETTING STARTED -->

## Getting Started

In order to use this package follow the next steps.

### Prerequisites

Yolov5 uses python 3.8. Check your python version using

```
python -V
```

To install python 3.8 go [here](https://tech.serhatteker.com/post/2019-12/upgrade-python38-on-ubuntu/). The installation of the dependencied is done using [anaconda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html). Installing dependencies with **pip** is also tested and working.

### Install dependencies

The installation of the dependencies is done using [anaconda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html).
Installing dependencies with **pip** is also tested and working.

### Install package dependencies (anaconda)

1. Create a new virtual environment named venv

```
conda create -n venv python=3.8 jupyter
```

2. Activate new virtual environment

```
conda activate venv
```

3. Install [pytorch](https://pytorch.org/get-started/locally/). Insert your specifications for os (windows/linux), your package manager (e.g. conda/pip etc) and Compute Platform (cpu/gpu etc). Copy the installation command next to Run this command to your terminal and run it.
4. Install other dependencies

```
sudo apt-get install python3-pip python3-yaml
pip3 install rospkg catkin_pkg
cd /path/to/ros_yolov5
pip3 install -r requirements.txt
```

### Choose python interpreter

Modify the shebang at the top of ros_yolov5.py file at `ros_yolov5/scripts/ros_yolov5.py` and `ros_yolov5/scripts/process_detections.py`

Example

Default:

```
#!/path/to/venv/bin/python3
```

Modified:

```
#!/home/ronnie_coleman/venv/bin/python
```

or

```
#!/home/ronnie_coleman/miniconda3/envs/venv/bin/python
```

## Installation

Clone the repository to your workspace.

```
mkdir -p "yolo_ws/src" && cd $_
git clone https://github.com/uniwa-gravastars/yolov5-rospackage.git
```

Then compile the package and source the workspace

```
cd ..
catkin_make
```

## Usage

To change the weights that Yolov5 uses, open `ros_yolov5.launch` on `ros_yolov5/launch/ros_yolov5.launch` and change the value of the parameter weights_path to point to your pt file. Moreover change the `source_topic` to the correct camera topic.

The YOLOv5 ros package is now ready. Open a terminal, source the workspace and run:

```
roslaunch ros_yolov5 ros_yolov5.launch
```

## Usual errors

### Resize error

!!ATTENTION!!

If you come across this error:

```
[ERROR] [1661788501.044783]: bad callback: <bound method ObjectDetector.image_callback of <__main__.ObjectDetector object at 0x7f011fe2bdc0>>
Traceback (most recent call last):
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/user/catkin_ws/src/yolov5-rospackage/ros_yolov5/scripts/ros_yolov5.py", line 46, in image_callback
    results = self.model(img, size=640)
  File "/home/user/miniconda3/envs/venv/lib/python3.8/site-packages/torch/nn/modules/module.py", line 1110, in _call_impl
    return forward_call(*input, **kwargs)
  File "/home/user/miniconda3/envs/venv/lib/python3.8/site-packages/torch/autograd/grad_mode.py", line 27, in decorate_context
    return func(*args, **kwargs)
  File "/home/user/catkin_ws/src/yolov5-rospackage/ros_yolov5/yolov5/models/common.py", line 278, in forward
    y = self.model(x, augment, profile)[0]  # forward
  File "/home/user/miniconda3/envs/venv/lib/python3.8/site-packages/torch/nn/modules/module.py", line 1110, in _call_impl
    return forward_call(*input, **kwargs)
  File "/home/user/catkin_ws/src/yolov5-rospackage/ros_yolov5/yolov5/models/yolo.py", line 122, in forward
    return self.forward_once(x, profile, visualize)  # single-scale inference, train
  File "/home/user/catkin_ws/src/yolov5-rospackage/ros_yolov5/yolov5/models/yolo.py", line 153, in forward_once
    x = m(x)  # run
  File "/home/user/miniconda3/envs/venv/lib/python3.8/site-packages/torch/nn/modules/module.py", line 1110, in _call_impl
    return forward_call(*input, **kwargs)
  File "/home/user/miniconda3/envs/venv/lib/python3.8/site-packages/torch/nn/modules/upsampling.py", line 154, in forward
    recompute_scale_factor=self.recompute_scale_factor)
  File "/home/user/miniconda3/envs/venv/lib/python3.8/site-packages/torch/nn/modules/module.py", line 1185, in __getattr__
    raise AttributeError("'{}' object has no attribute '{}'".format(
AttributeError: 'Upsample' object has no attribute 'recompute_scale_factor'
```

This error is from the internal code of pytorch. The "forward" function is not working properly.

To fix this, replace in `miniconda3/envs/venv/lib/python3.8/site-packages/torch/nn/modules/upsampling.py`:

```
def forward(self, input: Tensor) -> Tensor:
        return F.interpolate(input, self.size, self.scale_factor, self.mode, self.align_corners,
                         recompute_scale_factor=self.recompute_scale_factor)
```

with:

```
def forward(self, input: Tensor) -> Tensor:
        return F.interpolate(input, self.size, self.scale_factor, self.mode, self.align_corners)
```

Find fix [here](https://github.com/openai/DALL-E/issues/54#issuecomment-1092826376).

<!-- CONTRIBUTING -->

## Contributing

Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with an appropriate tag.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<!-- LICENSE -->

## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

## Acknowledgments

Special thanks to the original author of this work Kaloterakis Evangelos.

<a href="#top">(Back to top)</a>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

[contributors-url]: https://github.com/uniwa-gravastars/yolov5-rospackage/graphs/contributors
[forks-url]: https://github.com/uniwa-gravastars/yolov5-rospackage/network/members
[stars-url]: https://github.com/uniwa-gravastars/yolov5-rospackage/stargazers
[issues-url]: https://github.com/uniwa-gravastars/yolov5-rospackage/issues
[license-url]: https://github.com/uniwa-gravastars/yolov5-rospackage/master/LICENSE.txt
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge

<!-- [product-screenshot]: images/screenshot.png -->
