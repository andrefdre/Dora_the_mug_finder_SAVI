<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/github_username/repo_name">
    <img src="Docs/logo.svg" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">Dora the Mug Finder</h3>

  <p align="center">
    Consists on the implementation of a deep neural network to classify objects collected from 3D models or RGB-D cameras
    <br />
    <a href="https://github.com/github_username/repo_name"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/github_username/repo_name">View Demo</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues">Report Bug</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project
<div align="center">
<img  src="Docs/logo.svg" alt="Logo" width="400">
</div>

This project was developed for Advanced Industrial Vision Systems class for the second report. The objective is to detect and extract objects from a point cloud and then pass it through a classifier that will tell what the object is and some information about it's physical characteristics. For example, it's a Mug and has a certain bounding box.  

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ### Built With

* [![Next][Next.js]][Next-url]
* [![React][React.js]][React-url]
* [![Vue][Vue.js]][Vue-url]
* [![Angular][Angular.io]][Angular-url]
* [![Svelte][Svelte.dev]][Svelte-url]
* [![Laravel][Laravel.com]][Laravel-url]
* [![Bootstrap][Bootstrap.com]][Bootstrap-url]
* [![JQuery][JQuery.com]][JQuery-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->



<!-- GETTING STARTED -->
## Getting Started

This project uses [ROS Noetic](http://wiki.ros.org/ROS/Installation) to aid the use of the kinect camera and the object classification CNN is built based on [PyTorch](https://pytorch.org/). To do object detection is used [Open3D](http://www.open3d.org/).

### Prerequisites

To use this code, first add the next line in the bashrc or your shell configuration file:

  ```
#Dora The Mug FInder
export DORA=/home/andre/dora_the_mug_finder
export PYTHONPATH="$PYTHONPATH:${HOME}/catkin_ws/src/Dora_the_mug_finder_SAVI"
  ```
Replace the path of DORA to where all the datasets are stored in your computer. You can download them [here](rgbd-dataset.cs.washington.edu/dataset/rgbd-scenes-v2/0).

Afterward, update the shell with the new configuration using:
```
source ~/.bashrc
```
If you use zsh, just change to *.zshrc*.

Inside DORA folder, there should be a structure similar to:
  - models
  - rgbd-dataset
  - rgbd-scenes-v2
    - pc
    - imgs
  - rosbag



### Installation
To install the project, clone the repository inside the *src* folder of your *catkin_ws*, running the following lines:
```
git clone https://github.com/andrefdre/Dora_the_mug_finder_SAVI.git
cd ..
catkin_make
```

To install all the dependencies of this package, just run in your terminal:
```
pip install -r /full/path/to/Dora_the_mug_finder_SAVI/requirements.txt
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE EXAMPLES -->
## Usage

To Train the model, run the code:

```
rosrun dora_the_mug_finder_bringup model_train.py -fn <folder_name> -mn <model_name> -n_epochs 50 -batch_size 256 -c 0
```

Where the *<folder_name>* and *<model_name>*  should be replaced by the names you want to give. 

If instead of using point clouds from the dataset, you want to use point clouds acquired from a Kinect stored in a rosbag, before running the detector just run:
```
roslaunch dora_the_mug_finder_bringup bringup_bag.launch 
```

To run the detector with previous trained model run the code:
```
roslaunch dora_the_mug_finder_bringup bringup_dora.launch mn:=<model_name> fn:=<folder_name>
```

If you want to visualize extracted images run:
```
roslaunch dora_the_mug_finder_bringup bringup_dora.launch mn:=<model_name> fn:=<folder_name> visualize:=True
```
Where the <folder_name> and <model_name> should be replaced by a name for the model previously set while training. 
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the GPL License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

André Cardoso - andref@ua.pt

Tatiana Resende - tatianaresende@ua.pt

Fábio Sousa - fabiorsousa81@ua.pt

Project Link: [Dora the Mug Finder](https://github.com/andrefdre/Dora_the_mug_finder_SAVI)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* Professor Miguel Oliveira - mriem@ua.pt

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/andrefdre/Dora_the_mug_finder_SAVI.svg?style=for-the-badge
[contributors-url]: https://github.com/andrefdre/Dora_the_mug_finder_SAVI/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/andrefdre/Dora_the_mug_finder_SAVI.svg?style=for-the-badge
[forks-url]: https://github.com/andrefdre/Dora_the_mug_finder_SAVI/network/members
[stars-shield]: https://img.shields.io/github/stars/andrefdre/Dora_the_mug_finder_SAVI.svg?style=for-the-badge
[stars-url]: https://github.com/andrefdre/Dora_the_mug_finder_SAVI/stargazers
[issues-shield]: https://img.shields.io/github/issues/andrefdre/Dora_the_mug_finder_SAVI.svg?style=for-the-badge
[issues-url]: https://github.com/andrefdre/Dora_the_mug_finder_SAVI/issues
[license-shield]: https://img.shields.io/github/license/andrefdre/Dora_the_mug_finder_SAVI.svg?style=for-the-badge
[license-url]: https://github.com/andrefdre/Dora_the_mug_finder_SAVI/blob/master/LICENSE.txt
[product-screenshot]: Docs/logo.svg
<!-- [Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com  -->
