# rosemat
Rosbag Semantic Annotation Tool for Matlab

Documentation in [wiki](https://github.com/rtylecek/rosemat/wiki).

![UI](https://raw.githubusercontent.com/rtylecek/rosemat/master/rosemat-ui.png)

## Main Features

* Pixelwise semantic annotation with configrable set of classes
* Guide 2D annotation of captured images using 3D model of the scene
* Multiple camera topics from bagfile
* Camera calibration from Kalibr
* GT poses from bagfiles or external
* Project point cloud or mesh model with semantics
* Optical flow plug-in for next frame transfer
* Export to various formats

## Author

Radim Tylecek, University of Edinburgh 

rtylecek@inf.ed.ac.uk

If you use our tool, please cite our [paper](http://www.mdpi.com/1424-8220/18/7/2249)

```
@Article{s18072249,
  AUTHOR = {Tylecek, Radim and Fisher, Robert  B.},
  TITLE = {Consistent Semantic Annotation of Outdoor Datasets via 2D/3D Label Transfer},
  JOURNAL = {Sensors},
  VOLUME = {18},
  YEAR = {2018},
  NUMBER = {7},
  ARTICLE NUMBER = {2249},
  URL = {http://www.mdpi.com/1424-8220/18/7/2249},
  ISSN = {1424-8220},DOI = {10.3390/s18072249}
}
```


[![HitCount](http://hits.dwyl.io/rtylecek/rosemat.svg)](http://hits.dwyl.io/rtylecek/rosemat)

![ROS](http://www.ros.org/wp-content/uploads/2013/10/rosorg-logo1.png)
![Matlab](https://upload.wikimedia.org/wikipedia/commons/thumb/2/21/Matlab_Logo.png/267px-Matlab_Logo.png)
