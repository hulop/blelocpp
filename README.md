# BLE localization library
This is a BLE beacon-based localization library.
It is necessary to build this library as an iOS Framework before building NavCog2.

## Prerequisites
- [cereal version 1.1.2](http://uscilab.github.io/cereal/) (BSD License)
- [picojson version 1.3.0](https://github.com/kazuho/picojson) (BSD License)
- [Boost version 1.57.0](http://www.boost.org) (Boost Software License)
- [Eigen version 3.2.5](http://eigen.tuxfamily.org) (Mozilla Public License Version 2.0)
- [OpenCV version 2.4.9](http://opencv.org/) (BSD License)

## Build instructions
### Install CocoaPods (If you have not installed)
- Install and update Homebrew
- $brew install python
- $pip install mercurial
- Install CocoaPods

### Build bleloc Framework
- $git clone https://github.com/hulop/blelocpp.git
- $cd blelocpp/platform/ios
- $pod install
- Open the Xcode workspace by typing $open bleloc.xcworkspace
- Product > Clean
- Build bleloc target for Generic iOS device
- Build framework target for Generic iOS device
- Build bleloc target for Generic iOS device
