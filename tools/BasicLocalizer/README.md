## A command line tool to run bleloc

## Prerequisites
- [cereal version 1.1.2](http://uscilab.github.io/cereal/) (BSD License)
- [picojson version 1.3.0](https://github.com/kazuho/picojson) (BSD License)
- [Boost version 1.61.0](http://www.boost.org) (Boost Software License)
- [Eigen version 3.2.5 (EIGEN_MPL2_ONLY)](http://eigen.tuxfamily.org) (Mozilla Public License Version 2.0)
- [OpenCV version 3.2](http://opencv.org/) (BSD License)

## Build instructions
### Install CocoaPods (If you have not installed)
- Install and update Homebrew
- $brew install python
- $pip install mercurial
- Install CocoaPods

### Build BasicLocalizer binary
- $git clone https://github.com/hulop/blelocpp.git
- $cd blelocpp/tools/BasicLocalizer
- $pod install
- $sh build.sh

## Basic usage

- Type the following command to see command line options
```
$ ./BasicLocalizer.exec
```

- Example usege
```
$ ./BasicLocalizer.exec -m MODEL_FILE.json - t TEST_FILE.log -o OUTPUT_FILE.csv
```
  * MODEL_FILE: a localization model file created from training data
  * TEST_FILE: a log file containing recorded beacon and sensor data (See next section)
  * OUTPUT_FILE: a csv file containing the localization results (timestamp, latitude, longitude, etc.)

  This command runs beacon-based localization with time averaging. See the command line options and code for more detail.

### Test file format

```
yyyy-MM-dd HH:mm:ss.SSSZZZ TAG[pid:tid] DATA_FIELD
...
...
...
```

Only DATA_FIELD is used in the localization algorithm. The first three fields do not affect the localization results.

- Example
``` example.log
0000-00-00 00:00:00 Test Acc,0,0,0,1500000000
0000-00-00 00:00:00 Test Motion,0,0,0,1500000010
0000-00-00 00:00:00 Test Heading,0,0,15,0,0,0,1500000020
0000-00-00 00:00:00 Test Altimeter,0,101.325,1500000030
0000-00-00 00:00:00 Test Beacon,3,1,1,-70,1,2,-75,1,3,-80,1500000040
...
```

* Specification of DATA_FIELD
  * Beacon (required)
    * New format
    ```
    "Beacon",number_of_beacons(N),uuid1-major1-minor1,rss1,...,uuidN-majorN-minorN,rssN,timestamp
    ```
    * Legacy format
    ```
    "Beacon",number_of_beacons(N),major1,minor1,rss1,...,majorN,minorN,rssN,timestamp
    ```
  * Heading (required)
  ```
   "Heading",magneticHeading,trueHeading,headingAccuracy,mx,my,mz,timestamp
  ```
  * Acceleration (optional)
  ```
   "Acc",acceleration_x,acceleration_y,acceleration_z,timestamp
  ```
  * Motion (optional)
  ```
   "Motion",pitch,roll,yaw,timestamp
  ```
  * Altitude (optional)
  ```
   "Altimeter",relativeAltitude,pressure,timestamp
  ```
