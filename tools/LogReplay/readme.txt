Required softwares
- homebrew http://brew.sh
- cocoapods https://cocoapods.org

1. install opencv by homebrew
$ brew tap homebrew/science
$ brew install opencv
It will install opencv 2.4.12

2. install pods
$ pod install

3. open LogReplay.xcworkspace

4. run with example files (example command line argument is set in edit scheme view)
* You need to set working directory in options of edit schemes view.
* Set it to the root of LogReplay dir (repository/tools/LogReplay) containing example files.

$LogReplay -sft train.txt -b beacons.csv -m map.png -l navcog.log -1 9,60 -o out.txt
* see "LogReplay -h" for more information about options 

5. you can find LogPlay executable file (in Products group) to process your data
