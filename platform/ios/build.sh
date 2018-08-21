#!/bin/sh

if [ $# -eq 0 ]
 then
  CONFIG="Release"
elif [ $1 = "Release" -o $1 = "Debug" ]
 then
  CONFIG=$1
else
  echo "unknown configuration" $1
  exit 1
fi

xcodebuild -configuration $CONFIG -workspace bleloc.xcworkspace -scheme bleloc -sdk iphoneos;
xcodebuild -configuration $CONFIG -workspace bleloc.xcworkspace -scheme bleloc -sdk iphonesimulator;
xcodebuild -configuration $CONFIG -workspace bleloc.xcworkspace -scheme framework;
xcodebuild -configuration $CONFIG -workspace bleloc.xcworkspace -scheme bleloc -sdk iphoneos;
