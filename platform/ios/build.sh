#!/bin/sh

if [ $# -eq 0 ]
then
    CONFIG="Release"
elif [ $1 = "Release" -o $1 = "Debug" ]
then
    CONFIG=$1
elif [ $1 = "clean" ]
then
    xcodebuild clean -workspace bleloc.xcworkspace -scheme bleloc -sdk iphonesimulator
    xcodebuild clean -workspace bleloc.xcworkspace -scheme bleloc -sdk iphoneos
    rm -rf archives
    rm -rf bleloc.xcframework
    exit 1
else
    echo "unknown configuration" $1
    exit 1
fi

CONFIG="$CONFIG -workspace bleloc.xcworkspace -scheme bleloc SKIP_INSTALL=NO BUILD_LIBRARY_FOR_DISTRIBUTION=YES only_active_arch=no defines_module=yes -quiet"

xcodebuild archive -configuration $CONFIG -sdk iphonesimulator -arch arm64 -arch x86_64 -archivePath "archives/bleloc-iOS-Simulator" && \
xcodebuild archive -configuration $CONFIG -sdk iphoneos        -arch arm64 -arch armv7  -archivePath "archives/bleloc-iOS" #&& \
xcodebuild -create-xcframework \
	   -framework "./archives/bleloc-iOS-Simulator.xcarchive/Products/Library/Frameworks/bleloc.framework" \
	   -framework "./archives/bleloc-iOS.xcarchive/Products/Library/Frameworks/bleloc.framework" \
	   -output "./bleloc.xcframework"

zip -r bleloc.xcframework.zip bleloc.xcframework
