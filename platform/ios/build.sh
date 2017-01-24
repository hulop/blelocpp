#!/bin/sh
xcodebuild -workspace bleloc.xcworkspace -scheme bleloc -sdk iphoneos;
xcodebuild -workspace bleloc.xcworkspace -scheme bleloc -sdk iphonesimulator;
xcodebuild -workspace bleloc.xcworkspace -scheme framework;
xcodebuild -workspace bleloc.xcworkspace -scheme bleloc -sdk iphoneos;
