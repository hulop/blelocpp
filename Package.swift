// swift-tools-version:5.3
import PackageDescription

let version = "1.4.0-alpha"

let package = Package(
    name: "bleloc",
    platforms: [
      .iOS(.v9)
    ],
    products: [
        .library(
            name: "bleloc",
            targets: ["bleloc"])
    ],
    dependencies: [
    ],
    targets: [
        .binaryTarget(
            name: "bleloc",
            url: "https://github.com/hulop/blelocpp/releases/download/v1.4.0-alpha/bleloc.xcframework.zip",
            checksum: "33a235a178aa884fbb919add1fd841df22748a8d03337f3f2a8254c16aec1986"
        )
    ]
)
