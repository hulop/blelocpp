Pod::Spec.new do |s|
    s.name             = 'OpenCV-Dynamic'
    s.version          = '4.1.2'
    s.summary          = 'OpenCV (Computer Vision) for iOS as a dynamic framework.'

    s.description      = <<-DESC
OpenCV: open source computer vision library
    Homepage: http://opencv.org
    Online docs: http://docs.opencv.org
    Q&A forum: http://answers.opencv.org
    Dev zone: http://code.opencv.org
    DESC

    s.homepage         = 'https://github.com/opencv/opencv'
    s.license          = { :type => '3-clause BSD', :file => 'LICENSE' }
    s.authors          = 'opencv.org'
    s.source           = { :git => 'https://github.com/opencv/opencv.git', :tag => s.version.to_s }

    s.ios.deployment_target = "8.0"
    s.source_files = "opencv2.framework/Headers/**/*{.h,.hpp}"
    #s.public_header_files = "opencv2.framework/Headers/**/*{.h,.hpp}"
    s.preserve_paths = "opencv2.framework"
    #s.header_dir = "opencv2"
    #s.header_mappings_dir = "opencv2.framework/Versions/A/Headers/"
    s.vendored_frameworks = "opencv2.framework"
    s.requires_arc = false
    s.libraries = [ 'stdc++' ]
    s.frameworks = [
        "Accelerate",
        "AssetsLibrary",
        "AVFoundation",
        "CoreGraphics",
        "CoreImage",
        "CoreMedia",
        "CoreVideo",
        "Foundation",
        "QuartzCore",
        "UIKit"
    ]

    s.prepare_command = <<-CMD
        /usr/bin/python ./platforms/ios/build_framework.py ios --dynamic
        cp -a ./ios/opencv2.framework ./opencv2.framework
        install_name_tool -id "@rpath/opencv2.framework/opencv2" ./opencv2.framework/opencv2
    CMD
end
