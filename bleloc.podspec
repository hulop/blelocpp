Pod::Spec.new do |s|
  s.name         = "bleloc"
  s.version      = "1.3.6"
  s.summary      = "Localization library"
  s.homepage     = "https://github.com/hulop/"
  s.description  = <<-DESC
This is a localization library for bluetooth le beacons.
                   DESC

  s.license      = "MIT"
  s.author    = "HULOP"
#  s.source       = { :git => "https://github.com/hulop/blelocpp.git", :tag => "v1.3.6" }
#  s.preserve_path = "platform/ios/bleloc.framework"
#  s.vendored_frameworks = "platform/ios/bleloc.framework"
  s.source       = { :http => "https://github.com/hulop/blelocpp/releases/download/v1.3.6/bleloc.framework.zip" }
  s.preserve_path = "bleloc.framework"
  s.vendored_frameworks = "bleloc.framework"
  s.platforms = {:ios => "8.4"}
  s.requires_arc = false
  s.ios.frameworks = 'AVFoundation', 'AssetsLibrary', 'CoreMedia'
end
