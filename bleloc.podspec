Pod::Spec.new do |s|
  s.name         = "bleloc"
  s.version      = "1.0"
  s.summary      = "A short description of bleloc."
  s.homepage     = "https://github.com/hulop/"
  s.description  = <<-DESC
This is a localization library for bluetooth le beacons.
                   DESC

  s.license      = "MIT"
  s.author    = "HULOP"
  s.source       = { :git => "https://github.com/hulop/blelocpp.git", :tag => "v1.0" }
  s.platforms = {:ios => "8.4"}
  s.preserve_path = "platform/ios/bleloc.framework"
  s.vendored_frameworks = "platform/ios/bleloc.framework"
  s.requires_arc = false
end
