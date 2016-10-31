/*******************************************************************************
 * Copyright (c) 2014, 2016  IBM Corporation and others
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#import <XCTest/XCTest.h>
#import "Location.hpp"
#import "State.hpp"

using namespace loc;
using namespace std;

@interface BasicLocalizerTest : XCTestCase

@end

@implementation BasicLocalizerTest

- (void)setUp {
    [super setUp];
    // Put setup code here. This method is called before the invocation of each test method in the class.
}

- (void)tearDown {
    // Put teardown code here. This method is called after the invocation of each test method in the class.
    [super tearDown];
}

- (void)testExample {
    Anchor a;
    a.latlng.lat = 35.67881222;
    a.latlng.lng = 139.7870817;
    a.rotate = -152;
    Location l(10, 20, 0, 9);
    l.anchor(a);
    
    State s(l);
    State s2 = s;

    XCTAssertTrue(s.lat() == s.lat());
    XCTAssertTrue(s2.lng() == s2.lng());
    
    printf("%.2f, %.2f\n", s.lat(), s.lng());
    
    
    LatLng ll;
    ll.lat = l.lat();
    ll.lng = l.lng();
    Location l2(0, 0, 0, 9);
    l2.anchor(a);
    l2.latlng(ll);
    
    printf("%.10f, %.10f\n", fabs(l2.x()-l.x()), fabs(l2.y()-l.y()));
    
    XCTAssertTrue(fabs(l2.x()-l.x())<10e-6);
    XCTAssertTrue(fabs(l2.y()-l.y())<10e-6);
}

- (void)testPNG {
}

@end
