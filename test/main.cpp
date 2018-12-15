/** MIT License

 Copyright (c) 2018 Sarthak Mahajan

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

/**
 *@copyright Copyright (c) 2018 Sarthak Mahajan
 *@file main.cpp
 *@author Sarthak Mahajan
 *@brief Creates a node for the tests and calls all tests
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

/**
 * @brief Main function for testing
 *
 * @param argc  The argc as int
 * @param argv  The argv as char array
 *
 * @return 0 if everything passed
 */
int main(int argc, char **argv) {
  // create  node for the tests
  ros::init(argc, argv, "frontier_test");
  testing::InitGoogleTest(&argc, argv);
  // return the value of the flag, for RUN_ALL_TESTS()
  return RUN_ALL_TESTS();
}
