#include <iostream>
#include "opencv2/opencv.hpp"

#include "cmake_tutorial_3/functions.hpp"

int main(int argc, char * argv[])
{
  if (argc != 2) {
    printf("usage: DisplayImage.out <Image_Path>\n");
    return -1;
  }

  cv::Mat image;
  image = cv::imread(argv[1], cv::IMREAD_COLOR);

  cmake_tutorial_3::show_image(image);

  std::cout << "El doble de 2 es " << cmake_tutorial_3::duplica(2.0) << std::endl;

  return 0;
}
