//ros-grabber_pub.cpp (pub version)

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <vector>

#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorDataMatrixCode.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#include <my_topics/QrLabelInfo.h> 


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<my_topics::QrLabelInfo>("/publisher_topic", 10);
    ros::Rate loop_rate(10);

    my_topics::QrLabelInfo qr_data; //создание обьекта класса my_topics::QrLabelInfo с именем qr_data         
    
    std::cout << "visp_app ros-grabber_pub" << std::endl;
    int opt_barcode = 0; 
    bool opt_use_camera_info;
    vpImage<unsigned char> I; // Create a color image container

    vpROSGrabber g; // Create a grabber based on ROS
    g.setImageTopic("/simple_cam/camera1/image_raw");//! [Setting camera topic]
    if (opt_use_camera_info) 
    {
      g.setCameraInfoTopic("/simple_cam/camera1/camera_info");//! [Setting camera info]
      g.setRectify(true);
    }
    g.open(I);//! [Opening]
    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;
    vpDisplayX dm(I);


  //barcode detector process
  vpDetectorBase *detector = NULL;
  detector = new vpDetectorQRCode;

  while(1){ //цикл прорисовки и распознавания
    g.acquire(I);

    vpDisplay::display(I);
    bool status = detector->detect(I);

    std::ostringstream legend;
    legend << detector->getNbObjects() << " bar code detected";
    vpDisplay::displayText(I, 10, 10, legend.str(), vpColor::red);


    if (status) {
      for (size_t i = 0; i < detector->getNbObjects(); i++) {
        std::vector<vpImagePoint> p = detector->getPolygon(i);
        vpRect bbox = detector->getBBox(i);


        vpDisplay::displayRectangle(I, bbox, vpColor::green);// прорисовка квадрата распознавания вокруг углов маркера
        std::cout << std::endl << "bbox= " << bbox << std::endl;
        std::cout << "bbox.getTop= " << bbox.getTop() << std::endl;
        std::cout << "bbox.getLeft= " << bbox.getLeft() << std::endl;
        std::cout << "bbox.getWidth= " << bbox.getWidth() << std::endl; 
        std::cout << "bbox.getHeight= " << bbox.getHeight() << std::endl; 

          
        // прорисовка сообщения, прикрепленная к верхнему левому углу маркера
        vpDisplay::displayText(I, (int)bbox.getTop() - 20, (int)bbox.getLeft(), "Message: \"" + detector->getMessage(i) + "\"", vpColor::red);

        qr_data.text = detector->getMessage(i); //запись считанного сообщения в qr_data.text
        std::cout << "Message qr_data: " << qr_data.text << std::endl; // вывод считанного сообщения в терминал

        std::cout << "координаты центра bbox.getCenter = "<< bbox.getCenter() << std::endl; // вывод координат центра маркера
        vpDisplay::displayCross(I, bbox.getCenter(), 14, vpColor::green, 3);// прорисовка креста в центре маркера
        vpDisplay::displayText(I, bbox.getCenter() + vpImagePoint(12, 5), "Center", vpColor::green); //прорисовка текста в центре маркера
    
        vpImagePoint pc = bbox.getCenter(); //запись координат центра QR кода в qr_data
        qr_data.center.x = pc.get_i(); 
        qr_data.center.y = pc.get_j();


        std::cout << "координаты вершин QR кода " << std::endl; // вывод координат вершин QR кода
        for (size_t j = 0; j < p.size(); j++) {
          vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);// прорисовка крестов в углах маркера
          std::ostringstream number;
          number << j;
          vpDisplay::displayText(I, p[j] + vpImagePoint(12, 5), number.str(), vpColor::red); //прорисовка номеров вершин маркера
          std::cout << "p[" << j << "]= " << p[j] << std::endl; //вывод в терминал координат вершин

        }



      pub.publish(qr_data); //отправка данных в топик
      std::cout << "qr_data  published!" << std::endl;
      vpDisplay::displayText(I, (int)bbox.getTop() - 10, (int)bbox.getLeft(), "qr_data  published!", vpColor::yellow);
      ros::spinOnce();
      loop_rate.sleep();



      }
    }



    
    vpDisplay::displayText(I, (int)I.getHeight() - 25, 10, "Click to quit...", vpColor::red);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false)) // a click to exit
    break;
  }// end VISP



  return 0; //end programm
}

