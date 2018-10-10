
#include <ros.h>

#include <exo_ard/loadcell.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

exo_ard::loadcell sg;


ros::Publisher chatter("chatter", &sg);

float i= 0;
float x;
void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  x= i*2+0.0;
  sg.ld1=x;
  x= i*3+0.0;
  sg.ld2=x;
  x= i*4+0.0;
  sg.ld3=x;
  x= i*5+0.0;
  sg.ld4=x;


  chatter.publish( &sg );
  i += 1;
  nh.spinOnce();
  delay(1);
}
