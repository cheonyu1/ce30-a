#include <ros/ros.h>
#include <unistd.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <error.h>
#include <net/if.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ce30a_node");
  ros::NodeHandle nh;

  int s;
  struct sockaddr_can addr;
  const char *ifname = "can0";
  struct ifreq ifr;
  socklen_t len = sizeof(addr);
  struct can_frame frame;

  if(s = socket(PF_CAN, SOCK_RAW, CAN_RAW) < 0)
  {
    cout << "socket open failed. " << endl;
    return -1;
  }

  strcpy(ifr.ifr_name, ifname);
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  cout << ifname << " at index " << ifr.ifr_ifindex << endl;

  if(bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    cout << "socket bind failed. " << endl;
    return -2;
  }

  struct timeval timeout;      
  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;

  if (setsockopt (s, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
  {
    cout << "setsockopt failed" << endl;
    return -3;
  }

  frame.can_id  = 0x606;
  frame.can_dlc = 5;
  frame.data[0] = 0xc1;
  frame.data[1] = 0x00;
  frame.data[2] = 0x23;
  frame.data[3] = 0x28;
  frame.data[4] = 0x00;

  int n = -1;
  
  while(n < 0)
    n = write(s, &frame, sizeof(struct can_frame));

  cout<< showbase << setfill('0') << hex << right << internal;

  if(n > 0)
    cout
      << "frame.can_id  : " << setw(6) << (int)frame.can_id << endl
      << "frame.can_dlc : " << setw(4) << (int)frame.can_dlc << endl
      << "frame.__pad   : " << setw(4) << (int)frame.__pad << endl
      << "frame.__res0  : " << setw(4) << (int)frame.__res0 << endl
      << "frame.__res1  : " << setw(4) << (int)frame.__res1 << endl
      << setw(4) << (int)frame.data[0] << ", " << setw(4) << (int)frame.data[1] << ", " << setw(4) << (int)frame.data[2] << ", " << setw(4) << (int)frame.data[3] << ", " << endl
      << setw(4) << (int)frame.data[4] << ", " << setw(4) << (int)frame.data[5] << ", " << setw(4) << (int)frame.data[6] << ", " << setw(4) << (int)frame.data[7] << endl
      << endl;
  //  n = sendto(s, &frame, sizeof(struct can_frame), 0, 
  //      (struct sockaddr*)&addr, sizeof(addr));

  while(ros::ok())
  {
    frame.can_id  = 0x586;
    frame.can_dlc = 8;

    n = read(s, &frame, sizeof(struct can_frame));

    if(n > 0)
      cout << setw(8) << hex << uppercase
        << "frame.can_id  : " << (int)frame.can_id << endl
        << "frame.can_dlc : " << (int)frame.can_dlc << endl
        << "frame.__pad   : " << (int)frame.__pad << endl
        << "frame.__res0  : " << (int)frame.__res0 << endl
        << "frame.__res1  : " << (int)frame.__res1 << endl
        << (int)frame.data[0] << ", " << (int)frame.data[1] << ", " << (int)frame.data[2] << ", " << (int)frame.data[3] << ", " << endl
        << (int)frame.data[4] << ", " << (int)frame.data[5] << ", " << (int)frame.data[6] << ", " << (int)frame.data[7] << endl
        << endl;

    ros::spinOnce();
    usleep(1000);
  }

  frame.can_id  = 0x606;
  frame.can_dlc = 5;
  frame.__pad   = 0;
  frame.__res0  = 0;
  frame.__res1  = 0;
  frame.data[0] = 0xc0;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  n = sendto(s, &frame, sizeof(struct can_frame), 0, 
      (struct sockaddr*)&addr, sizeof(addr));

  close(s);
  return 0;
}

