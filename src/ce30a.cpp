#include <ros/ros.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <error.h>
#include <net/if.h>

using namespace std;

void PrintCANFrame(struct can_frame f);

// id, dlc, pad, res0, res1, data
static const struct can_frame cmd_start = {
  0x606, 5, 0, 0, 0, {0xc1, 0x00, 0x00, 0x00, 0x00, }
};
static const struct can_frame cmd_stop = {
  0x606, 5, 0, 0, 0, {0xc0, 0x00, 0x00, 0x00, 0x00, }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ce30a_node");
  ros::NodeHandle nh;

  int s = -1;
  struct sockaddr_can addr;
  string ifname = "can0";
  struct ifreq ifr;
  struct can_frame frame;

  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if(s < 0)
  {
    cout << "socket open failed. " << endl;
    return -1;
  }

  strcpy(ifr.ifr_name, ifname.c_str());
  //ioctl(s, SIOCGIFINDEX, &ifr);
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  //cout << ifname << " at index " << ifr.ifr_ifindex << endl;

  struct timeval timeout;      
  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;
  setsockopt (s, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));

  if(bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    cout << "socket bind failed. " << endl;
    close(s);
    return -2;
  }

  int n = -1;

  frame = cmd_stop;
    n = write(s, &frame, sizeof(struct can_frame));
    cout << n <<endl;

  frame = cmd_start;
    n = write(s, &frame, sizeof(struct can_frame));
    cout << n <<endl;

  while(ros::ok())
  {
    n = read(s, &frame, sizeof(struct can_frame));

    if(n > 0 && frame.can_id == 0x586 )
    {
      int obstacle_dist = (int)(((frame.data[1] << 16) & 0xff00) | frame.data[0]);  //cm
      int obstacle_angle = (int8_t)frame.data[3];                        //degree

      cout<< showbase << setfill('0') << hex << right << internal;

      cout << setw(8) << dec
        << "obstacle_dist  : " << obstacle_dist << endl
        << "obstacle_angle : " << obstacle_angle << endl
        << endl;
    }

    ros::spinOnce();
  }

  frame = cmd_stop;
  write(s, &frame, sizeof(struct can_frame));

  close(s);
  return 0;
}





void PrintCANFrame(struct can_frame f)
{
  cout<< showbase << setfill('0') << hex << right << internal;

  cout << setw(8) << hex << uppercase
    << "frame.can_id  : " << (int)f.can_id << endl
    << "frame.can_dlc : " << (int)f.can_dlc << endl
    //<< "frame.__pad   : " << (int)f.__pad << endl
    //<< "frame.__res0  : " << (int)f.__res0 << endl
    //<< "frame.__res1  : " << (int)f.__res1 << endl
    << dec
    << (int)f.data[0] << ", " << (int)f.data[1] << ", " << (int)f.data[2] << ", " << (int)f.data[3] << ", " << endl
    << (int)f.data[4] << ", " << (int)f.data[5] << ", " << (int)f.data[6] << ", " << (int)f.data[7] << endl
    << endl;
}


