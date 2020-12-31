//
//  roboep_teleop.h
//  PyRIDE
//
//  Created by Xun Wang on 23/12/2020.
//  Copyright (c) 2020 Xun Wang. All rights reserved.
//

#ifndef ROBOEP_TELE_OP_H_
#define ROBOEP_TELE_OP_H_

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <signal.h>

#include <iostream>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace ros;

namespace roboep {

class RoboEPTeleop
{
public:
  RoboEPTeleop();
  virtual ~RoboEPTeleop();

  bool init();
  void fini();

  void stopProcess();

  void continueProcessing();

private:
  NodeHandle priNode_;

  struct sockaddr_in epcoreAddr_;

  Subscriber nodeSub_;

  volatile sig_atomic_t isRunning_;
  int connSocket_;
  int maxFD_;
  fd_set masterFDSet_;
  unsigned char * clientDataBuffer_;

  boost::mutex mutex_;
  
  geometry_msgs::TwistConstPtr twistMsgPtr_;

  AsyncSpinner * cmdDataThread_;
  CallbackQueue cmdDataQueue_;

  bool initRobotConnection( const string & epcoreID );
  bool getRobotAddress( const string & epname );

  void cmdVelCB( const geometry_msgs::TwistConstPtr & msg );
  void processIncomingData( fd_set * readyFDSet );
};

} // namespace roboep

#endif /* ROBOEP_TELE_OP_H_ */
