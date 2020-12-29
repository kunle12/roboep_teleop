//
//  roboep_teleop.cpp
//  PyRIDE
//
//  Created by Xun Wang on 23/12/2020.
//  Copyright (c) 2020 Xun Wang. All rights reserved.
//

#include <ros/ros.h>
#include <signal.h>

#include "roboep_teleop.h"

using namespace roboep;
static RoboEPTeleop * s_server = NULL;

void stopProcess( int sig )
{
  if (s_server)
    s_server->stopProcess();
}

int main( int argc, char * argv[] )
{
  ros::init( argc, argv, "robope_teleop", ros::init_options::NoSigintHandler );

  s_server = new RoboEPTeleop();

  signal( SIGINT, ::stopProcess );
  
  if (s_server->init()) {
    s_server->continueProcessing();
  }

  s_server->fini();
  
  delete s_server;

  ros::shutdown();
  return 0;
}
