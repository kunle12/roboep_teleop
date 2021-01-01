//
//  roboep_teleop.cpp
//  PyRIDE
//
//  Created by Xun Wang on 23/12/2020.
//  Copyright (c) 2020 Xun Wang. All rights reserved.
//
#include <time.h>
#include <stdio.h>
#include "roboep_teleop.h"

#define INVALID_SOCKET  -1
#define ROBOTEP_DEFAULT_BUFFER_SIZE 4096

namespace roboep {

static const int kPublishFreq = 10;
static const short kCtrlPort = 40923;
static const short kBCPort = 40926;
static const float kMaxXSpeed = 3.0;
static const float kMaxYSpeed = 2.5;
static const float kMaxThetaSpeed = 30.0;

static inline long timediff_usec( timespec start, timespec end )
{
  if ((end.tv_nsec - start.tv_nsec) < 0) {
    return (end.tv_sec - start.tv_sec - 1) * 1E6 + (1E9 + end.tv_nsec - start.tv_nsec) / 1E3;
  }
  else {
    return (end.tv_sec - start.tv_sec) * 1E6 + (end.tv_nsec - start.tv_nsec) / 1E3;
  }
}

static inline float clamp( float val, float max )
{
  if (val > max) {
    return max;
  }
  else if (val < -max) {
    return -max;
  }
  else {
    return val;
  }
}
RoboEPTeleop::RoboEPTeleop() :
  isRunning_( 0 ),
  connSocket_( INVALID_SOCKET ),
  maxFD_( 0 ),
  clientDataBuffer_( NULL ),
  cmdDataThread_( NULL )
{
  FD_ZERO( &masterFDSet_ );
}

RoboEPTeleop::~RoboEPTeleop()
{

}

bool RoboEPTeleop::init()
{
  NodeHandle priNh( "~" );

  std::string epcoreID;

  priNh.param<std::string>( "epcore", epcoreID, "local" );

  ROS_INFO( "EP core name %s", epcoreID.c_str() );

  if (!initRobotConnection( epcoreID )) {
    ROS_ERROR( "Unable to initialise robot connection" );
    return false;
  }

  ros::SubscribeOptions sopts = ros::SubscribeOptions::create<geometry_msgs::Twist>( "cmd_vel",
        1, boost::bind( &RoboEPTeleop::cmdVelCB, this, _1 ), ros::VoidPtr(), &cmdDataQueue_ );

  nodeSub_ = priNode_.subscribe( sopts );

  cmdDataThread_ = new ros::AsyncSpinner( 1, &cmdDataQueue_ );
  cmdDataThread_->start();

  return true;
}

bool RoboEPTeleop::initRobotConnection( const string & epcoreID )
{
  if (!getRobotAddress( epcoreID )) {
    ROS_ERROR( "Unable to retrieve robotmaster EP core ip address" );
    return false;
  }

  connSocket_ = socket( AF_INET, SOCK_STREAM, 0 );
  if (connSocket_ == INVALID_SOCKET) {
    ROS_ERROR( "Unable to create TCP connection socket to robot." );
    return false;
  }

  if (connect( connSocket_, (struct sockaddr *)&epcoreAddr_, sizeof( epcoreAddr_ ) ) < 0) {
    ROS_ERROR( "Unable to connect to robomaster ep core." );
    close( connSocket_ );
    connSocket_ = INVALID_SOCKET;
    return false;
  }
  FD_SET( connSocket_, &masterFDSet_ );
  maxFD_ = max( connSocket_, maxFD_ );
  clientDataBuffer_ = new unsigned char[ROBOTEP_DEFAULT_BUFFER_SIZE];
  // enter command mode
  write( connSocket_, "command;", 8 );

  ROS_INFO( "Connected to robomaster ep core." );

  return true;
}

bool RoboEPTeleop::getRobotAddress( const string & epcorename )
{
  char ipAddrStr[20];
  memset( ipAddrStr, '\0', 20 );

  if (epcorename.compare( "local" )) { // not local usb connection
    int bcsocket = socket( AF_INET, SOCK_DGRAM, 0 );
    if (bcsocket == INVALID_SOCKET) {
      ROS_ERROR( "Unable to create UDP socket to listen for robot address." );
      return false;
    }

    struct sockaddr_in bcAddr;
    bcAddr.sin_family = AF_INET;
    bcAddr.sin_addr.s_addr = INADDR_ANY;
    bcAddr.sin_port = htons( kBCPort );

    if (bind( bcsocket, (struct sockaddr *)&bcAddr, sizeof( bcAddr ) ) < 0) {
      ROS_ERROR( "Unable to bind to network interface." );
      close( bcsocket );
      return false;
    }

    fd_set  masterFDSet;
    fd_set readyFDSet;

    FD_ZERO( &masterFDSet );
    FD_SET( bcsocket, &masterFDSet );
    int recvDataLen = 0;

    ROS_INFO( "Waiting for robot address on UDP broadcasting..." );
    isRunning_ = 1;
    char dataBuffer[200];

    while (recvDataLen < 16 && isRunning_) {
      FD_ZERO( &readyFDSet );
      memcpy( &readyFDSet, &masterFDSet, sizeof( masterFDSet ) );
      int localMaxFD = bcsocket;

      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000; // 100ms
      select( localMaxFD + 1, &readyFDSet, NULL, NULL, &timeout );
    
      if (isRunning_ && FD_ISSET( bcsocket, &readyFDSet )) {
        recvDataLen = recvfrom( bcsocket, dataBuffer, 200,
                          0, NULL, NULL );
      }
    }
    strncpy( ipAddrStr, dataBuffer+9, (recvDataLen - 9) );
    ROS_INFO( "Received robot IP address as %s", ipAddrStr );
    close( bcsocket );
    isRunning_ = 0;
  }
  else {
    strncpy( ipAddrStr, "192.168.42.2", 12 );
    ROS_INFO( "Receive default local USB robot IP address as %s", ipAddrStr );
  }

  unsigned long saddr = 0;
  struct hostent * hostInfo = gethostbyname( ipAddrStr ); // try resolve name first

  if (hostInfo) {
    memcpy( (char *)&epcoreAddr_.sin_addr, hostInfo->h_addr, hostInfo->h_length );
    epcoreAddr_.sin_family = AF_INET;
    epcoreAddr_.sin_port = htons( kCtrlPort );
    return true;
  }
  return false;
}

void RoboEPTeleop::fini()
{
  isRunning_ = 0; // not really necessary

  nodeSub_.shutdown();

  if (connSocket_ != INVALID_SOCKET) {
    ROS_INFO( "close connection with the robot." );
    write( connSocket_, "quit;", 5 );
    usleep( 20000 );
    close( connSocket_ );
    FD_CLR( connSocket_, &masterFDSet_ );
    connSocket_ = INVALID_SOCKET;
    delete [] clientDataBuffer_;
    clientDataBuffer_ = NULL;
  }
  cmdDataThread_->stop();
  delete cmdDataThread_;
  cmdDataThread_ = NULL;
}

void RoboEPTeleop::processIncomingData( fd_set * readyFDSet )
{
  if (isRunning_ && FD_ISSET( connSocket_, readyFDSet )) {
    int readLen = (int)read( connSocket_, clientDataBuffer_, 2048 );
    if (readLen <= 0) {
      if (readLen == 0) {
        ROS_INFO( "Socket connection %d closed.\n", (int)connSocket_ );
      }
      else {
        ROS_ERROR( "RoboEPTeleop::processIncomingData: "
                  "error reading data stream on %d error = %d.\n", (int)connSocket_, errno );
      }
      close( connSocket_ );
      FD_CLR( connSocket_, &masterFDSet_ );
      connSocket_ = INVALID_SOCKET;
    }
    else {
      //ROS_INFO( "Receive %s from robot", clientDataBuffer_ );
    }
  }
}

void RoboEPTeleop::cmdVelCB( const geometry_msgs::TwistConstPtr & msg )
{
  boost::mutex::scoped_lock lock( mutex_, boost::try_to_lock );
  if (lock) {
    twistMsgPtr_ = msg;
  }
}

void RoboEPTeleop::stopProcess()
{
  isRunning_ = 0;
}

void RoboEPTeleop::continueProcessing()
{
  fd_set readyFDSet;
  isRunning_ = 1;

  timespec time1, time2;

  long interval = long( 1.0 / double( kPublishFreq ) * 1E6);
  long proctime = 0;

  float x, y, theta;
  int missing_data_cnt = 0;
  char veldata[200];

  while (isRunning_) {
    clock_gettime( CLOCK_MONOTONIC, &time1 );

    struct timeval timeout, timeStamp;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100ms

    FD_ZERO( &readyFDSet );
    memcpy( &readyFDSet, &masterFDSet_, sizeof( masterFDSet_ ) );
    int maxFD = maxFD_;

    select( maxFD+1, &readyFDSet, NULL, NULL, &timeout ); // non-blocking select

    processIncomingData( &readyFDSet );

    ros::spinOnce();

    // The follow code works best with my own version of bs4 driver
    { // release the lock asap
      boost::mutex::scoped_lock lock( mutex_ );
      if (twistMsgPtr_.get()) {
        x = clamp( twistMsgPtr_->linear.x * 3, kMaxXSpeed );
        y = clamp( twistMsgPtr_->linear.y * 2.5, kMaxYSpeed );
        theta = clamp( twistMsgPtr_->angular.z * 10, kMaxThetaSpeed );
        missing_data_cnt = 0;
        twistMsgPtr_.reset();
      }
      else if (missing_data_cnt < kPublishFreq * 2) {
        missing_data_cnt++;
      }
    }

    if (!missing_data_cnt) {
      snprintf( veldata, 200, "chassis speed x %.2f y %.2f z %.2f;", x, y, theta );
      if (connSocket_ != INVALID_SOCKET) {
        write( connSocket_, veldata, strlen(veldata) );
      }
    }
    else if (missing_data_cnt >= kPublishFreq * 2) { // more than 2 seconds
      snprintf( veldata, 200, "chassis wheel w1 0.0 w2 0.0 w3 0.0 w4 0.0;" );
      if (connSocket_ != INVALID_SOCKET) {
        write( connSocket_, veldata, strlen(veldata) );
      }
    }

    clock_gettime( CLOCK_MONOTONIC, &time2 );
    proctime = timediff_usec( time1, time2 );
    //printf( "detect process time %li usec\n",  proctime);
    if (interval > proctime)
      usleep( interval - proctime );
  }
}
/*
void RoboEPTeleop::setFD( const SOCKET_T & fd )
{
  FD_SET( fd, &masterFDSet_ );
  maxFD_ = max( fd, maxFD_ );
}

void RoboEPTeleop::clearFD( const SOCKET_T & fd )
{
  FD_CLR( fd, &masterFDSet_ );
  maxFD_ = max( fd, maxFD_ );
}
*/

} // namespace roboep
