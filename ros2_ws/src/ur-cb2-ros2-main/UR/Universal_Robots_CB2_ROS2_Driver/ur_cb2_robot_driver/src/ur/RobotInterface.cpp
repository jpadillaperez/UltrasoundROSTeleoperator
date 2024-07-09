/*! \file robotInterface.cpp */

#include "ur_cb2_robot_driver/ur/RobotInterface.h"

#include <cstring>
#include <cisstOSAbstraction/osaSocket.h>

// Packet structs for different versions
#pragma pack(push, 1)     // Eliminate structure padding
struct module1 {
    uint32_t messageSize;
    double time;          // Time elapsed since controller was started
    double qTarget[6];    // Target joint positions
    double qdTarget[6];   // Target joint velocities
    double qddTarget[6];  // Target joint accelerations
    double I_Target[6];   // Target joint currents
    double M_Target[6];   // Target joint torques
    double qActual[6];    // Actual joint positions
    double qdActual[6];   // Actual joint velocities
    double I_Actual[6];   // Actual joint currents
};
#pragma pack(pop)

#pragma pack(push, 1)
struct module2 {
    unsigned long long digital_Input;  // Digital input bitmask
    double motor_Tem[6];      // Joint temperatures (degC)
    double controller_Time;   // Controller real-time thread execution time
    double test_Val;          // UR internal use only
    double robot_Mode;        // Robot mode (see RobotModes enum)
    double joint_Modes[6];    // Joint control modes (Version 1.8+, see JointModes enum)
};
#pragma pack(pop)

#pragma pack(push, 1)
struct packet_pre_3 {
    module1 base1;
    double tool_Accele[3];    // Tool accelerometer values (Version 1.7+)
    double blank[15];         // Unused
    double TCP_force[6];      // Generalized forces in the TCP
    double tool_Vector[6];    // Tool Cartesian pose (x, y, z, rx, ry, rz)
    double TCP_speed[6];      // Tool Cartesian speed
    module2 base2;
};
#pragma pack(pop)

RobotInterface::RobotInterface() :
  moving(false)
{
  memset(&jointData.raw[0], 0, sizeof(jointData));
  memset(&robotMode.raw[0], 0, sizeof(robotMode));
  memset(&toolData.raw[0], 0, sizeof(toolData));
  memset(&masterboardData.raw[0], 0, sizeof(masterboardData));
  memset(&masterboardDataWithEuromap.raw[0], 0, sizeof(masterboardDataWithEuromap));
  memset(&cartesianInfo.raw[0], 0, sizeof(cartesianInfo));
  memset(&configurationData.raw[0], 0, sizeof(configurationData));
  memset(&forceModeData.raw[0], 0, sizeof(forceModeData));
  memset(&additionalInfo.raw[0], 0, sizeof(additionalInfo));

  jointModes.SetAll(0);
  jsPosition.SetSize(NB_Actuators);
  jsVelocity.SetSize(NB_Actuators);
  jsEffort.SetSize(NB_Actuators);
  jsTPosition.SetSize(NB_Actuators);
  jsTVelocity.SetSize(NB_Actuators);
  jsTEffort.SetSize(NB_Actuators);
  toolPose.SetSize(6);
}

RobotInterface::~RobotInterface()
{
}

struct UniversalRobot::RobotState RobotInterface::getState() const
{

  UniversalRobot::RobotState robotState;
  // switch (static_cast<UniversalRobot::RobotMode>(ur_interface.robotMode_))
  // {
  // case UniversalRobot::ROBOT_EMERGENCY_STOPPED
  //   robotState = UniversalRobot::RobotState::isEmergencyStopped;
  //   break;

  // case UniversalRobot::ROBOT_READY
  //   robotState = UniversalRobot::RobotState::isRobotConnected;
  //   break;
  
  // default:
  //   break;
  // }
  return robotState;
}

void RobotInterface::readPacket(const char *buf, const int len)
{

  if(buf[0] != char(16)) {
    std::cerr << "Unexpected packet type" << std::endl;
    return;
  }
  buf++;
  int readBytes = 1;
  int bytesToRead = 0;
  UniversalRobot::intUnion packLen;
  int packType;

  // read in each individual packet
  while(readBytes < len) {
    memcpy(&packLen.raw[0], &buf[0], 4);
    packType = buf[4];
    bytesToRead = unionValue(packLen);

    switch(packType) {
    case UniversalRobot::ROBOT_MODE_DATA:
      memcpy(&robotMode.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::JOINT_DATA:
      memcpy(&jointData.raw[0], &buf[0], bytesToRead);

      // compute the Jacobian
      for(int i=0; i<6; i++)
      {
        cur_joint_velocity_[i] = unionValue(jointData.jd.joint[i].qd_actual);
        cur_joints_[i] = unionValue(jointData.jd.joint[i].q_actual);
      }
      cur_geo_jac_ = ur_kin_.geo_jac(cur_joints_);

      break;
    case UniversalRobot::TOOL_DATA:
      memcpy(&toolData.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::MASTERBOARD_DATA:
      if(bytesToRead == 72)
        memcpy(&masterboardData.raw[0], &buf[0], bytesToRead);
      else
        memcpy(&masterboardDataWithEuromap.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::CARTESIAN_INFO:
      memcpy(&cartesianInfo.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::KINEMATICS_INFO:
      // nothing to do here
      break;
    case UniversalRobot::CONFIGURATION_DATA:
      memcpy(&configurationData.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::FORCE_MODE_DATA:
      memcpy(&forceModeData.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::ADDITIONAL_INFO:
      memcpy(&additionalInfo.raw[0], &buf[0], bytesToRead);
      break;
    case UniversalRobot::CALIBRATION_DATA:
      // nothing to do here
      break;
    default:
      std::cerr << "Unrecognized packet " << packType << std::endl;
      return;
    }

    readBytes += bytesToRead;
    buf += bytesToRead;
  }
}

void RobotInterface::saveDH()
{
  for(int i=0; i<6; i++) {
    a[i] = unionValue(configurationData.config.dh_a[i]) * 1000.0;
    d[i] = unionValue(configurationData.config.dh_d[i]) * 1000.0;
    alpha[i] = unionValue(configurationData.config.dh_alpha[i]);
  }
  std::cout<<"SaveDH"<<std::endl;
}

// // This will eventually replace the run method
// void RobotInterface::readFromSocket(osaSocket& sock)
// {
//   UniversalRobot::intUnion len;
//   len.data = 0;
//   int nBytes = sock.Receive(&len.raw[0], 4);
//   if((nBytes != 4) && (nBytes != 0)) {
//     // bad packet
//     std::cerr << "Bad packet length" << std::endl;
//     return;
//   }
//   std::cout << "nBytes length: " <<nBytes << std::endl;
//   int packetLength = unionValue(len);
  
//   if((packetLength <= 4) || (packetLength > 4096))
//     return;

//   // read the packet, less the 4 bytes for length
//   int packetBytes = packetLength - 4;
//   nBytes = sock.Receive(&ur_packet_buf[0], packetBytes);

//   if(nBytes != packetBytes) {
//     // bad packet
//     std::cerr << "Bad packet" << std::endl;
//     return;
//   }
//   //std::cout << "read bytes: " << packetBytes + 4 << std::endl;
//   //readPacket(ur_packet_buf, packetBytes);
// }

void RobotInterface::readFromSocket(osaSocket& sock)
{
    // Receive a packet with timeout. We choose a timeout of 1 sec, which is much
    // larger than expected (should get packets every 8 msec). Thus, if we don't get
    // a packet, then we raise the ReceiveTimeout event.
    buffer[buffer_idx+0] = buffer[buffer_idx+1] = buffer[buffer_idx+2] = buffer[buffer_idx+3] = 0;
    int numBytes = buffer_idx + sock.Receive(buffer+buffer_idx, sizeof(buffer)-buffer_idx, 1.0 * 1.0);
    if (numBytes < 0) {
        buffer_idx = 0;
        sock.Close();
        return;
    }
    else if (numBytes == 0) {
        buffer_idx = 0;
        return;
    }
    uint32_t packageLength;
    // Byteswap package length
    char *p = (char *)(&packageLength);
    p[0] = buffer[3]; p[1] = buffer[2]; p[2] = buffer[1]; p[3] = buffer[0];
    // std::cout<< numBytes <<" " <<packageLength<< std::endl;

    if ((packageLength > 0) && (packageLength <= static_cast<uint32_t>(numBytes))) {
      // Byteswap all the doubles in the package
      for (size_t i = 4; i < packageLength-4; i += 8) {
          for (size_t j = 0; j < 4; j++) {
              char tmp = buffer[i + j];
              buffer[i + j] = buffer[i + 7 - j];
              buffer[i + 7 - j] = tmp;
          }
      }

      // Following is valid for all versions
      module1 *base1 = reinterpret_cast<module1 *>(buffer);
      // First, do a sanity check on the packet. The new ControllerTime (base1->time)
      // should be about 0.008 seconds (CB2/CB3) or 0.002 seconds (e-Series) later than
      // the previous value.

      // double timeDiff = base1->time - ControllerTime;
      ControllerTime = base1->time;

      jsPosition.Assign(base1->qActual);
      jsVelocity.Assign(base1->qdActual);
      jsEffort.Assign(base1->I_Actual);

      
      // compute the Jacobian
      for(int i=0; i<6; i++)
      {
        cur_joint_velocity_[i] = jsVelocity[i];
        cur_joints_[i] = jsPosition[i];
      }
      cur_geo_jac_ = ur_kin_.geo_jac(cur_joints_);

      jsTPosition.Assign(base1->qTarget);
      jsTVelocity.Assign(base1->qdTarget);
      jsTEffort.Assign(base1->I_Target);


      module2 *base2 = (module2 *)(&((packet_pre_3 *)buffer)->base2);
      // Following is documented to be "controller realtime thread execution time"
      // Not sure what this is, or what are the units
      ControllerExecTime = base2->controller_Time;
      if (base2->robot_Mode < 0.0)
          robotMode_ = static_cast<int>(base2->robot_Mode-0.5);
      else
          robotMode_ = static_cast<int>(base2->robot_Mode+0.5);

      // joint_Modes should always be positive
      for (int i = 0; i < NB_Actuators; i++)
          jointModes[i] = static_cast<int>(base2->joint_Modes[i]+0.5);

      // We use the jointModes rather than robotMode to determine whether power is on because
      // the defined jointModes are consistent between firmware versions, whereas robotMode is not.
      // For jointModes, power is on if we are RUNNING, FREEDRIVE, INITIALISATION, or VIOLATION.
      // The following code handles joints in any combination of the above states (e.g., some joints
      // can be in JOINT_INITIALISATION_MODE while the rest of the joints are in JOINT_RUNNING_MODE).
      // Note that in cisstVector, addition is specialized as logical OR for boolean vectors.
      vctBool6 jointHasPower(jointModes.ElementwiseEqual(UniversalRobot::JointMode::RUNNING));
      jointHasPower.Add(jointModes.ElementwiseEqual(UniversalRobot::JointMode::JOINT_FREEDRIVE));
      jointHasPower.Add(jointModes.ElementwiseEqual(UniversalRobot::JointMode::JOINT_INITIALIZATION));
      jointHasPower.Add(jointModes.ElementwiseEqual(UniversalRobot::JointMode::JOINT_STOPPED));
      isPowerOn = jointHasPower.All();

      double *tool_vec = 0;

      packet_pre_3 *packet = (packet_pre_3 *)(buffer);
      // Documentation does not specify whether tool_Vector or TCP_speed
      // are the actual or target Cartesian position or velocity.
      // We assume they are the actual (measured) position or velocity.
      tool_vec = packet->tool_Vector;
      // Whether e-stop is pressed.
      // Could instead use (robotMode == ROBOT_EMERGENCY_STOPPED_MODE)
      isEStop = jointModes.Equal(UniversalRobot::JointMode::EMERGENCY_STOPPED);
      // Whether security stop is activated.
      // Could instead use (robotMode == ROBOT_SECURITY_STOPPED_MODE)
      isSecurityStop = jointModes.Equal(UniversalRobot::JointMode::JOINT_STOPPED);

      if (tool_vec) {
        toolPose.Assign(packet->tool_Vector);
        vct3 position(tool_vec);
        vct3 orientation(tool_vec+3);
        vctRodriguezRotation3<double> rot(orientation);
        vctDoubleRot3 cartRot(rot);  // rotation matrix, from world frame to the end-effector frame
        vctFrm3 frm(cartRot, position);
      }

      // Finished with packet; now preserve any extra data for next time
      if (packageLength < static_cast<unsigned long>(numBytes)) {
          memmove(buffer, buffer+packageLength, numBytes-packageLength);
          buffer_idx = numBytes-packageLength;
      } else {
          buffer_idx = 0;
      }
    }
    else {
        buffer_idx = 0;
        // purge buffer
        while (sock.Receive(buffer, sizeof(buffer)) > 0);
    }

}


void RobotInterface::getPositions(double pos[6])
{
  // for(int i=0; i<6; i++)
  //   pos[i] = unionValue(jointData.jd.joint[i].q_actual);

  for(int i=0; i<6; i++)
  {
    pos[i] = jsPosition[i];
  }
}

void RobotInterface::getToolPose(double pose[6])
{
  // pose[0] = unionValue(cartesianInfo.info.x);
  // pose[1] = unionValue(cartesianInfo.info.y);
  // pose[2] = unionValue(cartesianInfo.info.z);
  // pose[3] = unionValue(cartesianInfo.info.Rx);
  // pose[4] = unionValue(cartesianInfo.info.Ry);
  // pose[5] = unionValue(cartesianInfo.info.Rz);
  for(int i=0; i<6; i++)
  {
    pose[i] = toolPose[i];
  }
}

int RobotInterface::unionValue(UniversalRobot::intUnion u)
{
  UniversalRobot::intUnion tmp;
  for(int i=0; i<4; i++)
    tmp.raw[3-i] = u.raw[i];

  return tmp.data;
}

float RobotInterface::unionValue(UniversalRobot::floatUnion u)
{
  UniversalRobot::floatUnion tmp;
  for(int i=0; i<4; i++)
    tmp.raw[3-i] = u.raw[i];

  return tmp.data;
}

double RobotInterface::unionValue(UniversalRobot::doubleUnion u)
{
  UniversalRobot::doubleUnion tmp;
  for(int i=0; i<8; i++)
    tmp.raw[7-i] = u.raw[i];

  return tmp.data;
}

