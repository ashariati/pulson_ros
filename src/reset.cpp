#include <ros/ros.h>

#include <vector>
#include <boost/algorithm/string.hpp>

#include <pulson_ros/P4xx/RCM/rcmIf.h>
#include <pulson_ros/P4xx/RCM/rcm.h>
#include <pulson_ros/RangeArray.h>
#include <pulson_ros/RangeMeasurement.h>

void print_status(rcmMsg_GetStatusInfoConfirm *statusInfo);
void print_configuration(rcmConfiguration *config);
void error_check(int r, const char *msg);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "reset");
    ros::NodeHandle nh("~");

    // parameters
    int node_id;
    std::string dev_path;
    rcmIfType rcmIf;

    nh.param("node_id", node_id, 100);

    if (nh.getParam("ip_address", dev_path))
    {
       rcmIf = rcmIfIp;
    }
    else if (nh.getParam("serial_dev", dev_path))
    {
      rcmIf = rcmIfSerial;
    }
    else
    {
      nh.param("dev_path", dev_path, std::string("/dev/ttyACM0"));
      rcmIf = rcmIfUsb;
    }

    int r;

    // initialize interface
    r = rcmIfInit(rcmIf, (char*) dev_path.c_str());
    error_check(r, "Initialization Failed");

    // put in idle mode during configuration
    r = rcmSleepModeSet(RCM_SLEEP_MODE_IDLE);
    error_check(r, "Time out waiting for sleep mode set");

    // make sure correct internal opmode is set
    r = rcmOpModeSet(RCM_OPMODE_RCM);
    error_check(r, "Timeout waiting for opmode set");

    // execute Built-In Test
    int status;
    r = rcmBit(&status);
    error_check(r, "Timeout waiting for test");
    error_check(status, "Built-In Test Failed");

    // get status information
    rcmMsg_GetStatusInfoConfirm statusInfo;
    r = rcmStatusInfoGet(&statusInfo);
    error_check(r, "Timeout waiting for status information");
    print_status(&statusInfo);

    // get configuration from RCM
    rcmConfiguration config;
    r = rcmConfigGet(&config);
    error_check(r, "Timeout watiing for configuration");
    print_configuration(&config);

    // default configuration
    config.nodeId = node_id;
    config.integrationIndex = 7;
    config.codeChannel = 0;
    config.electricalDelayPsA = 0;
    config.electricalDelayPsB = 0;
    config.flags = 0;
    config.flags |= RCM_DISABLE_CRE_RANGES;
    config.txGain = 63;
    config.persistFlag = 2;

    r = rcmConfigSet(&config);
    error_check(r, "Time out waiting for rcmConfig confirm");
    print_configuration(&config);

    // cleanup
    rcmIfFlush();
    rcmIfClose();

}


void print_status(rcmMsg_GetStatusInfoConfirm *statusInfo)
{
    char buf[1024];
    sprintf(buf, "\n\tStatus Information:\n");
    sprintf(buf, "%s\t\tPackage Version: %s\n", buf, statusInfo->packageVersionStr);
    sprintf(buf, "%s\t\tRCM Version: %d.%d Build %d\n", buf,
            statusInfo->appVersionMajor,
            statusInfo->appVersionMinor,
            statusInfo->appVersionBuild);
    sprintf(buf, "%s\t\tUWB Kernel Version: %d.%d Build %d\n", buf,
            statusInfo->uwbKernelVersionMajor,
            statusInfo->uwbKernelVersionMinor,
            statusInfo->uwbKernelVersionBuild);
    sprintf(buf, "%s\t\tFirmware Version: %x/%x/%x Ver. %X\n", buf,
            statusInfo->firmwareMonth,
            statusInfo->firmwareDay,
            statusInfo->firmwareYear,
            statusInfo->firmwareVersion);
    sprintf(buf, "%s\t\tSerial Number: %08X\n", buf, statusInfo->serialNum);
    sprintf(buf, "%s\t\tBoard Revision: %c\n", buf, statusInfo->boardRev);
    sprintf(buf, "%s\t\tTemperature: %.2f degC\n", buf, statusInfo->temperature/4.0);

    ROS_INFO("[rcm_node] %s", buf);

}

void print_configuration(rcmConfiguration *config)
{
    char buf[1024];
    sprintf(buf, "\n\tConfiguration:\n");
    sprintf(buf, "%s\t\tNode ID: %d\n", buf, config->nodeId);
    sprintf(buf, "%s\t\tIntegration Index: %d\n", buf, config->integrationIndex);
    sprintf(buf, "%s\t\tAntenna Mode: %d\n", buf, config->antennaMode);
    sprintf(buf, "%s\t\tCode Channel: %d\n", buf, config->codeChannel);
    sprintf(buf, "%s\t\tElectrical Delay PsA: %d\n", buf, config->electricalDelayPsA);
    sprintf(buf, "%s\t\tElectrical Delay PsB: %d\n", buf, config->electricalDelayPsB);
    sprintf(buf, "%s\t\tFlags: 0x%X\n", buf, config->flags);
    sprintf(buf, "%s\t\ttxGain: %d\n", buf, config->txGain);

    ROS_INFO("[rcm_node] %s", buf);
}

void error_check(int r, char const *msg)
{
    if (r != 0)
    {
        ROS_ERROR("[rcm_node] %s\n", msg);
        ros::requestShutdown();
    }

}
