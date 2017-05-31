#include <ros/ros.h>

#include <vector>
#include <boost/algorithm/string.hpp>

#include <pulson_ros/P4xx/RCM/rcmIf.h>
#include <pulson_ros/P4xx/RCM/rcm.h>
#include <pulson_ros/RangeArray.h>
#include <pulson_ros/RangeMeasurement.h>

void init(rcmIfType rcmIf, std::string dev_path);
void print_status(rcmMsg_GetStatusInfoConfirm *statusInfo);
void print_configuration(rcmConfiguration *config);
void error_check(int r, const char *msg);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rcm_node");
    ros::NodeHandle nh("~");

    // parameters
    int rate;
    std::string dev_path;
    std::string network_nodes;

    // interface
    rcmIfType rcmIf;

    nh.param("rate", rate, 30);
    nh.param("network_nodes", network_nodes, std::string(""));

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
      rcmIf = rcmIfUsb;
    }

    // publisher
    ros::Publisher range_pub = nh.advertise<pulson_ros::RangeArray>("ranges", 100);

    // initialization
    init(rcmIf, dev_path);

    // extract node IDs from parameter
    std::vector<std::string> node_ids;
    boost::split(node_ids, network_nodes, boost::is_any_of("\t "));

    // variables to store the data
    int r;
    rcmMsg_FullRangeInfo rangeInfo;
    rcmMsg_DataInfo dataInfo;
    rcmMsg_ScanInfo scanInfo;
    rcmMsg_FullScanInfo fullScanInfo;

    // publish range data
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        // message to be built
        pulson_ros::RangeArray range_array;

        // loop through all the nodes in the network
        for (int i = 0; i < node_ids.size(); i++)
        {

            // get next id
            int id = atoi(node_ids[i].c_str());
            if (id == 0)
            {
                break;
            }

            // compute range measurement
            r = rcmRangeTo(id, RCM_ANTENNAMODE_TXA_RXA, 0, NULL,
                    &rangeInfo, &dataInfo, &scanInfo, &fullScanInfo);
            if (r != 0)
            {
                continue;
            }

            // create message
            pulson_ros::RangeMeasurement range_measurement;
            range_measurement.message_id = rangeInfo.msgId;
            range_measurement.responder_id = rangeInfo.responderId;
            range_measurement.range_status = rangeInfo.rangeStatus;
            range_measurement.antenna_mode = rangeInfo.antennaMode;
            range_measurement.stopwatch_time = rangeInfo.stopwatchTime;
            range_measurement.precision_range = rangeInfo.precisionRangeMm;
            range_measurement.coarse_range = rangeInfo.coarseRangeMm;
            range_measurement.filtered_range = rangeInfo.filteredRangeMm;
            range_measurement.precision_range_error = rangeInfo.precisionRangeErrEst;
            range_measurement.coarse_range_error = rangeInfo.coarseRangeErrEst;
            range_measurement.filtered_range_error = rangeInfo.filteredRangeErrEst;
            range_measurement.filtered_velocity = rangeInfo.filteredRangeVel;
            range_measurement.filtered_velocity_error = rangeInfo.filteredRangeVelErrEst;
            range_measurement.timestamp = rangeInfo.timestamp;

            // save
            range_array.ranges.push_back(range_measurement);
        }

        // publish
        range_pub.publish(range_array);

        // block
        loop_rate.sleep();
    }

    // cleanup
    rcmIfClose();

}

void init(rcmIfType rcmIf, std::string dev_path)
{
    int r;

    // initialize interface
    r = rcmIfInit(rcmIf, (char*) dev_path.c_str());
    error_check(r, "Initialization Failed");

    // make sure RCM is awake
    r = rcmSleepModeSet(RCM_SLEEP_MODE_ACTIVE);
    error_check(r, "Timeout waiting for sleep mode set");

    // make sure correct internal opmode is set
    r = rcmOpModeSet(RCM_OPMODE_RCM);
    error_check(r, "Timeout waiting for opmode set");

    // execute Built-In Test
    int status;
    r = rcmBit(&status);
    error_check(r, "Timeout waiting for test");
    error_check(status, "Built-In Test Failed");

    // get configuration from RCM
    rcmConfiguration config;
    r = rcmConfigGet(&config);
    error_check(r, "Timeout watiing for configuration");
    print_configuration(&config);

    // get status information
    rcmMsg_GetStatusInfoConfirm statusInfo;
    r = rcmStatusInfoGet(&statusInfo);
    error_check(r, "Timeout waiting for status information");
    print_status(&statusInfo);
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
