#include <ros/ros.h>

#include <pulson_ros/P4xx/RangeNet/rcmIf.h>
#include <pulson_ros/P4xx/RangeNet/rcm.h>
#include <pulson_ros/P4xx/RangeNet/ALOHA/rn.h>

#include <pulson_ros/RangeNetNDBEntry.h>
#include <pulson_ros/RangeNetNDB.h>

void init(rcmIfType &rcmIf, std::string &dev_path,
        rcmConfiguration *rcmConfig,
        rnConfiguration *rnConfig,
        rnALOHAConfiguration *rnAlohaConfig);
void cleanup();
void configure_radio(
        int rate,
        rcmConfiguration *rcmConfig,
        rnConfiguration *rnConfig,
        rnALOHAConfiguration *rnAlohaConfig);
void print_status(rcrmMsg_GetStatusInfoConfirm *statusInfo);
void print_rcm_configuration(rcmConfiguration *config);
void print_rn_configuration(rnConfiguration *rnConfig);
void print_rn_aloha_configuration(rnALOHAConfiguration *rnAlohaConfig);
void error_check(int r, const char *msg);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "aloha_node");
    ros::NodeHandle nh("~");

    // parameters
    int rate;
    int node_id;
    std::string dev_path;

    // interface
    rcmIfType rcmIf;

    nh.param("rate", rate, 10);
    nh.param("node_id", node_id, 100);

    if (nh.getParam("ip_address", dev_path))
    {
       rcmIf = rcmIfIp;
    }
    else if (nh.getParam("serial_dev", dev_path))
    {
      rcmIf = rcmIfSerial;
    }
    else if (nh.getParam("dev_path", dev_path))
    {
      rcmIf = rcmIfUsb;
    }
    else
    {
      std::stringstream ss;
      ss << node_id;
      dev_path = ss.str();

      rcmIf = rcmIfUsb;
    }

    // publisher
    ros::Publisher database_pub = nh.advertise<pulson_ros::RangeNetNDB>("range_net_db", 100);

    // configurations
    rcmConfiguration rcmConfig;
    rnConfiguration rnConfig;
	  rnALOHAConfiguration rnAlohaConfig;

    // initialize
    init(rcmIf, dev_path, &rcmConfig, &rnConfig, &rnAlohaConfig);

    // configure radio
    configure_radio(rate, &rcmConfig, &rnConfig, &rnAlohaConfig);

    // P4xx messages datastructures
    rcmMsg_FullRangeInfo rangeInfo;
	  rnMsg_GetFullNeighborDatabaseConfirm ndbInfo;

    int r;

    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        // get data
        r = rcmInfoGet(&rangeInfo, &ndbInfo);

        // ROS message
        pulson_ros::RangeNetNDB ndb;

        // build message
        ndb.host_node = node_id;
        ndb.message_id = ndbInfo.msgId;
        ndb.number_of_neighbor_entries = ndbInfo.numNeighborEntries;
        for (int i = 0; i < ndbInfo.numNeighborEntries; i++)
        {
            // ROS NDB entry
            pulson_ros::RangeNetNDBEntry entry;

            // build
            entry.node_id = ndbInfo.neighbors[i].nodeId;
            entry.range_status = ndbInfo.neighbors[i].rangeStatus;
            entry.antenna_mode = ndbInfo.neighbors[i].antennaMode;
            entry.stopwatch_time = ndbInfo.neighbors[i].stopwatchTime;
            entry.range = ndbInfo.neighbors[i].rangeMm;
            entry.range_error = ndbInfo.neighbors[i].rangeErrorEstimate;
            entry.range_velocity = ndbInfo.neighbors[i].rangeVelocity;
            entry.range_measurement_type = ndbInfo.neighbors[i].rangeMeasurementType;
            entry.flags = ndbInfo.neighbors[i].flags;
            entry.stats_age_ms = ndbInfo.neighbors[i].statsAgeMs;
            entry.range_update_timestamp_ms = ndbInfo.neighbors[i].rangeUpdateTimestampMs;
            entry.last_heard_timestamp_ms = ndbInfo.neighbors[i].lastHeardTimestampMs;
            entry.added_to_ndb_timestamp_ms = ndbInfo.neighbors[i].addedToNDBTimestampMs;

            // save
            ndb.database.push_back(entry);
        }

        // publish
        database_pub.publish(ndb);
    }

    cleanup();


}

void init(rcmIfType &rcmIf, std::string &dev_path,
        rcmConfiguration *rcmConfig,
        rnConfiguration *rnConfig,
        rnALOHAConfiguration *rnAlohaConfig)
{
    int r;

    ROS_INFO("Initialization...");

    // initialize interface
    r = rcmIfInit(rcmIf, (char*) dev_path.c_str());
    error_check(r, "Initialization failed");

    // put in idle mode during configuration
    r = rcmSleepModeSet(RCRM_SLEEP_MODE_IDLE);
    error_check(r, "Time out waiting for sleep mode set");

    // set to RCM mode
    r = rcmOpModeSet(RCRM_OPMODE_RCM);
    error_check(r, "Time out waiting for opmode set");

    // execute Built-In Test
    int status;
    r = rcmBit(&status);
    error_check(r, "Timeout waiting for BIT");
    error_check(status, "Built-in test failed");

    // get status information from RCM
    rcrmMsg_GetStatusInfoConfirm statusInfo;
    r = rcmStatusInfoGet(&statusInfo);
    error_check(r, "Timeout waiting for status information");
    print_status(&statusInfo);

    // Check RCM configuration
    r = rcmConfigGet(rcmConfig);
    error_check(r, "Timeout waiting for rcmConfig confirm");
    print_rcm_configuration(rcmConfig);

    // Check RN configuration
    r = rnConfigGet(rnConfig);
    error_check(r, "Time out waiting for rnConfig confirm");
    print_rn_configuration(rnConfig);

    // Check ALOHA configuration
    r = rnAlohaConfigGet(rnAlohaConfig);
    error_check(r, "Time out waiting for rnAlohaConfig confirm");
    print_rn_aloha_configuration(rnAlohaConfig);

}

void cleanup()
{
    int r;

    // return to idle
    r = rcmSleepModeSet(RCRM_SLEEP_MODE_IDLE);
    error_check(r, "Time out waiting for sleep mode set");

    // return to RCM mode
    r = rcmOpModeSet(RCRM_OPMODE_RCM);
    error_check(r, "Time out waiting for opmode set");

    // Flush any pending messages
    rcmIfFlush();

    // cleanup
    rcmIfClose();

}

void configure_radio(
        int rate,
        rcmConfiguration *rcmConfig,
        rnConfiguration *rnConfig,
        rnALOHAConfiguration *rnAlohaConfig)
{
    ROS_INFO("Configuring...");

    int r;

    // Set RCM configuration
	  rcmConfig->flags &= ~RCM_FLAG_ENABLE_ECHO_LAST_RANGE; // Set ELR
	  rcmConfig->flags &= ~RCM_SEND_SCANINFO_PKTS; // Clear Scans
	  rcmConfig->flags &= ~RCM_SEND_FULL_SCANINFO_PKTS; // Clear Full Scans
	  rcmConfig->flags |= RCM_DISABLE_CRE_RANGES; // Disable CRE Ranges (note: setting the bit disables the sending of CREs)
    r = rcmConfigSet(rcmConfig);
    error_check(r, "Time out waiting for rcmConfig confirm");
    print_rcm_configuration(rcmConfig);

	  // Moving on to RangeNet configuration
    rnConfig->autosendType &= ~RN_AUTOSEND_RANGEINFO_FLAGS_MASK; // clear
    rnConfig->autosendType &= ~RN_AUTOSEND_NEIGHBOR_DATABASE_FLAGS_MASK; // clear
    rnConfig->autosendType |= RN_AUTOSEND_NEIGHBOR_DATABASE_FULL;
	  rnConfig->rnFlags &= ~RN_CONFIG_FLAG_DO_NOT_RANGE_TO_ME; // Ensure Do Not Range To Me flag is not set
	  rnConfig->autosendNeighborDbUpdateIntervalMs = (int) ((1 / rate) * 1000);
    r = rnConfigSet(rnConfig);
    error_check(r, "Time out waiting for rnConfig confirm");
    print_rn_configuration(rnConfig);

    // Finally set up ALOHA configuration
    rnAlohaConfig->flags &= 0;
    rnAlohaConfig->flags |= RN_ALOHA_CONFIG_FLAG_USE_ACC;
    rnAlohaConfig->maxRequestDataSize = 0; // Let's not have any data this time around
    rnAlohaConfig->maxResponseDataSize = 0;
    r = rnAlohaConfigSet(rnAlohaConfig);
    error_check(r, "Time out waiting for rnAlohaConfig confirm");
    print_rn_aloha_configuration(rnAlohaConfig);

    // Put into RangeNet Mode
    r = rcmOpModeSet(RCRM_OPMODE_RN);
    error_check(r, "Time out waiting for opmode set");

    // Put into active mode
    r = rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE);
    error_check(r, "Time out waiting for sleep mode set");

    // Flush any pending messages
    rcmIfFlush();

}

void print_status(rcrmMsg_GetStatusInfoConfirm *statusInfo)
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

    ROS_INFO("[aloha_node] %s", buf);

}

void print_rcm_configuration(rcmConfiguration *rcmConfig)
{
    char buf[1024];
    sprintf(buf, "\n\tRCM Configuration:\n");
    sprintf(buf, "%s\t\tNode ID: %d\n", buf, rcmConfig->nodeId);
    sprintf(buf, "%s\t\tIntegration Index: %d\n", buf, rcmConfig->integrationIndex);
    sprintf(buf, "%s\t\tAntenna Mode: %d\n", buf, rcmConfig->antennaMode);
    sprintf(buf, "%s\t\tCode Channel: %d\n", buf, rcmConfig->codeChannel);
    sprintf(buf, "%s\t\tElectrical Delay PsA: %d\n", buf, rcmConfig->electricalDelayPsA);
    sprintf(buf, "%s\t\tElectrical Delay PsB: %d\n", buf, rcmConfig->electricalDelayPsB);
    sprintf(buf, "%s\t\tFlags: 0x%X\n", buf, rcmConfig->flags);
    sprintf(buf, "%s\t\ttxGain: %d\n", buf, rcmConfig->txGain);

    ROS_INFO("[aloha_node] %s", buf);
}

void print_rn_configuration(rnConfiguration *rnConfig)
{
    char buf[1024];
    sprintf(buf, "\n\tRN Configuration: \n");
    sprintf(buf, "%s\t\tMax Neighbor Age Ms: %d\n", buf,
            rnConfig->maxNeighborAgeMs);
    sprintf(buf, "%s\t\tAuto Send NDB Update Interval Ms: %d\n", buf,
            rnConfig->autosendNeighborDbUpdateIntervalMs);
    sprintf(buf, "%s\t\tRN Flags: 0x%X\n", buf, rnConfig->rnFlags);
    sprintf(buf, "%s\t\tNetwork Sync Mode: %d\n", buf, rnConfig->networkSyncMode);
    sprintf(buf, "%s\t\tAuto Send Type: 0x%X\n", buf, rnConfig->autosendType);

    sprintf(buf, "%s\t\tDefault If: %d\n", buf, rnConfig->defaultIf);
    sprintf(buf, "%s\t\tDefault If Addr1: %d\n", buf, rnConfig->defaultIfAddr1);
    sprintf(buf, "%s\t\tDefault If Addr2: %d\n", buf, rnConfig->defaultIfAddr2);

    ROS_INFO("[aloha_node] %s", buf);
}

void print_rn_aloha_configuration(rnALOHAConfiguration *rnAlohaConfig)
{
    char buf[1024];
    sprintf(buf, "\n\tRN ALOHA Configuration: \n");
    sprintf(buf, "%s\t\tminTimeBetweenTxMs: %d\n", buf, rnAlohaConfig->minTimeBetweenTxMs);
    sprintf(buf, "%s\t\tmaxTimeBetweenTxMs: %d\n", buf, rnAlohaConfig->maxTimeBetweenTxMs);
    sprintf(buf, "%s\t\tmaxRequestDataSize: %d\n", buf, rnAlohaConfig->maxRequestDataSize);
    sprintf(buf, "%s\t\tmaxResponseDataSize: %d\n", buf, rnAlohaConfig->maxResponseDataSize);
    sprintf(buf, "%s\t\tFlags: 0x%X\n", buf, rnAlohaConfig->flags);
    sprintf(buf, "%s\t\taccKfactor: %d\n", buf, rnAlohaConfig->accKfactor);

    ROS_INFO("[aloha_node] %s", buf);
}

void error_check(int r, char const *msg)
{
    if (r != 0)
    {
        ROS_ERROR("[aloha_node] %s\n", msg);
        ros::requestShutdown();
    }

}
