#include <ros/ros.h>

#include <pulson_ros/P4xx/RangeNet/rcmIf.h>
#include <pulson_ros/P4xx/RangeNet/rcm.h>
#include <pulson_ros/P4xx/RangeNet/TDMA/rn.h>

#include <pulson_ros/RangeNetNDBEntry.h>
#include <pulson_ros/RangeNetNDB.h>
#include <pulson_ros/RangeMeasurement.h>
#include <pulson_ros/EchoedRangeMeasurement.h>

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <vector>

void init(rcmIfType rcmIf, std::string dev_path);
void run();
void set_rcm_config(rcmConfiguration *rcmConfig, int pii);
void set_range_net_config(rnConfiguration *rnConfig);
void set_tdma_config(rnTDMAConfiguration *rnTDMAConfig);
void set_slotmap_config(rnMsg_SetTDMASlotmapRequest *rnTdmaSlotMap);
void to_msg(rcmMsg_FullRangeInfo *rangeInfo, pulson_ros::RangeMeasurement *rm);
void to_msg(rcmMsg_EchoedRangeInfo *echoedInfo, pulson_ros::EchoedRangeMeasurement *erm);
void to_msg(rnMsg_GetFullNeighborDatabaseConfirm *ndbInfo, pulson_ros::RangeNetNDB *ndb);
void parse_slot_map_file(std::string f, rnMsg_SetTDMASlotmapRequest *rnTdmaSlotMap);
void cleanup();
void print_status(rcrmMsg_GetStatusInfoConfirm *statusInfo);
void print_rcm_configuration(rcmConfiguration *config);
void print_rn_configuration(rnConfiguration *rnConfig);
void print_slotmap(rnMsg_SetTDMASlotmapRequest *rnTdmaSlotMap);
void error_check(int r, const char *msg);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tdma_node");
    ros::NodeHandle nh("~");

    // parameters
    int node_id;
    int pii;
    std::string slot_map_file;
    std::string dev_path;
    nh.param("node_id", node_id, 100);
    nh.param("pulse_integration_index", pii, 4);
    nh.param("slot_map_file", slot_map_file, std::string(""));

    //interface
    rcmIfType rcmIf;

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

    // configurations
    rcmConfiguration rcmConfig;
    rnConfiguration rnConfig;
    rnTDMAConfiguration rnTDMAConfig;

    // slot map
    rnMsg_SetTDMASlotmapRequest rnTdmaSlotMap;
    parse_slot_map_file(slot_map_file, &rnTdmaSlotMap);
    print_slotmap(&rnTdmaSlotMap);

    // initialize
    init(rcmIf, dev_path);

    // configure
    set_rcm_config(&rcmConfig, pii);
    set_range_net_config(&rnConfig);
    set_tdma_config(&rnTDMAConfig);
    set_slotmap_config(&rnTdmaSlotMap);

    // P4xx messages datastructures
    rcmMsg_FullRangeInfo rangeInfo;
    rcmMsg_EchoedRangeInfo echoedInfo;
	  rnMsg_GetFullNeighborDatabaseConfirm ndbInfo;

    // start radio
    run();

    // publishers
    ros::Publisher range_pub;
    range_pub = nh.advertise<pulson_ros::RangeMeasurement>("ranges", 1000);
    ros::Publisher echo_pub;
    echo_pub = nh.advertise<pulson_ros::EchoedRangeMeasurement>("echo_ranges", 1000);

    int r;

    while (ros::ok())
    {
        // get data
        r = rcmInfoWithEchoGet(&rangeInfo, &echoedInfo, &ndbInfo);

        // ROS message
        pulson_ros::RangeMeasurement rm;
        to_msg(&rangeInfo, &rm);

        // ROS message
        pulson_ros::EchoedRangeMeasurement erm;
        to_msg(&echoedInfo, &erm);

        // publish
        range_pub.publish(rm);
        echo_pub.publish(erm);
    }

    cleanup();


}

void init(rcmIfType rcmIf, std::string dev_path)
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

}

void set_rcm_config(rcmConfiguration *rcmConfig, int pii)
{
    int r;

    // Check RCM configuration
    r = rcmConfigGet(rcmConfig);
    error_check(r, "Timeout waiting for rcmConfig confirm");

    // Set RCM configuration
    rcmConfig->integrationIndex = pii;

	rcmConfig->flags |= RCM_FLAG_ENABLE_ECHO_LAST_RANGE; // Set ELR
	rcmConfig->flags &= ~RCM_SEND_SCANINFO_PKTS; // Clear Scans
	rcmConfig->flags &= ~RCM_SEND_FULL_SCANINFO_PKTS; // Clear Full Scans
	rcmConfig->flags |= RCM_DISABLE_CRE_RANGES;

    // set persist flag
    rcmConfig->persistFlag |= RCRM_PERSIST_ALL;

    r = rcmConfigSet(rcmConfig);
    error_check(r, "Time out waiting for rcmConfig confirm");

    print_rcm_configuration(rcmConfig);

}

void set_range_net_config(rnConfiguration *rnConfig)
{
    int r;

    // Check RN configuration
    r = rnConfigGet(rnConfig);
    error_check(r, "Time out waiting for rnConfig confirm");

	// Set RangeNet configuration
    rnConfig->maxNeighborAgeMs = 4 * 1000;
	rnConfig->autosendNeighborDbUpdateIntervalMs = 65535;

	rnConfig->rnFlags &= ~RN_CONFIG_FLAG_DO_NOT_RANGE_TO_ME;
    rnConfig->rnFlags &= ~RN_CONFIG_FLAG_NEIGHBOR_DATABASE_FILTERED_RANGE;
    rnConfig->rnFlags |= RN_CONFIG_FLAG_ECHO_LAST_RANGE;

    rnConfig->networkSyncMode = RN_NETWORK_SYNC_MODE_TDMA;

    rnConfig->autosendType &= ~RN_AUTOSEND_RANGEINFO_FLAGS_MASK; // reset
    rnConfig->autosendType |= RN_AUTOSEND_RANGEINFO_SUCCESSFUL;
    rnConfig->autosendType &= ~RN_AUTOSEND_NEIGHBOR_DATABASE_FLAGS_MASK; // reset

    r = rnConfigSet(rnConfig);
    error_check(r, "Time out waiting for rnConfig confirm");

    print_rn_configuration(rnConfig);

}

void set_tdma_config(rnTDMAConfiguration *rnTDMAConfig)
{
    int r;

    // Check TDMA configuration
    r = rnTdmaConfigGet(rnTDMAConfig);
    error_check(r, "Time out waiting for rnTdmaConfig confirm");

	// Set TDMA configuration
    rnTDMAConfig->maxRequestDataSize = 0;
    rnTDMAConfig->maxResponseDataSize = 0;

    r = rnTdmaConfigSet(rnTDMAConfig);
    error_check(r, "Time out waiting for rnTdmaConfig confirm");

}

void set_slotmap_config(rnMsg_SetTDMASlotmapRequest *rnTdmaSlotMap)
{
    int r;

    // Set Slotmap metadata
    rnTdmaSlotMap->msgType = RN_SET_TDMA_SLOTMAP_REQUEST;
    rnTdmaSlotMap->slotmapFlags &= ~RN_TDMA_SLOTMAP_FLAG_MODIFY;
    rnTdmaSlotMap->persistFlag = 0; // do not write to flash
    r = rnTdmaSlotMapSet(rnTdmaSlotMap);
    error_check(r, "Time out waiting for rnTdmaSlotMapSet confirm");

}

void to_msg(rnMsg_GetFullNeighborDatabaseConfirm *ndbInfo, pulson_ros::RangeNetNDB *ndb)
{
    // build message
    ndb->message_id = ndbInfo->msgId;
    ndb->number_of_neighbor_entries = ndbInfo->numNeighborEntries;
    for (int i = 0; i < ndbInfo->numNeighborEntries; i++)
    {
        // ROS NDB entry
        pulson_ros::RangeNetNDBEntry entry;

        // build
        entry.node_id = ndbInfo->neighbors[i].nodeId;
        entry.range_status = ndbInfo->neighbors[i].rangeStatus;
        entry.antenna_mode = ndbInfo->neighbors[i].antennaMode;
        entry.stopwatch_time = ndbInfo->neighbors[i].stopwatchTime;
        entry.range = ndbInfo->neighbors[i].rangeMm;
        entry.range_error = ndbInfo->neighbors[i].rangeErrorEstimate;
        entry.range_velocity = ndbInfo->neighbors[i].rangeVelocity;
        entry.range_measurement_type = ndbInfo->neighbors[i].rangeMeasurementType;
        entry.flags = ndbInfo->neighbors[i].flags;
        entry.stats_age_ms = ndbInfo->neighbors[i].statsAgeMs;
        entry.range_update_timestamp_ms = ndbInfo->neighbors[i].rangeUpdateTimestampMs;
        entry.last_heard_timestamp_ms = ndbInfo->neighbors[i].lastHeardTimestampMs;
        entry.added_to_ndb_timestamp_ms = ndbInfo->neighbors[i].addedToNDBTimestampMs;

        // save
        ndb->database.push_back(entry);
    }
}

void to_msg(rcmMsg_FullRangeInfo *rangeInfo, pulson_ros::RangeMeasurement *rm)
{

    rm->message_id = rangeInfo->msgId;
    rm->responder_id = rangeInfo->responderId;
    rm->range_status = rangeInfo->rangeStatus;
    rm->antenna_mode = rangeInfo->antennaMode;
    rm->stopwatch_time = rangeInfo->stopwatchTime;
    rm->precision_range = rangeInfo->precisionRangeMm;
    rm->coarse_range = rangeInfo->coarseRangeMm;
    rm->filtered_range = rangeInfo->filteredRangeMm;
    rm->precision_range_error = rangeInfo->precisionRangeErrEst;
    rm->coarse_range_error = rangeInfo->coarseRangeErrEst;
    rm->filtered_range_error = rangeInfo->filteredRangeErrEst;
    rm->filtered_velocity = rangeInfo->filteredRangeVel;
    rm->filtered_velocity_error = rangeInfo->filteredRangeVelErrEst;
    rm->timestamp = rangeInfo->timestamp;

}

void to_msg(rcmMsg_EchoedRangeInfo *echoedInfo, pulson_ros::EchoedRangeMeasurement *erm)
{
    erm->message_id = echoedInfo->msgId;
    erm->requester_id = echoedInfo->requesterId;
    erm->responder_id = echoedInfo->responderId;
    erm->precision_range = echoedInfo->precisionRangeMm;
    erm->precision_range_error = echoedInfo->precisionRangeErrEst;
    erm->timestamp = echoedInfo->timestamp;
}

void parse_slot_map_file(std::string f, rnMsg_SetTDMASlotmapRequest *rnTdmaSlotMap)
{
    // open file
    std::fstream fs;
    fs.open(f.c_str());
    error_check(!fs.is_open(), "Slotmap file not found");
    YAML::Node map = YAML::LoadFile(f.c_str());
    assert(map.IsSequence());

    // populate fields
    rnTdmaSlotMap->numSlots = (uint8_t) map.size();
    for (int i = 0; i < map.size(); i++)
    {
        rnTdmaSlotMap->slots[i].slotType = (uint8_t) map[i]["type"].as<int>();
        rnTdmaSlotMap->slots[i].slotNumber = (uint8_t) map[i]["number"].as<int>();
        rnTdmaSlotMap->slots[i].flags = (uint16_t) map[i]["flags"].as<int>();
        rnTdmaSlotMap->slots[i].integrationIndex = (uint8_t) map[i]["integration_index"].as<int>();
        rnTdmaSlotMap->slots[i].antennaMode = (uint8_t) map[i]["antenna_mode"].as<int>();
        rnTdmaSlotMap->slots[i].codeChannel = (uint8_t) map[i]["channel"].as<int>();
        rnTdmaSlotMap->slots[i].requesterId = (uint32_t) map[i]["requester_id"].as<int>();
        rnTdmaSlotMap->slots[i].responderId = (uint32_t) map[i]["responder_id"].as<int>();
        rnTdmaSlotMap->slots[i].requestedDurationMicroseconds = (uint32_t) map[i]["duration"].as<int>();
    }

    // close file
    fs.close();
}

void run()
{

    int r;

    // Put into RangeNet Mode
    r = rcmOpModeSet(RCRM_OPMODE_RN);
    error_check(r, "Time out waiting for opmode set");

    // Put into active mode
    r = rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE);
    error_check(r, "Time out waiting for sleep mode set");

    // Flush
    rcmIfFlush();

}

void cleanup()
{
    int r;

    // Put into sleep mode
    r = rcmSleepModeSet(RCRM_SLEEP_MODE_IDLE);
    error_check(r, "Time out waiting for sleep mode set");

    // Put back into RCM Mode
    r = rcmOpModeSet(RCRM_OPMODE_RCM);
    error_check(r, "Time out waiting for opmode set");

    // Flush
    rcmIfFlush();

    // cleanup
    rcmIfClose();

    // shutdown
    ros::requestShutdown();

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

    ROS_INFO("[tdma_node] %s", buf);

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

    ROS_INFO("[tdma_node] %s", buf);
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

    ROS_INFO("[tdma_node] %s", buf);
}

void print_slotmap(rnMsg_SetTDMASlotmapRequest *rnTdmaSlotMap)
{
    char buf[4096];
    sprintf(buf, "\n\tSlotmap: \n");
    sprintf(buf, "%s\t\t Entries: %d\n", buf, rnTdmaSlotMap->numSlots);
    sprintf(buf, "%s\t\t Flags: %d\n", buf, rnTdmaSlotMap->slotmapFlags);
    sprintf(buf, "%s\t\t Persist Flags: %d\n", buf, rnTdmaSlotMap->persistFlag);
    for(int i = 0; i < rnTdmaSlotMap->numSlots; i++)
    {
        sprintf(buf, "%s\t\t\t Slot Type: %d\n", buf, rnTdmaSlotMap->slots[i].slotType);
        sprintf(buf, "%s\t\t\t Slot Number: %d\n", buf, rnTdmaSlotMap->slots[i].slotNumber);
        sprintf(buf, "%s\t\t\t Flags: 0x%X\n", buf, rnTdmaSlotMap->slots[i].flags);
        sprintf(buf, "%s\t\t\t Integration Index: %d\n", buf, rnTdmaSlotMap->slots[i].integrationIndex);
        sprintf(buf, "%s\t\t\t Antenna Mode: %d\n", buf, rnTdmaSlotMap->slots[i].antennaMode);
        sprintf(buf, "%s\t\t\t Code Channel: %d\n", buf, rnTdmaSlotMap->slots[i].codeChannel);
        sprintf(buf, "%s\t\t\t Requester ID: %d\n", buf, rnTdmaSlotMap->slots[i].requesterId);
        sprintf(buf, "%s\t\t\t Responder ID: %d\n", buf, rnTdmaSlotMap->slots[i].responderId);
        sprintf(buf, "%s\t\t\t Request Duration uS: %d\n\n", buf, rnTdmaSlotMap->slots[i].requestedDurationMicroseconds);
    }

    ROS_INFO("[tdma_node] %s", buf);
}

void error_check(int r, char const *msg)
{
    if (r != 0)
    {
        ROS_ERROR("[tdma_node] %s\n", msg);
        ros::requestShutdown();
    }

}
