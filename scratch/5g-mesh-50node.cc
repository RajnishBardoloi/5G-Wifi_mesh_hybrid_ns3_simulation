// Copyright (c) 2024 5G-Mesh Integration Project
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * \file 5g-mesh-50node.cc
 * \brief 50-Node Hybrid 5G-Mesh Architecture with Dual Gateways
 *
 * Phase 3: Scaling Implementation
 *
 * This simulation scales the hybrid 5G-Mesh architecture to:
 * - 48 mesh nodes in 8x6 grid (40m spacing)
 * - 2 gateways for load distribution (center-split placement)
 * - 6 representative source nodes (1 per row)
 * - 3 QoS classes per source (18 total flows)
 *
 * Topology:
 * +----+----+----+----+----+----+----+----+
 * | o  | S  | o  | o  | o  | o  | S  | o  |  Row 5
 * +----+----+----+----+----+----+----+----+
 * | o  | S  | o  | o  | o  | o  | S  | o  |  Row 4
 * +----+----+----+----+----+----+----+----+
 * | o  | o  |GW1 | o  | o  |GW2 | o  | o  |  Row 3 (Gateways)
 * +----+----+----+----+----+----+----+----+
 * | o  | S  | o  | o  | o  | o  | S  | o  |  Row 2
 * +----+----+----+----+----+----+----+----+
 * | o  | S  | o  | o  | o  | o  | S  | o  |  Row 1
 * +----+----+----+----+----+----+----+----+
 * | o  | S  | o  | o  | o  | o  | S  | o  |  Row 0
 * +----+----+----+----+----+----+----+----+
 *   0    1    2    3    4    5    6    7   (columns)
 *
 * S = Source node, GW = Gateway, o = mesh node
 * Left half (cols 0-3) -> GW1, Right half (cols 4-7) -> GW2
 *
 * Traffic Classes:
 * - Critical (QCI 1, Port 5050): 500 kbps control traffic
 * - Video (QCI 4, Port 5150): 2 Mbps video stream
 * - Best Effort (QCI 9, Port 5250): 2 Mbps bulk data
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("5gMesh50Node");

// ========================================================================
// QoS PORT DEFINITIONS
// ========================================================================
const uint16_t CRITICAL_PORT = 5050;
const uint16_t VIDEO_PORT = 5150;
const uint16_t BEST_EFFORT_PORT = 5250;

// ========================================================================
// DSCP/ToS VALUES FOR WIFI EDCA QOS DIFFERENTIATION
// ========================================================================
const uint8_t TOS_CRITICAL = 0xB8;   // DSCP EF (46) -> AC_VO (Voice, CWmin=3, highest)
const uint8_t TOS_VIDEO = 0x80;      // DSCP CS4 (32) -> AC_VI (Video, CWmin=7, medium)
const uint8_t TOS_BESTEFFORT = 0x00; // DSCP CS0 (0) -> AC_BE (Best Effort, CWmin=15, lowest)

// ========================================================================
// QoS MAPPING (Paper Algorithm 1: Mesh Priority -> 5G QFI)
// ========================================================================
// The QoS differentiation happens at the 5G layer via dedicated EPS bearers:
// - Critical traffic (Port 5050) -> GBR_CONV_VOICE (QCI 1, highest priority)
// - Video traffic (Port 5150) -> GBR_NON_CONV_VIDEO (QCI 4, medium priority)
// - Best Effort (Port 5250) -> NGBR_VIDEO_TCP_DEFAULT (QCI 9, lowest priority)
//
// The 5G QoS scheduler (NrMacSchedulerTdmaQos) prioritizes GBR bearers over
// Non-GBR bearers, giving Critical and Video traffic preferential treatment.

// ========================================================================
// HELPER FUNCTION: Get node index from grid position
// ========================================================================
uint32_t GetNodeIndex(uint32_t col, uint32_t row, uint32_t gridWidth)
{
    return row * gridWidth + col;
}



// ========================================================================
// MAIN FUNCTION
// ========================================================================
int main(int argc, char* argv[])
{
    // ====================================================================
    // SECTION 1: COMMAND-LINE PARAMETERS
    // ====================================================================

    Time simTime = Seconds(40.0);
    Time appStartTime = Seconds(15.0);  // Allow mesh peering time (increased for large grid)

    // Grid parameters
    uint32_t gridWidth = 8;
    uint32_t gridHeight = 6;
    double meshSpacing = 30.0;  // meters (reduced for better connectivity)

    // NR parameters
    uint16_t numerology = 0;
    double centralFrequency = 4e9;
    double bandwidth = 20e6;
    double gnbTxPower = 43;

    // Traffic parameters (reduced for 50-node scale)
    uint32_t criticalRate = 50;    // kbps (reduced to avoid congestion)
    uint32_t videoRate = 200;      // kbps
    uint32_t beRate = 200;         // kbps

    bool logging = false;
    std::string simTag = "5g-mesh-50node";

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue("appStartTime", "App start time - allow mesh convergence (seconds)", appStartTime);
    cmd.AddValue("gridWidth", "Grid width (columns)", gridWidth);
    cmd.AddValue("gridHeight", "Grid height (rows)", gridHeight);
    cmd.AddValue("meshSpacing", "Mesh node spacing (meters)", meshSpacing);
    cmd.AddValue("criticalRate", "Critical traffic rate (kbps)", criticalRate);
    cmd.AddValue("videoRate", "Video traffic rate (kbps)", videoRate);
    cmd.AddValue("beRate", "Best effort traffic rate (kbps)", beRate);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("simTag", "Output filename tag", simTag);
    cmd.Parse(argc, argv);

    if (logging)
    {
        LogComponentEnable("5gMesh50Node", LOG_LEVEL_INFO);
        LogComponentEnable("HwmpProtocol", LOG_LEVEL_DEBUG);
    }

    uint32_t numMeshNodes = gridWidth * gridHeight;
    uint32_t numGateways = 2;
    uint32_t totalNodes = numMeshNodes + numGateways;

    // Print configuration
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     5G-Mesh Hybrid Architecture - 50 Node Scaling               ║\n";
    std::cout << "║                    Phase 3 Implementation                        ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Topology:                                                        ║\n";
    std::cout << "║   Grid: " << gridWidth << "x" << gridHeight << " (" << numMeshNodes << " mesh nodes)                                ║\n";
    std::cout << "║   Gateways: " << numGateways << " (center-split placement)                          ║\n";
    std::cout << "║   Total nodes: " << totalNodes << "                                               ║\n";
    std::cout << "║   Mesh Spacing: " << meshSpacing << "m                                             ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ QoS Configuration (Paper Algorithm 1 - Mesh-to-5G Mapping):      ║\n";
    std::cout << "║   5G Bearer Mapping (via Port-based TFT):                        ║\n";
    std::cout << "║   Critical: Port 5050 -> QCI 1 (GBR_CONV_VOICE, highest)         ║\n";
    std::cout << "║   Video:    Port 5150 -> QCI 4 (GBR_NON_CONV_VIDEO, medium)      ║\n";
    std::cout << "║   BestEff:  Port 5250 -> QCI 9 (NGBR, lowest priority)           ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Traffic:                                                         ║\n";
    std::cout << "║   Source nodes: 6 (1 per row, alternating left/right)            ║\n";
    std::cout << "║   Flows per source: 3 (Critical, Video, Best Effort)             ║\n";
    std::cout << "║   Total flows: 18                                                ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Simulation Time: " << simTime.GetSeconds() << " seconds                                      ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";

    // ====================================================================
    // SECTION 2: NODE CREATION
    // ====================================================================

    std::cout << "[1/12] Creating nodes...\n";

    // Create gateway nodes (2 gateways)
    NodeContainer gatewayNodes;
    gatewayNodes.Create(numGateways);

    // Create mesh-only nodes (48 nodes in 8x6 grid)
    NodeContainer meshOnlyNodes;
    meshOnlyNodes.Create(numMeshNodes);

    // All mesh-capable nodes (gateways + mesh nodes)
    NodeContainer allMeshNodes;
    allMeshNodes.Add(gatewayNodes);
    allMeshNodes.Add(meshOnlyNodes);

    // 5G gNB node
    NodeContainer gnbNode;
    gnbNode.Create(1);

    // MEC server
    NodeContainer mecServer;
    mecServer.Create(1);

    std::cout << "   Gateway nodes: " << numGateways << "\n";
    std::cout << "   Mesh-only nodes: " << numMeshNodes << "\n";
    std::cout << "   gNB nodes: 1\n";
    std::cout << "   MEC server: 1\n";
    std::cout << "   Total: " << (numGateways + numMeshNodes + 2) << "\n\n";

    // ====================================================================
    // SECTION 3: NODE POSITIONING
    // ====================================================================

    std::cout << "[2/12] Setting up node positions (8x6 grid)...\n";

    MobilityHelper mobility;

    // Gateway positions (center-split: column 2 and column 5, row 3)
    Ptr<ListPositionAllocator> gwPositions = CreateObject<ListPositionAllocator>();
    double gw1X = 2 * meshSpacing;  // Column 2: 80m
    double gw1Y = 3 * meshSpacing;  // Row 3: 120m
    double gw2X = 5 * meshSpacing;  // Column 5: 200m
    double gw2Y = 3 * meshSpacing;  // Row 3: 120m

    gwPositions->Add(Vector(gw1X, gw1Y, 1.5));  // Gateway 1
    gwPositions->Add(Vector(gw2X, gw2Y, 1.5));  // Gateway 2

    mobility.SetPositionAllocator(gwPositions);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gatewayNodes);

    std::cout << "   Gateway 1: (" << gw1X << ", " << gw1Y << ", 1.5) - serves left half\n";
    std::cout << "   Gateway 2: (" << gw2X << ", " << gw2Y << ", 1.5) - serves right half\n";

    // Mesh node positions (8x6 grid)
    Ptr<ListPositionAllocator> meshPositions = CreateObject<ListPositionAllocator>();
    for (uint32_t row = 0; row < gridHeight; ++row)
    {
        for (uint32_t col = 0; col < gridWidth; ++col)
        {
            double x = col * meshSpacing;
            double y = row * meshSpacing;
            meshPositions->Add(Vector(x, y, 1.5));
        }
    }

    mobility.SetPositionAllocator(meshPositions);
    mobility.Install(meshOnlyNodes);

    std::cout << "   Mesh nodes: " << numMeshNodes << " nodes in " << gridWidth << "x" << gridHeight << " grid\n";
    std::cout << "   Grid dimensions: " << (gridWidth-1)*meshSpacing << "m x " << (gridHeight-1)*meshSpacing << "m\n";

    // gNB position (center of grid, elevated)
    double gnbX = (gridWidth - 1) * meshSpacing / 2.0;  // 140m
    double gnbY = (gridHeight - 1) * meshSpacing / 2.0; // 100m

    Ptr<ListPositionAllocator> gnbPosition = CreateObject<ListPositionAllocator>();
    gnbPosition->Add(Vector(gnbX, gnbY, 10.0));

    mobility.SetPositionAllocator(gnbPosition);
    mobility.Install(gnbNode);

    std::cout << "   gNB: (" << gnbX << ", " << gnbY << ", 10) - center of grid\n";

    // MEC server position
    Ptr<ListPositionAllocator> mecPosition = CreateObject<ListPositionAllocator>();
    mecPosition->Add(Vector(gnbX + 50, gnbY, 1.5));
    mobility.SetPositionAllocator(mecPosition);
    mobility.Install(mecServer);

    std::cout << "\n";

    // ====================================================================
    // SECTION 4: NR HELPER AND EPC SETUP
    // ====================================================================

    std::cout << "[3/12] Setting up 5G NR infrastructure...\n";

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));

    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));

    // QoS-aware scheduler
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaQos"));

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Antennas
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    std::cout << "   NR helpers configured with QoS scheduler\n\n";

    // ====================================================================
    // SECTION 5: NR SPECTRUM CONFIGURATION
    // ====================================================================

    std::cout << "[4/12] Configuring NR spectrum...\n";

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;

    CcBwpCreator::SimpleOperationBandConf bandConf(
        centralFrequency,
        bandwidth,
        1,
        BandwidthPartInfo::UMi_StreetCanyon
    );
    bandConf.m_numBwp = 1;

    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    std::cout << "   Created 1 BWP at " << centralFrequency/1e9 << " GHz\n\n";

    // ====================================================================
    // SECTION 6: INSTALL NR DEVICES
    // ====================================================================

    std::cout << "[5/12] Installing NR devices...\n";

    // Install gNB device
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNode, allBwps);

    // Install NR UE devices on BOTH gateways
    NetDeviceContainer gatewayNrDevs = nrHelper->InstallUeDevice(gatewayNodes, allBwps);

    // Configure PHY
    double txPowerLinear = pow(10, gnbTxPower / 10);
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("Numerology", UintegerValue(numerology));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(10 * log10(txPowerLinear)));

    // Update configurations
    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }
    for (auto it = gatewayNrDevs.Begin(); it != gatewayNrDevs.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    std::cout << "   gNB device installed\n";
    std::cout << "   Gateway 1 NR UE device installed\n";
    std::cout << "   Gateway 2 NR UE device installed\n\n";

    // ====================================================================
    // SECTION 7: MESH NETWORK SETUP
    // ====================================================================

    std::cout << "[6/12] Setting up 802.11s mesh network...\n";

    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());

    MeshHelper mesh = MeshHelper::Default();
    mesh.SetStackInstaller("ns3::Dot11sStack");
    mesh.SetSpreadInterfaceChannels(MeshHelper::ZERO_CHANNEL);
    mesh.SetNumberOfInterfaces(1);
    mesh.SetMacType("RandomStart", TimeValue(Seconds(0.1)));
    // Use 802.11a - QoS differentiation handled via 5G bearers at gateway
    mesh.SetStandard(WIFI_STANDARD_80211a);

    // Install mesh on all mesh-capable nodes
    NetDeviceContainer meshDevices = mesh.Install(wifiPhy, allMeshNodes);

    std::cout << "   Mesh devices installed on " << meshDevices.GetN() << " nodes\n";
    std::cout << "   (2 gateways + " << numMeshNodes << " mesh-only nodes)\n";
    std::cout << "   Standard: 802.11a (5 GHz)\n";
    std::cout << "   Note: QoS handled at 5G layer via dedicated EPS bearers\n\n";

    // ====================================================================
    // SECTION 8: INTERNET STACK AND IP CONFIGURATION
    // ====================================================================

    std::cout << "[7/12] Configuring Internet stack and IP addresses...\n";

    InternetStackHelper internet;
    internet.Install(mecServer);
    internet.Install(meshOnlyNodes);
    internet.Install(gatewayNodes);

    // Get PGW
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();

    // Connect MEC server to PGW
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gb/s")));
    p2p.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    NetDeviceContainer mecDevices = p2p.Install(pgw, mecServer.Get(0));

    // Assign MEC IP
    Ipv4AddressHelper mecIpHelper;
    mecIpHelper.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer mecIpIface = mecIpHelper.Assign(mecDevices);

    std::cout << "   MEC Server IP: " << mecIpIface.GetAddress(1) << "\n";

    // MEC routing
    Ipv4StaticRoutingHelper routingHelper;
    Ptr<Ipv4StaticRouting> mecRouting = routingHelper.GetStaticRouting(
        mecServer.Get(0)->GetObject<Ipv4>());
    mecRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Assign gateway NR IPs
    Ipv4InterfaceContainer gatewayNrIpIfaces = nrEpcHelper->AssignUeIpv4Address(gatewayNrDevs);

    std::cout << "   Gateway 1 NR IP: " << gatewayNrIpIfaces.GetAddress(0) << "\n";
    std::cout << "   Gateway 2 NR IP: " << gatewayNrIpIfaces.GetAddress(1) << "\n";

    // PGW routing for mesh subnet
    Ptr<Ipv4StaticRouting> pgwRouting = routingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());

    // Route to mesh subnet via both gateways (PGW will use first matching route)
    pgwRouting->AddNetworkRouteTo(Ipv4Address("10.1.1.0"),
                                   Ipv4Mask("255.255.255.0"),
                                   gatewayNrIpIfaces.GetAddress(0),
                                   pgw->GetObject<Ipv4>()->GetInterfaceForDevice(pgw->GetDevice(1)));

    // Assign mesh IPs
    Ipv4AddressHelper meshIpHelper;
    meshIpHelper.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer meshIpIfaces = meshIpHelper.Assign(meshDevices);

    std::cout << "   Gateway 1 mesh IP: " << meshIpIfaces.GetAddress(0) << "\n";
    std::cout << "   Gateway 2 mesh IP: " << meshIpIfaces.GetAddress(1) << "\n";
    std::cout << "   Mesh nodes: 10.1.1.3 - 10.1.1." << (numMeshNodes + 2) << "\n";

    // MEC route to mesh subnet
    mecRouting->AddNetworkRouteTo(Ipv4Address("10.1.1.0"), Ipv4Mask("255.255.255.0"), 1);

    std::cout << "\n";

    // ====================================================================
    // SECTION 9: ROUTING CONFIGURATION
    // ====================================================================

    std::cout << "[8/12] Configuring routing...\n";

    // Gateway 1 routing
    Ptr<Ipv4StaticRouting> gw1Routing = routingHelper.GetStaticRouting(
        gatewayNodes.Get(0)->GetObject<Ipv4>());
    Ptr<Ipv4> gw1Ipv4 = gatewayNodes.Get(0)->GetObject<Ipv4>();
    int32_t gw1NrIdx = gw1Ipv4->GetInterfaceForAddress(gatewayNrIpIfaces.GetAddress(0));
    gw1Routing->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), gw1NrIdx);
    gatewayNodes.Get(0)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));

    // Gateway 2 routing
    Ptr<Ipv4StaticRouting> gw2Routing = routingHelper.GetStaticRouting(
        gatewayNodes.Get(1)->GetObject<Ipv4>());
    Ptr<Ipv4> gw2Ipv4 = gatewayNodes.Get(1)->GetObject<Ipv4>();
    int32_t gw2NrIdx = gw2Ipv4->GetInterfaceForAddress(gatewayNrIpIfaces.GetAddress(1));
    gw2Routing->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), gw2NrIdx);
    gatewayNodes.Get(1)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));

    std::cout << "   Gateway 1: default route via NR interface\n";
    std::cout << "   Gateway 2: default route via NR interface\n";

    // Mesh node routing: left half -> GW1, right half -> GW2
    Ipv4Address gw1MeshIp = meshIpIfaces.GetAddress(0);
    Ipv4Address gw2MeshIp = meshIpIfaces.GetAddress(1);

    uint32_t leftCount = 0, rightCount = 0;
    for (uint32_t i = 0; i < numMeshNodes; ++i)
    {
        uint32_t col = i % gridWidth;
        Ptr<Ipv4StaticRouting> nodeRouting = routingHelper.GetStaticRouting(
            meshOnlyNodes.Get(i)->GetObject<Ipv4>());

        if (col < gridWidth / 2)
        {
            // Left half -> Gateway 1
            nodeRouting->SetDefaultRoute(gw1MeshIp, 1);
            leftCount++;
        }
        else
        {
            // Right half -> Gateway 2
            nodeRouting->SetDefaultRoute(gw2MeshIp, 1);
            rightCount++;
        }
    }

    std::cout << "   Left half (" << leftCount << " nodes) -> Gateway 1 (" << gw1MeshIp << ")\n";
    std::cout << "   Right half (" << rightCount << " nodes) -> Gateway 2 (" << gw2MeshIp << ")\n\n";

    // ====================================================================
    // SECTION 10: ATTACH GATEWAYS TO GNB AND CONFIGURE BEARERS
    // ====================================================================

    std::cout << "[9/12] Attaching gateways to gNB and configuring QoS bearers...\n";

    nrHelper->AttachToClosestGnb(gatewayNrDevs, gnbNetDev);

    std::cout << "   Both gateways attached to gNB\n";

    // Configure bearers for BOTH gateways
    for (uint32_t gwIdx = 0; gwIdx < numGateways; ++gwIdx)
    {
        std::cout << "   Configuring bearers for Gateway " << (gwIdx + 1) << ":\n";

        // Critical bearer (QCI 1)
        Ptr<NrEpcTft> criticalTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter criticalUp;
        criticalUp.localAddress = Ipv4Address("10.1.1.0");
        criticalUp.localMask = Ipv4Mask("255.255.255.0");
        criticalUp.remotePortStart = CRITICAL_PORT;
        criticalUp.remotePortEnd = CRITICAL_PORT;
        criticalUp.direction = NrEpcTft::UPLINK;
        criticalTft->Add(criticalUp);

        NrEpcTft::PacketFilter criticalDown;
        criticalDown.remoteAddress = Ipv4Address("10.1.1.0");
        criticalDown.remoteMask = Ipv4Mask("255.255.255.0");
        criticalDown.localPortStart = CRITICAL_PORT;
        criticalDown.localPortEnd = CRITICAL_PORT;
        criticalDown.direction = NrEpcTft::DOWNLINK;
        criticalTft->Add(criticalDown);

        NrEpsBearer criticalBearer(NrEpsBearer::GBR_CONV_VOICE);
        nrHelper->ActivateDedicatedEpsBearer(gatewayNrDevs.Get(gwIdx), criticalBearer, criticalTft);
        std::cout << "     Critical (QCI 1) - Port " << CRITICAL_PORT << "\n";

        // Video bearer (QCI 4)
        Ptr<NrEpcTft> videoTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter videoUp;
        videoUp.localAddress = Ipv4Address("10.1.1.0");
        videoUp.localMask = Ipv4Mask("255.255.255.0");
        videoUp.remotePortStart = VIDEO_PORT;
        videoUp.remotePortEnd = VIDEO_PORT;
        videoUp.direction = NrEpcTft::UPLINK;
        videoTft->Add(videoUp);

        NrEpcTft::PacketFilter videoDown;
        videoDown.remoteAddress = Ipv4Address("10.1.1.0");
        videoDown.remoteMask = Ipv4Mask("255.255.255.0");
        videoDown.localPortStart = VIDEO_PORT;
        videoDown.localPortEnd = VIDEO_PORT;
        videoDown.direction = NrEpcTft::DOWNLINK;
        videoTft->Add(videoDown);

        NrEpsBearer videoBearer(NrEpsBearer::GBR_NON_CONV_VIDEO);
        nrHelper->ActivateDedicatedEpsBearer(gatewayNrDevs.Get(gwIdx), videoBearer, videoTft);
        std::cout << "     Video (QCI 4) - Port " << VIDEO_PORT << "\n";

        // Best Effort bearer (QCI 9)
        Ptr<NrEpcTft> beTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter beUp;
        beUp.localAddress = Ipv4Address("10.1.1.0");
        beUp.localMask = Ipv4Mask("255.255.255.0");
        beUp.remotePortStart = BEST_EFFORT_PORT;
        beUp.remotePortEnd = BEST_EFFORT_PORT;
        beUp.direction = NrEpcTft::UPLINK;
        beTft->Add(beUp);

        NrEpcTft::PacketFilter beDown;
        beDown.remoteAddress = Ipv4Address("10.1.1.0");
        beDown.remoteMask = Ipv4Mask("255.255.255.0");
        beDown.localPortStart = BEST_EFFORT_PORT;
        beDown.localPortEnd = BEST_EFFORT_PORT;
        beDown.direction = NrEpcTft::DOWNLINK;
        beTft->Add(beDown);

        NrEpsBearer beBearer(NrEpsBearer::NGBR_VIDEO_TCP_DEFAULT);
        nrHelper->ActivateDedicatedEpsBearer(gatewayNrDevs.Get(gwIdx), beBearer, beTft);
        std::cout << "     Best Effort (QCI 9) - Port " << BEST_EFFORT_PORT << "\n";
    }
    std::cout << "\n";

    // ====================================================================
    // SECTION 11: TRAFFIC APPLICATIONS (6 REPRESENTATIVE SOURCES)
    // ====================================================================

    std::cout << "[10/12] Setting up traffic applications (6 sources, 18 flows)...\n\n";

    // Source node selection: 1 per row, alternating left/right
    // Row 0: col 1 (left), Row 1: col 6 (right), Row 2: col 1, etc.
    std::vector<uint32_t> sourceNodes;
    std::vector<std::string> sourceGateways;

    for (uint32_t row = 0; row < gridHeight; ++row)
    {
        uint32_t col = (row % 2 == 0) ? 1 : 6;  // Alternate left/right
        uint32_t nodeIdx = GetNodeIndex(col, row, gridWidth);
        sourceNodes.push_back(nodeIdx);
        sourceGateways.push_back((col < gridWidth/2) ? "GW1" : "GW2");
    }

    // Install sink applications on MEC server
    PacketSinkHelper criticalSink("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), CRITICAL_PORT));
    PacketSinkHelper videoSink("ns3::UdpSocketFactory",
                               InetSocketAddress(Ipv4Address::GetAny(), VIDEO_PORT));
    PacketSinkHelper beSink("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), BEST_EFFORT_PORT));

    ApplicationContainer sinkApps;
    sinkApps.Add(criticalSink.Install(mecServer.Get(0)));
    sinkApps.Add(videoSink.Install(mecServer.Get(0)));
    sinkApps.Add(beSink.Install(mecServer.Get(0)));
    sinkApps.Start(Seconds(1.0));
    sinkApps.Stop(simTime);

    // Install traffic generators on source nodes
    Ipv4Address mecIp = mecIpIface.GetAddress(1);

    for (uint32_t i = 0; i < sourceNodes.size(); ++i)
    {
        uint32_t nodeIdx = sourceNodes[i];
        Ptr<Node> sourceNode = meshOnlyNodes.Get(nodeIdx);
        Ipv4Address sourceIp = meshIpIfaces.GetAddress(nodeIdx + 2);  // +2 for gateway offsets

        std::cout << "   Source " << (i+1) << ": Node " << nodeIdx
                  << " (" << sourceIp << ") via " << sourceGateways[i] << "\n";

        Time startOffset = appStartTime + MilliSeconds(i * 100);

        // =================================================================
        // CRITICAL TRAFFIC -> GBR_CONV_VOICE bearer (QCI 1, highest priority)
        // Port 5050 -> matched by TFT -> dedicated GBR bearer
        // =================================================================
        OnOffHelper criticalOnOff("ns3::UdpSocketFactory",
                                  InetSocketAddress(mecIp, CRITICAL_PORT));
        criticalOnOff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(criticalRate) + "kbps")));
        criticalOnOff.SetAttribute("PacketSize", UintegerValue(128));
        criticalOnOff.SetAttribute("Tos", UintegerValue(TOS_CRITICAL));
        criticalOnOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        criticalOnOff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer critApp = criticalOnOff.Install(sourceNode);
        critApp.Start(startOffset);
        critApp.Stop(simTime - Seconds(0.5));

        // =================================================================
        // VIDEO TRAFFIC -> GBR_NON_CONV_VIDEO bearer (QCI 4, medium priority)
        // Port 5150 -> matched by TFT -> dedicated GBR bearer
        // =================================================================
        OnOffHelper videoOnOff("ns3::UdpSocketFactory",
                               InetSocketAddress(mecIp, VIDEO_PORT));
        videoOnOff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(videoRate) + "kbps")));
        videoOnOff.SetAttribute("PacketSize", UintegerValue(1400));
        videoOnOff.SetAttribute("Tos", UintegerValue(TOS_VIDEO));
        videoOnOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        videoOnOff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer videoApp = videoOnOff.Install(sourceNode);
        videoApp.Start(startOffset);
        videoApp.Stop(simTime - Seconds(0.5));

        // =================================================================
        // BEST EFFORT TRAFFIC -> NGBR_VIDEO_TCP_DEFAULT bearer (QCI 9, lowest)
        // Port 5250 -> matched by TFT -> Non-GBR bearer
        // =================================================================
        OnOffHelper beOnOff("ns3::UdpSocketFactory",
                            InetSocketAddress(mecIp, BEST_EFFORT_PORT));
        beOnOff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(beRate) + "kbps")));
        beOnOff.SetAttribute("PacketSize", UintegerValue(1400));
        beOnOff.SetAttribute("Tos", UintegerValue(TOS_BESTEFFORT));
        beOnOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        beOnOff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer beApp = beOnOff.Install(sourceNode);
        beApp.Start(startOffset);
        beApp.Stop(simTime - Seconds(0.5));
    }

    std::cout << "\n   Total: 6 sources × 3 classes = 18 flows\n";
    std::cout << "   Per-source rate: " << (criticalRate + videoRate + beRate) << " kbps\n";
    std::cout << "   Total offered load: " << 6 * (criticalRate + videoRate + beRate) / 1000.0 << " Mbps\n\n";

    // ====================================================================
    // SECTION 12: FLOW MONITOR
    // ====================================================================

    std::cout << "[11/12] Setting up FlowMonitor...\n";

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    std::cout << "   FlowMonitor installed on all nodes\n\n";

    // ====================================================================
    // SECTION 13: RUN SIMULATION
    // ====================================================================

    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                      Starting Simulation                         ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";

    Simulator::Stop(simTime);
    Simulator::Run();

    // ====================================================================
    // SECTION 14: RESULTS ANALYSIS
    // ====================================================================

    std::cout << "\n╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                      Simulation Results                          ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    // Per-class statistics
    struct QosStats {
        std::string name;
        uint16_t port;
        uint32_t txPackets = 0;
        uint32_t rxPackets = 0;
        double delaySum = 0;
        double throughput = 0;
    };

    QosStats criticalStats = {"Critical (QCI 1)", CRITICAL_PORT, 0, 0, 0, 0};
    QosStats videoStats = {"Video (QCI 4)", VIDEO_PORT, 0, 0, 0, 0};
    QosStats beStats = {"Best Effort (QCI 9)", BEST_EFFORT_PORT, 0, 0, 0, 0};

    uint32_t flowCount = 0;
    double totalThroughput = 0;

    std::cout << "Per-Flow Statistics:\n";
    std::cout << "────────────────────────────────────────────────────────────────────\n";

    for (auto& flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);

        QosStats* classStats = nullptr;
        std::string className;

        if (t.destinationPort == CRITICAL_PORT) {
            classStats = &criticalStats;
            className = "CRITICAL";
        } else if (t.destinationPort == VIDEO_PORT) {
            classStats = &videoStats;
            className = "VIDEO";
        } else if (t.destinationPort == BEST_EFFORT_PORT) {
            classStats = &beStats;
            className = "BEST EFFORT";
        } else {
            continue;  // Skip non-QoS flows
        }

        double pdr = 0, throughput = 0, meanDelay = 0;

        if (flow.second.txPackets > 0)
            pdr = 100.0 * flow.second.rxPackets / flow.second.txPackets;

        if (flow.second.rxPackets > 0)
        {
            double duration = (simTime - appStartTime).GetSeconds();
            throughput = flow.second.rxBytes * 8.0 / duration / 1e6;
            meanDelay = flow.second.delaySum.GetMilliSeconds() / flow.second.rxPackets;
        }

        // Accumulate stats
        classStats->txPackets += flow.second.txPackets;
        classStats->rxPackets += flow.second.rxPackets;
        classStats->delaySum += flow.second.delaySum.GetMilliSeconds();
        classStats->throughput += throughput;

        totalThroughput += throughput;
        flowCount++;

        std::cout << "Flow " << flow.first << " [" << className << "]: "
                  << t.sourceAddress << " → " << t.destinationAddress << ":" << t.destinationPort
                  << " | PDR: " << std::fixed << std::setprecision(1) << pdr << "%"
                  << " | " << std::setprecision(2) << throughput << " Mbps"
                  << " | " << meanDelay << " ms\n";
    }

    // Print per-class summary
    std::cout << "\n────────────────────────────────────────────────────────────────────\n";
    std::cout << "QoS Class Summary:\n";
    std::cout << "────────────────────────────────────────────────────────────────────\n";

    auto printStats = [](const QosStats& s) {
        double pdr = (s.txPackets > 0) ? 100.0 * s.rxPackets / s.txPackets : 0;
        double meanDelay = (s.rxPackets > 0) ? s.delaySum / s.rxPackets : 0;

        std::cout << "  " << s.name << ":\n";
        std::cout << "    TX/RX: " << s.txPackets << "/" << s.rxPackets << " packets\n";
        std::cout << "    PDR: " << std::fixed << std::setprecision(1) << pdr << "%\n";
        std::cout << "    Throughput: " << std::setprecision(3) << s.throughput << " Mbps\n";
        std::cout << "    Mean Delay: " << std::setprecision(2) << meanDelay << " ms\n\n";
    };

    printStats(criticalStats);
    printStats(videoStats);
    printStats(beStats);

    // Overall summary
    std::cout << "────────────────────────────────────────────────────────────────────\n";
    std::cout << "Overall Summary:\n";
    std::cout << "   Total flows: " << flowCount << "\n";
    std::cout << "   Total throughput: " << std::setprecision(2) << totalThroughput << " Mbps\n";

    // Calculate overall PDR
    uint32_t totalTx = criticalStats.txPackets + videoStats.txPackets + beStats.txPackets;
    uint32_t totalRx = criticalStats.rxPackets + videoStats.rxPackets + beStats.rxPackets;
    double overallPdr = (totalTx > 0) ? 100.0 * totalRx / totalTx : 0;
    std::cout << "   Overall PDR: " << std::setprecision(1) << overallPdr << "%\n";

    // Success check
    std::cout << "\n";
    if (overallPdr >= 80.0)
    {
        std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║  SUCCESS: 50-node scaled architecture working!                  ║\n";
        std::cout << "║  PDR >= 80% achieved with dual-gateway configuration            ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";
    }
    else
    {
        std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║  WARNING: PDR below target (< 80%)                               ║\n";
        std::cout << "║  Try: --simTime=30 or --meshSpacing=35                           ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";
    }

    // Generate reports
    std::cout << "\n[12/12] Generating reports...\n";

    for (uint32_t i = 0; i < std::min(meshDevices.GetN(), 4u); ++i)
    {
        std::ostringstream filename;
        filename << "50node-mesh-report-" << i << ".xml";
        std::ofstream of(filename.str());
        if (of.is_open())
        {
            mesh.Report(meshDevices.Get(i), of);
            of.close();
            std::cout << "   Generated: " << filename.str() << "\n";
        }
    }

    flowmonHelper.SerializeToXmlFile("50node-flowmon.xml", true, true);
    std::cout << "   Generated: 50node-flowmon.xml\n";

    std::cout << "\n";

    Simulator::Destroy();
    return 0;
}
