// Copyright (c) 2024 5G-Mesh Integration Project
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * \file 5g-mesh-bandit.cc
 * \brief 64-Node Hybrid 5G-Mesh with LinUCB Gateway Selection
 *
 * Extends the 50-node Phase 3 simulation to use per-node Contextual Bandits
 * (LinUCB) for adaptive gateway selection instead of static spatial assignment.
 *
 * Topology: 8x8 mesh grid (64 nodes) + 4 asymmetric gateways + 12 source nodes
 *
 * +----+----+----+----+----+----+----+----+
 * | o  | S  | o  | o  | o  | o  | S  | o |  Row 7
 * +----+----+----+----+----+----+----+----+
 * | o  | S  |GW1 | o  | o  | o  | S  | o |  Row 6   S = Source node
 * +----+----+----+----+----+----+----+----+          GW = Smart Gateway
 * | o  | o  | o  | o  | o  | o  | o  | o |  Row 5
 * +----+----+----+----+----+----+----+----+
 * | o  | S  | o  | o  | o  |GW2 | S  | o |  Row 4
 * +----+----+----+----+----+----+----+----+
 * | o  | o  | o  | o  | o  | o  | o  | o |  Row 3
 * +----+----+----+----+----+----+----+----+
 * | o  | S  |GW3 | o  | o  | o  | S  | o |  Row 2
 * +----+----+----+----+----+----+----+----+
 * | o  | S  | o  | o  | o  | o  | S  | o |  Row 1
 * +----+----+----+----+----+----+----+----+
 * | o  | o  | o  | o  | o  |GW4 | o  | o |  Row 0
 * +----+----+----+----+----+----+----+----+
 *   0    1    2    3    4    5    6    7    (columns)
 *
 * Gateway positions (asymmetric -- not clean quadrant centers):
 *   GW1: (col=2, row=6) = (70m, 210m)
 *   GW2: (col=5, row=4) = (175m, 140m)
 *   GW3: (col=2, row=2) = (70m,   70m)
 *   GW4: (col=5, row=0) = (175m,   0m)
 *
 * Test scenarios (--scenario=):
 *   default  : All gateways equal energy/position
 *   energy   : GW2(500J) and GW4(300J) are battery-depleted
 *   sinr     : GW4 moved far from gNB -> degraded 5G link
 *
 * Usage:
 *   ./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --useBandit=true --scenario=default"
 *   ./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --useBandit=false --scenario=energy"
 */

#include "linucb-gateway-selector.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/yans-wifi-helper.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("5gMeshBandit");

// ========================================================================
// ENERGY SCENARIO: gateway failure callback
// ========================================================================
static void
DisableGatewayForwarding(Ptr<Node> gwNode,
                          Ptr<energy::SimpleDeviceEnergyModel> dm,
                          std::string label)
{
    dm->SetCurrentA(0.0);  // stop draining so BasicEnergySource doesn't go below 0
    gwNode->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(false));
    std::cout << "[ENERGY] T=" << Simulator::Now().GetSeconds()
              << "s: " << label << " battery depleted -- gateway disabled\n";
}

// ========================================================================
// PORT / QoS CONSTANTS
// ========================================================================
const uint16_t CRITICAL_PORT    = 5050;
const uint16_t VIDEO_PORT       = 5150;
const uint16_t BEST_EFFORT_PORT = 5250;

const uint8_t TOS_CRITICAL    = 0xB8; // DSCP EF (46)  -> AC_VO
const uint8_t TOS_VIDEO       = 0x80; // DSCP CS4 (32) -> AC_VI
const uint8_t TOS_BESTEFFORT  = 0x00; // DSCP CS0 (0)  -> AC_BE

// ========================================================================
// HELPER: grid index
// ========================================================================
uint32_t GetNodeIndex(uint32_t col, uint32_t row, uint32_t gridWidth)
{
    return row * gridWidth + col;
}

// ========================================================================
// HELPER: nearest gateway by Euclidean distance (for relay node routing)
// ========================================================================
uint32_t FindNearestGateway(uint32_t meshNodeIdx,
                             uint32_t gridWidth,
                             double   meshSpacing,
                             double   gwCoords[][2],
                             uint32_t numGateways)
{
    uint32_t col    = meshNodeIdx % gridWidth;
    uint32_t row    = meshNodeIdx / gridWidth;
    double   nodeX  = col * meshSpacing;
    double   nodeY  = row * meshSpacing;

    uint32_t nearest = 0;
    double   minDist = 1e18;
    for (uint32_t g = 0; g < numGateways; g++)
    {
        double dx   = nodeX - gwCoords[g][0];
        double dy   = nodeY - gwCoords[g][1];
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < minDist)
        {
            minDist  = dist;
            nearest  = g;
        }
    }
    return nearest;
}

// ========================================================================
// HELPER: Matrix4x4 self-test (runs once at startup)
// ========================================================================
static void RunMatrixSelfTest()
{
    // Test 1: Identity inverts to Identity
    Matrix4x4 I     = Matrix4x4::Identity();
    Matrix4x4 I_inv = Matrix4x4::Zero();
    bool ok = I.Invert(I_inv);
    NS_ASSERT_MSG(ok, "Identity inversion failed");
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
            double expected = (i == j) ? 1.0 : 0.0;
            NS_ASSERT_MSG(std::abs(I_inv.m[i][j] - expected) < 1e-10,
                          "Identity inverse wrong at (" << i << "," << j << ")");
        }

    // Test 2: Sherman-Morrison matches full inversion for A = I + x*x^T
    double x[4] = {0.5, 0.3, 0.8, 1.0};
    Matrix4x4 A = I + Matrix4x4::OuterProduct(x);

    // Full inversion
    Matrix4x4 A_inv_full;
    ok = A.Invert(A_inv_full);
    NS_ASSERT_MSG(ok, "A inversion failed");

    // Sherman-Morrison starting from I^-1 = I
    Matrix4x4 A_inv_sm = Matrix4x4::Identity();
    UpdateInverse(A_inv_sm, x);

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
            NS_ASSERT_MSG(std::abs(A_inv_full.m[i][j] - A_inv_sm.m[i][j]) < 1e-10,
                          "SM vs full inversion mismatch at (" << i << "," << j << ")");
        }

    std::cout << "[OK] Matrix4x4 self-test passed (Identity + Sherman-Morrison)\n\n";
}

// ========================================================================
// MAIN
// ========================================================================
int main(int argc, char* argv[])
{
    // ====================================================================
    // SECTION 1: COMMAND-LINE PARAMETERS
    // ====================================================================

    // Grid / topology
    uint32_t gridWidth    = 8;
    uint32_t gridHeight   = 8;
    double   meshSpacing  = 35.0; // metres

    // NR parameters
    uint16_t numerology       = 0;
    double   centralFrequency = 4e9;
    double   bandwidth        = 20e6;
    double   gnbTxPower       = 43; // dBm

    // Traffic rates (kbps)
    // 12 sources x 70 kbps = 840 kbps total offered load.
    // Kept low because 68-node 802.11a mesh shares one channel: excessive load
    // causes HWMP control overhead to overwhelm data capacity.
    uint32_t criticalRate = 10;
    uint32_t videoRate    = 30;
    uint32_t beRate       = 30;

    // Simulation timing -- 30s convergence window for 64-node HWMP mesh
    Time simTime     = Seconds(90.0);
    Time appStartTime = Seconds(30.0);

    // Bandit parameters
    double   banditAlpha       = 0.3;
    double   decisionIntervalSec = 1.0;
    bool     useBandit         = true;
    std::string scenario       = "default"; // "default", "energy", "sinr"

    bool        logging = false;
    std::string simTag  = "5g-mesh-bandit";

    CommandLine cmd;
    cmd.AddValue("simTime",          "Simulation time (seconds)",                 simTime);
    cmd.AddValue("appStartTime",     "App start time - allow mesh convergence",   appStartTime);
    cmd.AddValue("gridWidth",        "Grid width (columns)",                      gridWidth);
    cmd.AddValue("gridHeight",       "Grid height (rows)",                        gridHeight);
    cmd.AddValue("meshSpacing",      "Mesh node spacing (metres)",                meshSpacing);
    cmd.AddValue("criticalRate",     "Critical traffic rate (kbps)",              criticalRate);
    cmd.AddValue("videoRate",        "Video traffic rate (kbps)",                 videoRate);
    cmd.AddValue("beRate",           "Best effort traffic rate (kbps)",           beRate);
    cmd.AddValue("banditAlpha",      "LinUCB exploration parameter alpha",        banditAlpha);
    cmd.AddValue("decisionInterval", "Bandit decision interval (seconds)",        decisionIntervalSec);
    cmd.AddValue("useBandit",        "Enable LinUCB bandit (false=static routing)", useBandit);
    cmd.AddValue("scenario",         "Test scenario: default | energy | sinr",   scenario);
    cmd.AddValue("logging",          "Enable verbose logging",                    logging);
    cmd.AddValue("simTag",           "Output filename tag",                       simTag);
    cmd.Parse(argc, argv);

    if (logging)
    {
        LogComponentEnable("5gMeshBandit", LOG_LEVEL_INFO);
        LogComponentEnable("HwmpProtocol", LOG_LEVEL_DEBUG);
    }

    uint32_t numGateways  = 4;
    uint32_t numMeshNodes = gridWidth * gridHeight; // 64
    uint32_t totalNodes   = numMeshNodes + numGateways;

    // ====================================================================
    // Matrix self-test (verifies math before simulation starts)
    // ====================================================================
    RunMatrixSelfTest();

    // ====================================================================
    // Print configuration banner
    // ====================================================================
    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     5G-Mesh Bandit -- LinUCB Adaptive Gateway Selection         ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Topology:                                                        ║\n";
    std::cout << "║   Grid: " << gridWidth << "x" << gridHeight
              << " (" << numMeshNodes << " mesh nodes) + " << numGateways << " gateways           ║\n";
    std::cout << "║   Mesh spacing: " << meshSpacing << "m                                        ║\n";
    std::cout << "║   Total nodes: " << totalNodes << "                                          ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Gateway Selection:                                               ║\n";
    std::cout << "║   Mode:     " << (useBandit ? "LinUCB bandit (adaptive)       " : "Static nearest-gateway        ") << "         ║\n";
    std::cout << "║   Scenario: " << std::setw(8) << scenario
              << "                                            ║\n";
    if (useBandit)
    {
        std::cout << "║   Alpha:    " << banditAlpha
                  << "  |  Interval: " << decisionIntervalSec << "s                       ║\n";
    }
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Simulation time: " << simTime.GetSeconds() << "s                                    ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";

    // ====================================================================
    // SECTION 2: NODE CREATION
    // ====================================================================

    std::cout << "[1/13] Creating nodes...\n";

    NodeContainer gatewayNodes;
    gatewayNodes.Create(numGateways);

    NodeContainer meshOnlyNodes;
    meshOnlyNodes.Create(numMeshNodes);

    NodeContainer allMeshNodes;
    allMeshNodes.Add(gatewayNodes);
    allMeshNodes.Add(meshOnlyNodes);

    NodeContainer gnbNode;
    gnbNode.Create(1);

    NodeContainer mecServer;
    mecServer.Create(1);

    std::cout << "   Gateway nodes:    " << numGateways << "\n";
    std::cout << "   Mesh-only nodes:  " << numMeshNodes << "\n";
    std::cout << "   gNB + MEC:        2\n\n";

    // ====================================================================
    // SECTION 3: NODE POSITIONING
    // ====================================================================

    std::cout << "[2/13] Setting up node positions...\n";

    // Gateway positions (asymmetric -- not clean quadrant centers)
    double gwCoords[4][2] = {
        {2 * meshSpacing, 6 * meshSpacing}, // GW1: (70, 210)  top-left area
        {5 * meshSpacing, 4 * meshSpacing}, // GW2: (175, 140) center-right
        {2 * meshSpacing, 2 * meshSpacing}, // GW3: (70, 70)   bottom-left
        {5 * meshSpacing, 0 * meshSpacing}, // GW4: (175, 0)   bottom-right edge
    };

    // Scenario 3: Move GW4 far from gNB to degrade its 5G SINR
    if (scenario == "sinr")
    {
        gwCoords[3][0] = 7 * meshSpacing; // (245, 0) -- far corner
        gwCoords[3][1] = 0.0;
        std::cout << "   [sinr] GW4 moved to far corner (" << gwCoords[3][0] << "m, 0m)\n";
    }

    MobilityHelper mobility;

    Ptr<ListPositionAllocator> gwPositions = CreateObject<ListPositionAllocator>();
    for (uint32_t g = 0; g < numGateways; g++)
        gwPositions->Add(Vector(gwCoords[g][0], gwCoords[g][1], 1.5));

    mobility.SetPositionAllocator(gwPositions);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gatewayNodes);

    for (uint32_t g = 0; g < numGateways; g++)
        std::cout << "   GW" << (g + 1) << ": (" << gwCoords[g][0] << ", "
                  << gwCoords[g][1] << ", 1.5)\n";

    // Mesh nodes: 8x8 grid
    Ptr<ListPositionAllocator> meshPositions = CreateObject<ListPositionAllocator>();
    for (uint32_t row = 0; row < gridHeight; row++)
        for (uint32_t col = 0; col < gridWidth; col++)
            meshPositions->Add(Vector(col * meshSpacing, row * meshSpacing, 1.5));

    mobility.SetPositionAllocator(meshPositions);
    mobility.Install(meshOnlyNodes);

    std::cout << "   Mesh: " << numMeshNodes << " nodes in " << gridWidth << "x" << gridHeight
              << " grid, spacing=" << meshSpacing << "m\n";

    // gNB at center of grid (elevated)
    double gnbX = (gridWidth - 1) * meshSpacing / 2.0;  // 122.5m
    double gnbY = (gridHeight - 1) * meshSpacing / 2.0; // 122.5m

    Ptr<ListPositionAllocator> gnbPosition = CreateObject<ListPositionAllocator>();
    gnbPosition->Add(Vector(gnbX, gnbY, 10.0));
    mobility.SetPositionAllocator(gnbPosition);
    mobility.Install(gnbNode);

    // MEC server near gNB
    Ptr<ListPositionAllocator> mecPosition = CreateObject<ListPositionAllocator>();
    mecPosition->Add(Vector(gnbX + 50, gnbY, 1.5));
    mobility.SetPositionAllocator(mecPosition);
    mobility.Install(mecServer);

    std::cout << "   gNB: (" << gnbX << ", " << gnbY << ", 10)\n\n";

    // ====================================================================
    // SECTION 4: NR HELPER AND EPC SETUP
    // ====================================================================

    std::cout << "[3/13] Setting up 5G NR infrastructure...\n";

    Ptr<NrPointToPointEpcHelper>  nrEpcHelper          = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper>   idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper>                 nrHelper              = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaQos"));

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    nrHelper->SetUeAntennaAttribute("NumRows",       UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns",    UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows",      UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns",   UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    std::cout << "   NR helpers configured (QoS scheduler)\n\n";

    // ====================================================================
    // SECTION 5: NR SPECTRUM CONFIGURATION
    // ====================================================================

    std::cout << "[4/13] Configuring NR spectrum...\n";

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator               ccBwpCreator;

    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency,
                                                    bandwidth,
                                                    1,
                                                    BandwidthPartInfo::UMi_StreetCanyon);
    bandConf.m_numBwp = 1;

    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    std::cout << "   1 BWP at " << centralFrequency / 1e9 << " GHz\n\n";

    // ====================================================================
    // SECTION 6: INSTALL NR DEVICES
    // ====================================================================

    std::cout << "[5/13] Installing NR devices on " << numGateways << " gateways...\n";

    NetDeviceContainer gnbNetDev      = nrHelper->InstallGnbDevice(gnbNode, allBwps);
    NetDeviceContainer gatewayNrDevs  = nrHelper->InstallUeDevice(gatewayNodes, allBwps);

    double txPowerLinear = std::pow(10.0, gnbTxPower / 10.0);
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("Numerology",
                                                             UintegerValue(numerology));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("TxPower",
                                                             DoubleValue(10.0 * std::log10(txPowerLinear)));

    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    for (auto it = gatewayNrDevs.Begin(); it != gatewayNrDevs.End(); ++it)
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();

    std::cout << "   gNB + " << numGateways << " gateway NR UE devices installed\n\n";

    // ====================================================================
    // SECTION 7: MESH NETWORK SETUP
    // ====================================================================

    std::cout << "[6/13] Setting up 802.11s mesh network...\n";

    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper     wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());

    MeshHelper mesh = MeshHelper::Default();
    mesh.SetStackInstaller("ns3::Dot11sStack");
    mesh.SetSpreadInterfaceChannels(MeshHelper::ZERO_CHANNEL);
    mesh.SetNumberOfInterfaces(1);
    mesh.SetMacType("RandomStart", TimeValue(Seconds(0.1)));
    mesh.SetStandard(WIFI_STANDARD_80211a);

    NetDeviceContainer meshDevices = mesh.Install(wifiPhy, allMeshNodes);

    std::cout << "   Mesh devices: " << meshDevices.GetN()
              << " nodes (" << numGateways << " GWs + " << numMeshNodes << " mesh)\n\n";

    // ====================================================================
    // SECTION 8: INTERNET STACK AND IP
    // ====================================================================

    std::cout << "[7/13] Configuring Internet stack and IP addresses...\n";

    InternetStackHelper internet;
    internet.Install(mecServer);
    internet.Install(meshOnlyNodes);
    internet.Install(gatewayNodes);

    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();

    // MEC server <-> PGW link
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gb/s")));
    p2p.SetDeviceAttribute("Mtu",      UintegerValue(2500));
    p2p.SetChannelAttribute("Delay",   TimeValue(MilliSeconds(1)));
    NetDeviceContainer mecDevices = p2p.Install(pgw, mecServer.Get(0));

    Ipv4AddressHelper mecIpHelper;
    mecIpHelper.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer mecIpIface = mecIpHelper.Assign(mecDevices);

    std::cout << "   MEC Server IP: " << mecIpIface.GetAddress(1) << "\n";

    // MEC routing
    Ipv4StaticRoutingHelper routingHelper;
    Ptr<Ipv4StaticRouting>  mecRouting =
        routingHelper.GetStaticRouting(mecServer.Get(0)->GetObject<Ipv4>());
    mecRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Assign NR IPs to gateways
    Ipv4InterfaceContainer gatewayNrIpIfaces = nrEpcHelper->AssignUeIpv4Address(gatewayNrDevs);
    for (uint32_t g = 0; g < numGateways; g++)
        std::cout << "   GW" << (g + 1) << " NR IP: " << gatewayNrIpIfaces.GetAddress(g) << "\n";

    // PGW route to mesh subnet (via GW1's NR IP)
    Ptr<Ipv4StaticRouting> pgwRouting = routingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
    pgwRouting->AddNetworkRouteTo(
        Ipv4Address("10.1.1.0"),
        Ipv4Mask("255.255.255.0"),
        gatewayNrIpIfaces.GetAddress(0),
        pgw->GetObject<Ipv4>()->GetInterfaceForDevice(pgw->GetDevice(1)));

    // Assign mesh IPs (allMeshNodes order: GW0..3 then meshOnlyNodes 0..63)
    Ipv4AddressHelper      meshIpHelper;
    meshIpHelper.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer meshIpIfaces = meshIpHelper.Assign(meshDevices);

    for (uint32_t g = 0; g < numGateways; g++)
        std::cout << "   GW" << (g + 1) << " mesh IP: " << meshIpIfaces.GetAddress(g) << "\n";
    std::cout << "   Mesh nodes: 10.1.1." << (numGateways + 1)
              << " - 10.1.1." << (numMeshNodes + numGateways) << "\n";

    // MEC route to mesh subnet
    mecRouting->AddNetworkRouteTo(Ipv4Address("10.1.1.0"), Ipv4Mask("255.255.255.0"), 1);

    std::cout << "\n";

    // ====================================================================
    // SECTION 9: ENERGY MODEL ON GATEWAYS
    // ====================================================================

    std::cout << "[8/13] Installing energy model on gateways...\n";

    double gwInitialEnergy[4] = {10000.0, 10000.0, 10000.0, 10000.0};

    if (scenario == "energy")
    {
        // GW2 and GW4 have severely depleted batteries
        gwInitialEnergy[1] = 500.0;
        gwInitialEnergy[3] = 300.0;
        std::cout << "   [energy] GW2: 500J  |  GW4: 300J  (degraded)\n";
    }

    BasicEnergySourceHelper        basicEnergySourceHelper;
    energy::EnergySourceContainer  gwEnergySources;

    for (uint32_t g = 0; g < numGateways; g++)
    {
        basicEnergySourceHelper.Set("BasicEnergySourceInitialEnergyJ",
                                    DoubleValue(gwInitialEnergy[g]));
        gwEnergySources.Add(basicEnergySourceHelper.Install(gatewayNodes.Get(g)));
        std::cout << "   GW" << (g + 1) << ": " << gwInitialEnergy[g] << " J initial energy\n";
    }

    if (scenario == "energy")
    {
        // GW4 (index 3): 300J / (3V × 2A) = 50s drain → disable at t=50s
        Ptr<energy::SimpleDeviceEnergyModel> dm4 = CreateObject<energy::SimpleDeviceEnergyModel>();
        dm4->SetNode(gatewayNodes.Get(3));
        dm4->SetEnergySource(gwEnergySources.Get(3));  // must set source before SetCurrentA
        gwEnergySources.Get(3)->AppendDeviceEnergyModel(dm4);
        dm4->SetCurrentA(2.0);

        // GW2 (index 1): 500J / (3V × 2.5A) = 66.7s drain → disable at t=67s
        Ptr<energy::SimpleDeviceEnergyModel> dm2 = CreateObject<energy::SimpleDeviceEnergyModel>();
        dm2->SetNode(gatewayNodes.Get(1));
        dm2->SetEnergySource(gwEnergySources.Get(1));  // must set source before SetCurrentA
        gwEnergySources.Get(1)->AppendDeviceEnergyModel(dm2);
        dm2->SetCurrentA(2.5);

        Simulator::Schedule(Seconds(50.0), &DisableGatewayForwarding,
                            gatewayNodes.Get(3), dm4, std::string("GW4"));
        Simulator::Schedule(Seconds(67.0), &DisableGatewayForwarding,
                            gatewayNodes.Get(1), dm2, std::string("GW2"));

        std::cout << "   [energy] GW4 drain: 6W → depletes at t=50s\n";
        std::cout << "   [energy] GW2 drain: 7.5W → depletes at t=67s\n";
    }

    std::cout << "\n";

    // ====================================================================
    // SECTION 10: ROUTING CONFIGURATION
    // ====================================================================

    std::cout << "[9/13] Configuring routing...\n";

    // Each gateway: default route via its NR interface + enable IP forwarding
    for (uint32_t g = 0; g < numGateways; g++)
    {
        Ptr<Ipv4>              gwIpv4   = gatewayNodes.Get(g)->GetObject<Ipv4>();
        int32_t                nrIfIdx  = gwIpv4->GetInterfaceForAddress(gatewayNrIpIfaces.GetAddress(g));
        Ptr<Ipv4StaticRouting> gwRoute  = routingHelper.GetStaticRouting(gwIpv4);
        gwRoute->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), nrIfIdx);
        gwIpv4->SetAttribute("IpForward", BooleanValue(true));
        std::cout << "   GW" << (g + 1) << ": default -> NR (iface " << nrIfIdx << ")\n";
    }

    // Determine source nodes (12 total across 8x8 grid)
    std::vector<uint32_t> sourceNodes;
    // Rows 0,1,3,5,7 x cols {1, 6} = 10 nodes
    uint32_t sourceRows[] = {0, 1, 3, 5, 7};
    for (uint32_t r = 0; r < 5; r++)
    {
        sourceNodes.push_back(GetNodeIndex(1, sourceRows[r], gridWidth));
        sourceNodes.push_back(GetNodeIndex(6, sourceRows[r], gridWidth));
    }
    // 2 more: row 4 col 1 and row 6 col 6  -> total 12
    sourceNodes.push_back(GetNodeIndex(1, 4, gridWidth));
    sourceNodes.push_back(GetNodeIndex(6, 6, gridWidth));

    std::sort(sourceNodes.begin(), sourceNodes.end());
    std::cout << "   Source nodes (" << sourceNodes.size() << "): ";
    for (uint32_t idx : sourceNodes)
        std::cout << idx << " ";
    std::cout << "\n";

    // Mesh node routing
    uint32_t banditInitGw = 0; // Source nodes start on GW1 (bandit explores others)
    Ipv4Address gwMeshIps[4];
    for (uint32_t g = 0; g < numGateways; g++)
        gwMeshIps[g] = meshIpIfaces.GetAddress(g);

    for (uint32_t i = 0; i < numMeshNodes; i++)
    {
        bool isSource = std::find(sourceNodes.begin(), sourceNodes.end(), i) != sourceNodes.end();

        Ptr<Ipv4StaticRouting> nodeRouting = routingHelper.GetStaticRouting(
            meshOnlyNodes.Get(i)->GetObject<Ipv4>());

        if (useBandit && isSource)
        {
            // Bandit-managed: start on GW1, will adapt dynamically
            nodeRouting->SetDefaultRoute(gwMeshIps[banditInitGw], 1);
        }
        else
        {
            // Static: nearest gateway by Euclidean distance
            uint32_t nearest = FindNearestGateway(i, gridWidth, meshSpacing, gwCoords, numGateways);
            nodeRouting->SetDefaultRoute(gwMeshIps[nearest], 1);
        }
    }

    std::cout << "   " << (useBandit ? "Source nodes start on GW1 (bandit will adapt)\n"
                                     : "All nodes use nearest gateway (static)\n");
    std::cout << "\n";

    // ====================================================================
    // SECTION 11: ATTACH GATEWAYS TO GNB + CONFIGURE BEARERS
    // ====================================================================

    std::cout << "[10/13] Attaching gateways to gNB and configuring QoS bearers...\n";

    nrHelper->AttachToClosestGnb(gatewayNrDevs, gnbNetDev);

    for (uint32_t gwIdx = 0; gwIdx < numGateways; gwIdx++)
    {
        // Critical bearer (QCI 1)
        Ptr<NrEpcTft> criticalTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter critUp;
        critUp.localAddress    = Ipv4Address("10.1.1.0");
        critUp.localMask       = Ipv4Mask("255.255.255.0");
        critUp.remotePortStart = CRITICAL_PORT;
        critUp.remotePortEnd   = CRITICAL_PORT;
        critUp.direction       = NrEpcTft::UPLINK;
        criticalTft->Add(critUp);

        NrEpcTft::PacketFilter critDown;
        critDown.remoteAddress  = Ipv4Address("10.1.1.0");
        critDown.remoteMask     = Ipv4Mask("255.255.255.0");
        critDown.localPortStart = CRITICAL_PORT;
        critDown.localPortEnd   = CRITICAL_PORT;
        critDown.direction      = NrEpcTft::DOWNLINK;
        criticalTft->Add(critDown);

        NrEpsBearer critBearer(NrEpsBearer::GBR_CONV_VOICE);
        nrHelper->ActivateDedicatedEpsBearer(gatewayNrDevs.Get(gwIdx), critBearer, criticalTft);

        // Video bearer (QCI 4)
        Ptr<NrEpcTft> videoTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter vidUp;
        vidUp.localAddress    = Ipv4Address("10.1.1.0");
        vidUp.localMask       = Ipv4Mask("255.255.255.0");
        vidUp.remotePortStart = VIDEO_PORT;
        vidUp.remotePortEnd   = VIDEO_PORT;
        vidUp.direction       = NrEpcTft::UPLINK;
        videoTft->Add(vidUp);

        NrEpcTft::PacketFilter vidDown;
        vidDown.remoteAddress  = Ipv4Address("10.1.1.0");
        vidDown.remoteMask     = Ipv4Mask("255.255.255.0");
        vidDown.localPortStart = VIDEO_PORT;
        vidDown.localPortEnd   = VIDEO_PORT;
        vidDown.direction      = NrEpcTft::DOWNLINK;
        videoTft->Add(vidDown);

        NrEpsBearer vidBearer(NrEpsBearer::GBR_NON_CONV_VIDEO);
        nrHelper->ActivateDedicatedEpsBearer(gatewayNrDevs.Get(gwIdx), vidBearer, videoTft);

        // Best Effort bearer (QCI 9)
        Ptr<NrEpcTft> beTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter beUp;
        beUp.localAddress    = Ipv4Address("10.1.1.0");
        beUp.localMask       = Ipv4Mask("255.255.255.0");
        beUp.remotePortStart = BEST_EFFORT_PORT;
        beUp.remotePortEnd   = BEST_EFFORT_PORT;
        beUp.direction       = NrEpcTft::UPLINK;
        beTft->Add(beUp);

        NrEpcTft::PacketFilter beDown;
        beDown.remoteAddress  = Ipv4Address("10.1.1.0");
        beDown.remoteMask     = Ipv4Mask("255.255.255.0");
        beDown.localPortStart = BEST_EFFORT_PORT;
        beDown.localPortEnd   = BEST_EFFORT_PORT;
        beDown.direction      = NrEpcTft::DOWNLINK;
        beTft->Add(beDown);

        NrEpsBearer beBearer(NrEpsBearer::NGBR_VIDEO_TCP_DEFAULT);
        nrHelper->ActivateDedicatedEpsBearer(gatewayNrDevs.Get(gwIdx), beBearer, beTft);

        std::cout << "   GW" << (gwIdx + 1)
                  << ": Critical(QCI1) + Video(QCI4) + BestEffort(QCI9)\n";
    }

    std::cout << "\n";

    // ====================================================================
    // SECTION 12: TRAFFIC APPLICATIONS (12 SOURCES, 36 FLOWS)
    // ====================================================================

    std::cout << "[11/13] Installing traffic applications (" << sourceNodes.size()
              << " sources, " << (sourceNodes.size() * 3) << " flows)...\n\n";

    Ipv4Address mecIp = mecIpIface.GetAddress(1);

    // Sinks on MEC server
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

    for (uint32_t i = 0; i < sourceNodes.size(); i++)
    {
        uint32_t    nodeIdx   = sourceNodes[i];
        Ptr<Node>   srcNode   = meshOnlyNodes.Get(nodeIdx);
        Ipv4Address sourceIp  = meshIpIfaces.GetAddress(nodeIdx + numGateways);
        Time        startOff  = appStartTime + MilliSeconds(i * 100);

        std::cout << "   Source " << std::setw(2) << (i + 1)
                  << ": Node " << std::setw(2) << nodeIdx
                  << " (" << sourceIp << ")"
                  << " start=" << startOff.GetSeconds() << "s\n";

        // Critical
        OnOffHelper critOnOff("ns3::UdpSocketFactory",
                              InetSocketAddress(mecIp, CRITICAL_PORT));
        critOnOff.SetAttribute("DataRate",  DataRateValue(DataRate(std::to_string(criticalRate) + "kbps")));
        critOnOff.SetAttribute("PacketSize", UintegerValue(128));
        critOnOff.SetAttribute("Tos",       UintegerValue(TOS_CRITICAL));
        critOnOff.SetAttribute("OnTime",    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        critOnOff.SetAttribute("OffTime",   StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer critApp = critOnOff.Install(srcNode);
        critApp.Start(startOff);
        critApp.Stop(simTime - Seconds(0.5));

        // Video
        OnOffHelper vidOnOff("ns3::UdpSocketFactory",
                             InetSocketAddress(mecIp, VIDEO_PORT));
        vidOnOff.SetAttribute("DataRate",  DataRateValue(DataRate(std::to_string(videoRate) + "kbps")));
        vidOnOff.SetAttribute("PacketSize", UintegerValue(1400));
        vidOnOff.SetAttribute("Tos",       UintegerValue(TOS_VIDEO));
        vidOnOff.SetAttribute("OnTime",    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        vidOnOff.SetAttribute("OffTime",   StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer vidApp = vidOnOff.Install(srcNode);
        vidApp.Start(startOff);
        vidApp.Stop(simTime - Seconds(0.5));

        // Best Effort
        OnOffHelper beOnOff("ns3::UdpSocketFactory",
                            InetSocketAddress(mecIp, BEST_EFFORT_PORT));
        beOnOff.SetAttribute("DataRate",  DataRateValue(DataRate(std::to_string(beRate) + "kbps")));
        beOnOff.SetAttribute("PacketSize", UintegerValue(1400));
        beOnOff.SetAttribute("Tos",       UintegerValue(TOS_BESTEFFORT));
        beOnOff.SetAttribute("OnTime",    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        beOnOff.SetAttribute("OffTime",   StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer beApp = beOnOff.Install(srcNode);
        beApp.Start(startOff);
        beApp.Stop(simTime - Seconds(0.5));
    }

    std::cout << "\n   Total: " << sourceNodes.size() << " sources x 3 classes = "
              << (sourceNodes.size() * 3) << " flows\n";
    std::cout << "   Per-source rate: " << (criticalRate + videoRate + beRate) << " kbps\n";
    std::cout << "   Total offered: "
              << sourceNodes.size() * (criticalRate + videoRate + beRate) / 1000.0 << " Mbps\n\n";

    // ====================================================================
    // SECTION 13: FLOW MONITOR + BANDIT SELECTOR
    // ====================================================================

    std::cout << "[12/13] Setting up FlowMonitor and bandit selector...\n";

    FlowMonitorHelper         flowmonHelper;
    Ptr<FlowMonitor>          monitor    = flowmonHelper.InstallAll();
    Ptr<Ipv4FlowClassifier>   classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());

    // Construct the selector unconditionally (PrintStats is safe even in static mode)
    LinUCBGatewaySelector selector(gatewayNodes,
                                   meshOnlyNodes,
                                   gatewayNrDevs,
                                   gwEnergySources,
                                   meshIpIfaces,
                                   monitor,
                                   classifier,
                                   sourceNodes,
                                   numGateways,
                                   banditAlpha,
                                   Seconds(decisionIntervalSec));

    if (useBandit)
    {
        selector.Start(appStartTime);
        std::cout << "   LinUCB selector started (first decision at T=" << appStartTime.GetSeconds() << "s)\n";
    }
    else
    {
        std::cout << "   Static mode: bandit disabled\n";
    }

    std::cout << "\n";

    // ====================================================================
    // SECTION 14: RUN SIMULATION
    // ====================================================================

    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║              Starting Simulation (" << simTime.GetSeconds() << "s)                          ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";

    Simulator::Stop(simTime);
    Simulator::Run();

    // ====================================================================
    // SECTION 15: RESULTS
    // ====================================================================

    std::cout << "\n╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                       Simulation Results                         ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";

    if (useBandit)
        selector.PrintStats();

    // Per-class flow stats
    monitor->CheckForLostPackets();
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    struct QosStats
    {
        std::string name;
        uint16_t    port;
        uint32_t    txPackets  = 0;
        uint32_t    rxPackets  = 0;
        double      delaySum   = 0;
        double      throughput = 0;
    };

    QosStats criticalStats = {"Critical (QCI 1)", CRITICAL_PORT, 0, 0, 0, 0};
    QosStats videoStats    = {"Video (QCI 4)",    VIDEO_PORT,    0, 0, 0, 0};
    QosStats beStats       = {"Best Effort (QCI 9)", BEST_EFFORT_PORT, 0, 0, 0, 0};

    uint32_t flowCount      = 0;
    double   totalThroughput = 0.0;
    double   activeDuration = (simTime - appStartTime).GetSeconds();

    std::cout << "Per-Flow Statistics:\n";
    std::cout << "────────────────────────────────────────────────────────────────────\n";

    for (auto& flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);

        QosStats* cs = nullptr;
        std::string cn;
        if      (t.destinationPort == CRITICAL_PORT)    { cs = &criticalStats; cn = "CRITICAL"; }
        else if (t.destinationPort == VIDEO_PORT)        { cs = &videoStats;    cn = "VIDEO"; }
        else if (t.destinationPort == BEST_EFFORT_PORT)  { cs = &beStats;       cn = "BE"; }
        else continue;

        double pdr = 0, tp = 0, delay = 0;
        if (flow.second.txPackets > 0)
            pdr = 100.0 * flow.second.rxPackets / flow.second.txPackets;
        if (flow.second.rxPackets > 0)
        {
            tp    = flow.second.rxBytes * 8.0 / activeDuration / 1e6;
            delay = flow.second.delaySum.GetMilliSeconds() / flow.second.rxPackets;
        }

        cs->txPackets  += flow.second.txPackets;
        cs->rxPackets  += flow.second.rxPackets;
        cs->delaySum   += flow.second.delaySum.GetMilliSeconds();
        cs->throughput += tp;
        totalThroughput += tp;
        flowCount++;

        std::cout << "Flow " << std::setw(3) << flow.first << " [" << cn << "]: "
                  << t.sourceAddress << " -> " << t.destinationAddress
                  << ":" << t.destinationPort
                  << " | PDR: " << std::fixed << std::setprecision(1) << pdr << "%"
                  << " | " << std::setprecision(2) << tp << " Mbps"
                  << " | " << delay << " ms\n";
    }

    auto printQosStats = [](const QosStats& s) {
        double pdr   = (s.txPackets > 0) ? 100.0 * s.rxPackets / s.txPackets : 0;
        double delay = (s.rxPackets > 0) ? s.delaySum / s.rxPackets : 0;
        std::cout << "  " << s.name << ":\n";
        std::cout << "    TX/RX:      " << s.txPackets << "/" << s.rxPackets << " packets\n";
        std::cout << "    PDR:        " << std::fixed << std::setprecision(1) << pdr << "%\n";
        std::cout << "    Throughput: " << std::setprecision(3) << s.throughput << " Mbps\n";
        std::cout << "    Mean Delay: " << std::setprecision(2) << delay << " ms\n\n";
    };

    std::cout << "\n────────────────────────────────────────────────────────────────────\n";
    std::cout << "QoS Class Summary:\n";
    std::cout << "────────────────────────────────────────────────────────────────────\n";
    printQosStats(criticalStats);
    printQosStats(videoStats);
    printQosStats(beStats);

    uint32_t totalTx  = criticalStats.txPackets + videoStats.txPackets + beStats.txPackets;
    uint32_t totalRx  = criticalStats.rxPackets + videoStats.rxPackets + beStats.rxPackets;
    double   overallPdr = (totalTx > 0) ? 100.0 * totalRx / totalTx : 0;

    std::cout << "────────────────────────────────────────────────────────────────────\n";
    std::cout << "Overall Summary:\n";
    std::cout << "   Flows:           " << flowCount << "\n";
    std::cout << "   Total throughput: " << std::setprecision(2) << totalThroughput << " Mbps\n";
    std::cout << "   Overall PDR:      " << std::setprecision(1) << overallPdr << "%\n";
    std::cout << "   Mode:             " << (useBandit ? "LinUCB bandit" : "Static") << "\n";
    std::cout << "   Scenario:         " << scenario << "\n\n";

    // Generate mesh reports
    std::cout << "[13/13] Generating mesh reports...\n";
    for (uint32_t i = 0; i < std::min(meshDevices.GetN(), 4u); i++)
    {
        std::ostringstream fn;
        fn << simTag << "-mesh-report-" << i << ".xml";
        std::ofstream of(fn.str());
        if (of.is_open())
        {
            meshDevices.Get(i)->GetObject<MeshPointDevice>()->Report(of);
            of.close();
            std::cout << "   " << fn.str() << "\n";
        }
    }

    Simulator::Destroy();
    std::cout << "\nDone.\n";
    return 0;
}
