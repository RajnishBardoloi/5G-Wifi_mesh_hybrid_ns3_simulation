// Copyright (c) 2024 5G-Mesh Integration Project
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * \file pure-wifi-mesh-baseline.cc
 * \brief Pure WiFi Mesh Baseline (No 5G) - For Comparison
 *
 * Baseline A from paper: Pure Wi-Fi Mesh (IEEE 802.11s)
 * All nodes communicate via multi-hop Wi-Fi to a single wired sink.
 * This represents a low-cost, decentralized legacy solution.
 *
 * Used to compare against hybrid 5G-Mesh architecture.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("PureWifiMeshBaseline");

// ========================================================================
// QoS PORT DEFINITIONS (same as hybrid for fair comparison)
// ========================================================================
const uint16_t CRITICAL_PORT = 5050;
const uint16_t VIDEO_PORT = 5150;
const uint16_t BEST_EFFORT_PORT = 5250;

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
    Time appStartTime = Seconds(15.0);

    // Grid parameters (same as hybrid)
    uint32_t gridWidth = 8;
    uint32_t gridHeight = 6;
    double meshSpacing = 30.0;

    // Traffic parameters (same as hybrid)
    uint32_t criticalRate = 50;
    uint32_t videoRate = 200;
    uint32_t beRate = 200;

    bool logging = false;

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue("appStartTime", "App start time (seconds)", appStartTime);
    cmd.AddValue("gridWidth", "Grid width (columns)", gridWidth);
    cmd.AddValue("gridHeight", "Grid height (rows)", gridHeight);
    cmd.AddValue("meshSpacing", "Mesh node spacing (meters)", meshSpacing);
    cmd.AddValue("criticalRate", "Critical traffic rate (kbps)", criticalRate);
    cmd.AddValue("videoRate", "Video traffic rate (kbps)", videoRate);
    cmd.AddValue("beRate", "Best effort traffic rate (kbps)", beRate);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.Parse(argc, argv);

    if (logging)
    {
        LogComponentEnable("PureWifiMeshBaseline", LOG_LEVEL_INFO);
        LogComponentEnable("HwmpProtocol", LOG_LEVEL_DEBUG);
    }

    uint32_t numMeshNodes = gridWidth * gridHeight;

    // Print configuration
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║        BASELINE A: Pure Wi-Fi Mesh (IEEE 802.11s)                ║\n";
    std::cout << "║              No 5G Backhaul - Legacy Solution                    ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Topology:                                                        ║\n";
    std::cout << "║   Grid: " << gridWidth << "x" << gridHeight << " (" << numMeshNodes << " mesh nodes)                                ║\n";
    std::cout << "║   Sink: 1 (wired, center position)                               ║\n";
    std::cout << "║   Mesh Spacing: " << meshSpacing << "m                                             ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Note: NO QoS differentiation in pure mesh                        ║\n";
    std::cout << "║   All traffic treated equally (best-effort)                      ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Traffic:                                                         ║\n";
    std::cout << "║   Source nodes: 6 (same positions as hybrid)                     ║\n";
    std::cout << "║   Flows per source: 3 (Critical, Video, Best Effort)             ║\n";
    std::cout << "║   Total flows: 18                                                ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Simulation Time: " << simTime.GetSeconds() << " seconds                                      ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";

    // ====================================================================
    // SECTION 2: NODE CREATION
    // ====================================================================

    std::cout << "[1/8] Creating nodes...\n";

    // Create mesh nodes (48 nodes in 8x6 grid)
    NodeContainer meshNodes;
    meshNodes.Create(numMeshNodes);

    // Create wired sink (represents the data collection point)
    NodeContainer sinkNode;
    sinkNode.Create(1);

    // Gateway node (mesh node that connects to wired sink)
    // Use center node as gateway (position 2,3 = node 26)
    uint32_t gatewayIdx = GetNodeIndex(2, 3, gridWidth);

    std::cout << "   Mesh nodes: " << numMeshNodes << "\n";
    std::cout << "   Wired sink: 1\n";
    std::cout << "   Gateway (mesh-to-wired): Node " << gatewayIdx << "\n\n";

    // ====================================================================
    // SECTION 3: NODE POSITIONING
    // ====================================================================

    std::cout << "[2/8] Setting up node positions...\n";

    MobilityHelper mobility;

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
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(meshNodes);

    // Sink position (next to gateway)
    double sinkX = 2 * meshSpacing + 5;  // Slightly offset from gateway
    double sinkY = 3 * meshSpacing;

    Ptr<ListPositionAllocator> sinkPosition = CreateObject<ListPositionAllocator>();
    sinkPosition->Add(Vector(sinkX, sinkY, 1.5));
    mobility.SetPositionAllocator(sinkPosition);
    mobility.Install(sinkNode);

    std::cout << "   Mesh nodes: " << numMeshNodes << " in " << gridWidth << "x" << gridHeight << " grid\n";
    std::cout << "   Sink: (" << sinkX << ", " << sinkY << ")\n\n";

    // ====================================================================
    // SECTION 4: MESH NETWORK SETUP
    // ====================================================================

    std::cout << "[3/8] Setting up 802.11s mesh network...\n";

    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());

    MeshHelper mesh = MeshHelper::Default();
    mesh.SetStackInstaller("ns3::Dot11sStack");
    mesh.SetSpreadInterfaceChannels(MeshHelper::ZERO_CHANNEL);
    mesh.SetNumberOfInterfaces(1);
    mesh.SetMacType("RandomStart", TimeValue(Seconds(0.1)));
    mesh.SetStandard(WIFI_STANDARD_80211a);

    NetDeviceContainer meshDevices = mesh.Install(wifiPhy, meshNodes);

    std::cout << "   Mesh devices installed on " << meshDevices.GetN() << " nodes\n";
    std::cout << "   Standard: 802.11a (5 GHz)\n";
    std::cout << "   Note: NO QoS differentiation (all best-effort)\n\n";

    // ====================================================================
    // SECTION 5: INTERNET STACK AND IP CONFIGURATION
    // ====================================================================

    std::cout << "[4/8] Configuring Internet stack and IP addresses...\n";

    InternetStackHelper internet;
    internet.Install(meshNodes);
    internet.Install(sinkNode);

    // Point-to-point link from gateway mesh node to sink
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Mbps")));  // Wired link
    p2p.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));

    NetDeviceContainer p2pDevices = p2p.Install(meshNodes.Get(gatewayIdx), sinkNode.Get(0));

    // Assign mesh IPs
    Ipv4AddressHelper meshIpHelper;
    meshIpHelper.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer meshIpIfaces = meshIpHelper.Assign(meshDevices);

    // Assign P2P IPs
    Ipv4AddressHelper p2pIpHelper;
    p2pIpHelper.SetBase("10.2.1.0", "255.255.255.0");
    Ipv4InterfaceContainer p2pIpIfaces = p2pIpHelper.Assign(p2pDevices);

    std::cout << "   Gateway mesh IP: " << meshIpIfaces.GetAddress(gatewayIdx) << "\n";
    std::cout << "   Gateway P2P IP: " << p2pIpIfaces.GetAddress(0) << "\n";
    std::cout << "   Sink IP: " << p2pIpIfaces.GetAddress(1) << "\n\n";

    // ====================================================================
    // SECTION 6: ROUTING CONFIGURATION
    // ====================================================================

    std::cout << "[5/8] Configuring routing...\n";

    Ipv4StaticRoutingHelper routingHelper;

    // All mesh nodes route to sink via gateway
    Ipv4Address gatewayMeshIp = meshIpIfaces.GetAddress(gatewayIdx);

    for (uint32_t i = 0; i < numMeshNodes; ++i)
    {
        if (i != gatewayIdx)
        {
            Ptr<Ipv4StaticRouting> nodeRouting = routingHelper.GetStaticRouting(
                meshNodes.Get(i)->GetObject<Ipv4>());
            nodeRouting->SetDefaultRoute(gatewayMeshIp, 1);
        }
    }

    // Gateway routes to sink
    Ptr<Ipv4StaticRouting> gwRouting = routingHelper.GetStaticRouting(
        meshNodes.Get(gatewayIdx)->GetObject<Ipv4>());
    gwRouting->AddNetworkRouteTo(Ipv4Address("10.2.1.0"), Ipv4Mask("255.255.255.0"), 2);

    // Enable IP forwarding on gateway
    meshNodes.Get(gatewayIdx)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));

    // Sink routes back to mesh
    Ptr<Ipv4StaticRouting> sinkRouting = routingHelper.GetStaticRouting(
        sinkNode.Get(0)->GetObject<Ipv4>());
    sinkRouting->AddNetworkRouteTo(Ipv4Address("10.1.1.0"), Ipv4Mask("255.255.255.0"), 1);

    std::cout << "   All mesh nodes route via gateway (node " << gatewayIdx << ")\n\n";

    // ====================================================================
    // SECTION 7: TRAFFIC APPLICATIONS
    // ====================================================================

    std::cout << "[6/8] Setting up traffic applications (6 sources, 18 flows)...\n\n";

    // Source node selection (same as hybrid for fair comparison)
    std::vector<uint32_t> sourceNodes;
    for (uint32_t row = 0; row < gridHeight; ++row)
    {
        uint32_t col = (row % 2 == 0) ? 1 : 6;
        uint32_t nodeIdx = GetNodeIndex(col, row, gridWidth);
        sourceNodes.push_back(nodeIdx);
    }

    // Install sink applications
    Ipv4Address sinkIp = p2pIpIfaces.GetAddress(1);

    PacketSinkHelper criticalSink("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), CRITICAL_PORT));
    PacketSinkHelper videoSink("ns3::UdpSocketFactory",
                               InetSocketAddress(Ipv4Address::GetAny(), VIDEO_PORT));
    PacketSinkHelper beSink("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), BEST_EFFORT_PORT));

    ApplicationContainer sinkApps;
    sinkApps.Add(criticalSink.Install(sinkNode.Get(0)));
    sinkApps.Add(videoSink.Install(sinkNode.Get(0)));
    sinkApps.Add(beSink.Install(sinkNode.Get(0)));
    sinkApps.Start(Seconds(1.0));
    sinkApps.Stop(simTime);

    // Install traffic generators
    for (uint32_t i = 0; i < sourceNodes.size(); ++i)
    {
        uint32_t nodeIdx = sourceNodes[i];
        Ptr<Node> sourceNode = meshNodes.Get(nodeIdx);
        Ipv4Address sourceIp = meshIpIfaces.GetAddress(nodeIdx);

        std::cout << "   Source " << (i+1) << ": Node " << nodeIdx
                  << " (" << sourceIp << ")\n";

        Time startOffset = appStartTime + MilliSeconds(i * 100);

        // CRITICAL TRAFFIC (no priority in pure mesh - treated as best effort)
        OnOffHelper criticalOnOff("ns3::UdpSocketFactory",
                                  InetSocketAddress(sinkIp, CRITICAL_PORT));
        criticalOnOff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(criticalRate) + "kbps")));
        criticalOnOff.SetAttribute("PacketSize", UintegerValue(128));
        criticalOnOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        criticalOnOff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer critApp = criticalOnOff.Install(sourceNode);
        critApp.Start(startOffset);
        critApp.Stop(simTime - Seconds(0.5));

        // VIDEO TRAFFIC
        OnOffHelper videoOnOff("ns3::UdpSocketFactory",
                               InetSocketAddress(sinkIp, VIDEO_PORT));
        videoOnOff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(videoRate) + "kbps")));
        videoOnOff.SetAttribute("PacketSize", UintegerValue(1400));
        videoOnOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        videoOnOff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer videoApp = videoOnOff.Install(sourceNode);
        videoApp.Start(startOffset);
        videoApp.Stop(simTime - Seconds(0.5));

        // BEST EFFORT TRAFFIC
        OnOffHelper beOnOff("ns3::UdpSocketFactory",
                            InetSocketAddress(sinkIp, BEST_EFFORT_PORT));
        beOnOff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(beRate) + "kbps")));
        beOnOff.SetAttribute("PacketSize", UintegerValue(1400));
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
    // SECTION 8: FLOW MONITOR AND RUN
    // ====================================================================

    std::cout << "[7/8] Setting up FlowMonitor...\n";

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    std::cout << "   FlowMonitor installed\n\n";

    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                Starting Pure Mesh Simulation                     ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";

    Simulator::Stop(simTime);
    Simulator::Run();

    // ====================================================================
    // RESULTS ANALYSIS
    // ====================================================================

    std::cout << "\n╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║           BASELINE A: Pure Wi-Fi Mesh Results                    ║\n";
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

    QosStats criticalStats = {"Critical", CRITICAL_PORT, 0, 0, 0, 0};
    QosStats videoStats = {"Video", VIDEO_PORT, 0, 0, 0, 0};
    QosStats beStats = {"Best Effort", BEST_EFFORT_PORT, 0, 0, 0, 0};

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
            continue;
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
    std::cout << "Traffic Class Summary (NO QoS - All Best Effort):\n";
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

    uint32_t totalTx = criticalStats.txPackets + videoStats.txPackets + beStats.txPackets;
    uint32_t totalRx = criticalStats.rxPackets + videoStats.rxPackets + beStats.rxPackets;
    double overallPdr = (totalTx > 0) ? 100.0 * totalRx / totalTx : 0;
    std::cout << "   Overall PDR: " << std::setprecision(1) << overallPdr << "%\n";

    double overallDelay = 0;
    if (totalRx > 0) {
        overallDelay = (criticalStats.delaySum + videoStats.delaySum + beStats.delaySum) / totalRx;
    }
    std::cout << "   Overall Mean Delay: " << std::setprecision(2) << overallDelay << " ms\n";

    std::cout << "\n╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  BASELINE A COMPLETE - Compare with Hybrid 5G-Mesh results       ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";

    flowmonHelper.SerializeToXmlFile("baseline-pure-mesh-flowmon.xml", true, true);
    std::cout << "\n[8/8] Generated: baseline-pure-mesh-flowmon.xml\n\n";

    Simulator::Destroy();
    return 0;
}
