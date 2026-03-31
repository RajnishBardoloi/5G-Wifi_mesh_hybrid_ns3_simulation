// Copyright (c) 2008,2009 IITP RAS
// Modified for 5G-Mesh Integration - Days 3-4
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * \file mesh-test.cc
 * \brief 802.11s Mesh Network Standalone Test - Days 3-4
 *
 * This simulation creates a standalone IEEE 802.11s mesh network
 * to verify HWMP routing and multi-hop communication before
 * integration with 5G NR backhaul (Days 5-7).
 *
 * Day 3-4 Goals:
 * - Create 4-node mesh network in 2×2 grid (50m spacing)
 * - Configure Dot11sStack with HWMP routing
 * - Verify multi-hop routing (corner-to-corner forces multi-hop)
 * - Measure PDR, throughput, and routing statistics
 * - Target: PDR >= 80%
 *
 * Topology:
 *   (0,50) -- 50m -- (50,50)
 *      |                |
 *     50m              50m
 *      |                |
 *   (0,0)  -- 50m -- (50,0)
 *
 * Diagonal distance (0,0) to (50,50): 70.7m
 * Typical 802.11a range: ~50m → forces multi-hop routing
 *
 * Traffic: UDP Echo from Node 0 (0,0) to Node 3 (50,50)
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MeshTest");

// ========================================================================
// GLOBAL PACKET COUNTERS
// ========================================================================

uint32_t g_udpTxCount = 0;  ///< Total UDP packets transmitted
uint32_t g_udpRxCount = 0;  ///< Total UDP packets received

// ========================================================================
// TRACE CALLBACKS
// ========================================================================

/**
 * Callback for packet transmission
 * \param p The packet being transmitted
 */
void
TxTrace(Ptr<const Packet> p)
{
    NS_LOG_DEBUG("Sent " << p->GetSize() << " bytes");
    g_udpTxCount++;
}

/**
 * Callback for packet reception
 * \param p The packet being received
 */
void
RxTrace(Ptr<const Packet> p)
{
    NS_LOG_DEBUG("Received " << p->GetSize() << " bytes");
    g_udpRxCount++;
}

// ========================================================================
// MAIN FUNCTION
// ========================================================================

int
main(int argc, char* argv[])
{
    // ========================================================================
    // SECTION 1: COMMAND-LINE PARAMETERS
    // ========================================================================

    // Simulation parameters
    Time simTime = Seconds(10.0);      // Simulation time
    uint32_t xSize = 2;                // Grid width (columns)
    uint32_t ySize = 2;                // Grid height (rows)
    double step = 50.0;                // Grid spacing in meters

    // Traffic parameters
    uint32_t packetSize = 1024;        // UDP packet size in bytes
    double packetInterval = 1.0;       // Packet interval in seconds

    // Logging and tracing
    bool logging = false;              // Enable HWMP logging
    bool pcap = false;                 // Enable PCAP traces
    std::string simTag = "mesh-test";  // Output file tag

    // Command line parsing
    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue("xSize", "Grid width (columns)", xSize);
    cmd.AddValue("ySize", "Grid height (rows)", ySize);
    cmd.AddValue("step", "Grid spacing (meters)", step);
    cmd.AddValue("packetSize", "UDP packet size (bytes)", packetSize);
    cmd.AddValue("interval", "Packet interval (seconds)", packetInterval);
    cmd.AddValue("logging", "Enable HWMP logging", logging);
    cmd.AddValue("pcap", "Enable PCAP traces", pcap);
    cmd.AddValue("simTag", "Tag for output filename", simTag);
    cmd.Parse(argc, argv);

    // Enable logging if requested
    if (logging)
    {
        LogComponentEnable("MeshTest", LOG_LEVEL_INFO);
        LogComponentEnable("HwmpProtocol", LOG_LEVEL_INFO);
        LogComponentEnable("HwmpRtable", LOG_LEVEL_INFO);
        LogComponentEnable("MeshPointDevice", LOG_LEVEL_INFO);
        LogComponentEnable("Dot11sPeerManagementProtocol", LOG_LEVEL_INFO);
    }

    // Calculate total nodes and diagonal distance
    uint32_t totalNodes = xSize * ySize;
    double diagonal = std::sqrt(2 * step * step);

    // Configuration banner
    std::cout << "========================================\n";
    std::cout << "802.11s Mesh Network Test - Days 3-4\n";
    std::cout << "========================================\n";
    std::cout << "Grid: " << xSize << "×" << ySize << " with " << step << "m spacing\n";
    std::cout << "Total nodes: " << totalNodes << "\n";
    std::cout << "Standard: 802.11a (5 GHz)\n";
    std::cout << "Routing: HWMP (Hybrid Wireless Mesh Protocol)\n";
    std::cout << "Traffic: Node 0 → Node " << (totalNodes - 1) << " (corner-to-corner)\n";
    std::cout << "Diagonal distance: " << diagonal << "m (forces multi-hop)\n";
    std::cout << "Simulation time: " << simTime.GetSeconds() << "s\n";
    std::cout << "========================================\n\n";

    // ========================================================================
    // SECTION 2: NODE CREATION AND GRID POSITIONING
    // ========================================================================

    std::cout << "Creating mesh nodes...\n";

    // Create mesh nodes
    NodeContainer nodes;
    nodes.Create(totalNodes);

    // Setup mobility with grid layout
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX",
                                  DoubleValue(0.0),
                                  "MinY",
                                  DoubleValue(0.0),
                                  "DeltaX",
                                  DoubleValue(step),
                                  "DeltaY",
                                  DoubleValue(step),
                                  "GridWidth",
                                  UintegerValue(xSize),
                                  "LayoutType",
                                  StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    std::cout << "  Created " << nodes.GetN() << " mesh nodes\n";
    std::cout << "  Node 0: (0, 0, 1.5) - Source\n";
    std::cout << "  Node 1: (" << step << ", 0, 1.5)\n";
    std::cout << "  Node 2: (0, " << step << ", 1.5)\n";
    std::cout << "  Node 3: (" << step << ", " << step << ", 1.5) - Destination\n";
    std::cout << "  802.11a typical range: ~50m\n";
    std::cout << "  Diagonal > 50m requires multi-hop routing\n\n";

    // ========================================================================
    // SECTION 3: WIFI CHANNEL SETUP (802.11a)
    // ========================================================================

    std::cout << "Configuring WiFi channel (802.11a)...\n";

    // Configure WiFi channel with default propagation models
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());

    std::cout << "  WiFi PHY configured (5 GHz band, default propagation)\n\n";

    // ========================================================================
    // SECTION 4: MESH HELPER CONFIGURATION
    // ========================================================================

    std::cout << "Configuring mesh stack (Dot11s with HWMP)...\n";

    // Configure mesh helper
    MeshHelper mesh = MeshHelper::Default();
    mesh.SetStackInstaller("ns3::Dot11sStack"); // Install Dot11s stack with HWMP routing
    mesh.SetSpreadInterfaceChannels(
        MeshHelper::ZERO_CHANNEL);              // All nodes on same channel (simpler)
    mesh.SetNumberOfInterfaces(1);              // Single radio per node
    mesh.SetMacType("RandomStart", TimeValue(Seconds(0.1))); // Avoid beacon collisions
    mesh.SetStandard(WIFI_STANDARD_80211a);     // 802.11a standard

    std::cout << "  Stack: ns3::Dot11sStack (HWMP routing enabled)\n";
    std::cout << "  Channels: ZERO_CHANNEL (all nodes on same channel)\n";
    std::cout << "  Interfaces: 1 per node\n";
    std::cout << "  Beacon randomization: 0.1s to avoid collisions\n";
    std::cout << "  Standard: 802.11a\n\n";

    // ========================================================================
    // SECTION 5: INSTALL MESH DEVICES
    // ========================================================================

    std::cout << "Installing mesh devices...\n";

    // Install mesh devices on all nodes
    NetDeviceContainer meshDevices = mesh.Install(wifiPhy, nodes);
    mesh.AssignStreams(meshDevices, 0); // Set random stream for reproducibility

    std::cout << "  Mesh devices installed on " << meshDevices.GetN() << " nodes\n";

    // Enable PCAP if requested
    if (pcap)
    {
        wifiPhy.EnablePcapAll(simTag);
        std::cout << "  PCAP tracing enabled (prefix: " << simTag << ")\n";
    }
    std::cout << "\n";

    // ========================================================================
    // SECTION 6: INTERNET STACK AND IP ADDRESSING
    // ========================================================================

    std::cout << "Installing Internet stack and assigning IP addresses...\n";

    // Install Internet stack on all nodes
    InternetStackHelper internet;
    internet.Install(nodes);

    // Assign IP addresses to mesh devices
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(meshDevices);

    std::cout << "  IP addresses assigned (10.1.1.0/24 subnet):\n";
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        std::cout << "    Node " << i << ": " << interfaces.GetAddress(i) << "\n";
    }
    std::cout << "\n";

    // ========================================================================
    // SECTION 7: UDP TRAFFIC SETUP (CORNER-TO-CORNER)
    // ========================================================================

    std::cout << "Setting up UDP Echo traffic (corner-to-corner)...\n";

    uint16_t port = 9;                            // UDP Echo port
    uint32_t sinkNodeId = xSize * ySize - 1;      // Last node (top-right corner)
    uint32_t maxPackets =
        static_cast<uint32_t>(simTime.GetSeconds() / packetInterval); // Total packets to send

    // UDP Echo Server on corner node (Node 3)
    UdpEchoServerHelper echoServer(port);
    ApplicationContainer serverApps = echoServer.Install(nodes.Get(sinkNodeId));
    serverApps.Start(Seconds(1.0)); // Start at 1s (allow peering time)
    serverApps.Stop(simTime + Seconds(1.0));

    // UDP Echo Client on opposite corner (Node 0)
    UdpEchoClientHelper echoClient(interfaces.GetAddress(sinkNodeId), port);
    echoClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
    echoClient.SetAttribute("PacketSize", UintegerValue(packetSize));

    ApplicationContainer clientApps = echoClient.Install(nodes.Get(0));

    // Connect trace callbacks to track packets
    Ptr<UdpEchoClient> app = clientApps.Get(0)->GetObject<UdpEchoClient>();
    app->TraceConnectWithoutContext("Tx", MakeCallback(&TxTrace));
    app->TraceConnectWithoutContext("Rx", MakeCallback(&RxTrace));

    clientApps.Start(Seconds(1.0)); // Start at 1s
    clientApps.Stop(simTime + Seconds(1.5));

    std::cout << "  UDP Echo Server:\n";
    std::cout << "    Node: " << sinkNodeId << " (Node " << sinkNodeId << ")\n";
    std::cout << "    IP: " << interfaces.GetAddress(sinkNodeId) << ":" << port << "\n";
    std::cout << "    Start: 1.0s, Stop: " << (simTime + Seconds(1.0)).GetSeconds() << "s\n";
    std::cout << "\n";
    std::cout << "  UDP Echo Client:\n";
    std::cout << "    Node: 0 (Node 0)\n";
    std::cout << "    IP: " << interfaces.GetAddress(0) << "\n";
    std::cout << "    Target: " << interfaces.GetAddress(sinkNodeId) << ":" << port << "\n";
    std::cout << "    Packets: " << maxPackets << " × " << packetSize << " bytes\n";
    std::cout << "    Interval: " << packetInterval << "s\n";
    std::cout << "    Start: 1.0s, Stop: " << (simTime + Seconds(1.5)).GetSeconds() << "s\n";
    std::cout << "\n";

    // ========================================================================
    // SECTION 8: MESH REPORT SCHEDULING
    // ========================================================================

    // Schedule mesh report at end of simulation (prints to console)
    Simulator::Schedule(simTime,
                        &MeshHelper::Report,
                        &mesh,
                        meshDevices.Get(0),
                        std::ref(std::cout));

    // ========================================================================
    // SECTION 9: RUN SIMULATION
    // ========================================================================

    std::cout << "========================================\n";
    std::cout << "Starting simulation...\n";
    std::cout << "========================================\n\n";

    Simulator::Stop(simTime + Seconds(2.0)); // Extra time for cleanup
    Simulator::Run();

    // ========================================================================
    // SECTION 10: RESULTS COLLECTION AND ANALYSIS
    // ========================================================================

    std::cout << "\n========================================\n";
    std::cout << "Simulation Complete - Results\n";
    std::cout << "========================================\n\n";

    // Calculate PDR (Packet Delivery Ratio)
    double pdr = 0.0;
    if (g_udpTxCount > 0)
    {
        pdr = static_cast<double>(g_udpRxCount) / static_cast<double>(g_udpTxCount) * 100.0;
    }

    // Print UDP Echo statistics
    std::cout << "UDP Echo Statistics:\n";
    std::cout << "  Packets sent: " << g_udpTxCount << "\n";
    std::cout << "  Packets received: " << g_udpRxCount << "\n";
    std::cout << "  Lost packets: " << (g_udpTxCount - g_udpRxCount) << "\n";
    std::cout << "  PDR: " << std::fixed << std::setprecision(1) << pdr << " %\n\n";

    // Check success criteria
    if (pdr >= 80.0)
    {
        std::cout << "✓ SUCCESS: PDR >= 80% (Days 3-4 goal achieved!)\n";
        std::cout << "Multi-hop mesh routing is working correctly.\n";
    }
    else if (pdr >= 50.0)
    {
        std::cout << "⚠ PARTIAL SUCCESS: PDR is " << pdr << "% (expected >= 80%)\n";
        std::cout << "Some packets getting through, but mesh may need tuning.\n";
        std::cout << "Check mp-report-*.xml for routing details.\n";
    }
    else if (pdr > 0.0)
    {
        std::cout << "✗ LOW PDR: PDR is " << pdr << "% (expected >= 80%)\n";
        std::cout << "Mesh routing may not be fully established.\n";
        std::cout << "Try: ./ns3 run \"mesh-test --simTime=20 --logging=true\"\n";
    }
    else
    {
        std::cout << "✗ FAILURE: No packets received (PDR = 0%)\n";
        std::cout << "Possible issues:\n";
        std::cout << "  - Peering not established (increase simTime)\n";
        std::cout << "  - Route discovery failing (check logs with --logging=true)\n";
        std::cout << "  - Distance too far (try --step=35 for closer nodes)\n";
    }

    std::cout << "\n";

    // Generate XML reports for each node
    std::cout << "Generating mesh reports (HWMP routing tables, peer statistics)...\n";
    for (uint32_t i = 0; i < meshDevices.GetN(); ++i)
    {
        std::ostringstream filename;
        filename << "mp-report-" << i << ".xml";
        std::ofstream of(filename.str().c_str());
        if (of.is_open())
        {
            mesh.Report(meshDevices.Get(i), of);
            of.close();
            std::cout << "  Generated: " << filename.str() << "\n";
        }
    }

    std::cout << "\nTo verify multi-hop routing, check mp-report-0.xml:\n";
    std::cout << "  grep -A 5 \"Destination>10.1.1.4\" mp-report-0.xml\n";
    std::cout << "  Look for Gateway/Retransmitter != 10.1.1.4 (proves intermediate hop)\n";

    std::cout << "\n========================================\n";
    std::cout << "Days 3-4 Complete!\n";
    std::cout << "========================================\n";

    Simulator::Destroy();
    return 0;
}
