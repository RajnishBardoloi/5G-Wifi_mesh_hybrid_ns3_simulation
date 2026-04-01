// Copyright (c) 2024 5G-Mesh Integration Project
// SPDX-License-Identifier: GPL-2.0-only
//
// linucb-gateway-selector.h
// LinUCB Contextual Bandit for per-node 5G-Mesh Gateway Selection
//
// Each of 12 source nodes independently runs LinUCB with 4 arms (one per gateway).
// Context vector per arm: [queue_load, residual_energy, sinr_normalized, bias(1.0)]
// Reward: PDR / (1 + mean_delay_ms / 100)  (range approx [0,1])

#pragma once

#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"

#include <cmath>
#include <map>
#include <vector>

using namespace ns3;

// ============================================================
// 4x4 Matrix utility  (no Eigen / external deps)
// ============================================================

struct Matrix4x4
{
    double m[4][4];

    static Matrix4x4 Identity();
    static Matrix4x4 Zero();

    Matrix4x4 operator+(const Matrix4x4& rhs) const;
    Matrix4x4& operator+=(const Matrix4x4& rhs);

    // Outer product: returns v * v^T  (rank-1 4x4 matrix)
    static Matrix4x4 OuterProduct(const double v[4]);

    // out = this * in
    void MultVec(const double in[4], double out[4]) const;

    // Returns v^T * this * v  (scalar)
    double QuadForm(const double v[4]) const;

    // Returns sum_j( this[row][j] * v[j] )  (dot of one row with v)
    double RowDot(int row, const double v[4]) const;

    // Cofactor-based full 4x4 inverse.  Returns false if singular.
    // Use as verification / fallback -- normally we use Sherman-Morrison.
    bool Invert(Matrix4x4& inv) const;
};

// Sherman-Morrison rank-1 inverse update:
//   After  A += x*x^T ,  update  A_inv  in O(d^2) = O(16) operations.
//   A_inv_new = A_inv - (A_inv*x * x^T*A_inv) / (1 + x^T * A_inv * x)
void UpdateInverse(Matrix4x4& A_inv, const double x[4]);

// ============================================================
// Per-gateway arm state
// ============================================================

struct LinUCBArm
{
    uint32_t  gatewayIndex;
    Matrix4x4 A;        // 4x4, init = Identity
    double    b[4];     // 4x1, init = zeros
    Matrix4x4 A_inv;    // Maintained incrementally via Sherman-Morrison
    uint32_t  pullCount;

    void Reset(uint32_t gwIdx);

    // Incorporate new observation: context x, scalar reward r
    //   A += x*x^T ,  b += r*x ,  A_inv updated via Sherman-Morrison
    void Update(const double x[4], double reward);

    // LinUCB score = theta^T * x + alpha * sqrt(x^T * A_inv * x)
    //   where theta = A_inv * b
    double ComputeScore(const double x[4], double alpha) const;
};

// ============================================================
// Per-source-node learner  (independent 4-arm bandit)
// ============================================================

struct NodeLearner
{
    uint32_t             meshNodeIndex;    // Index into meshOnlyNodes
    uint32_t             currentGateway;  // Currently active gateway (0-based)
    std::vector<LinUCBArm> arms;          // One arm per gateway

    double   lastContext[4];              // Context stored from last MakeDecisions call
    uint32_t lastChosenArm;              // Arm chosen last round (for delayed update)

    void Init(uint32_t nodeIdx, uint32_t numGateways);
};

// ============================================================
// Gateway selector: orchestrates all per-node learners
// ============================================================

class LinUCBGatewaySelector
{
  public:
    LinUCBGatewaySelector(NodeContainer                    gatewayNodes,
                          NodeContainer                    meshOnlyNodes,
                          NetDeviceContainer               gatewayNrDevs,
                          energy::EnergySourceContainer    gwEnergySources,
                          Ipv4InterfaceContainer           meshIpIfaces,
                          Ptr<FlowMonitor>                 monitor,
                          Ptr<Ipv4FlowClassifier>          classifier,
                          std::vector<uint32_t>            sourceNodeIndices,
                          uint32_t                         numGateways,
                          double                           alpha,
                          Time                             decisionInterval);

    // Connect DlDataSinr traces, initialise learners, schedule first round
    void Start(Time firstDecisionTime);

    // Called every decisionInterval.  Updates arms, picks new gateways.
    void MakeDecisions();

    // After Simulator::Run(): print summary box + export bandit-decisions.csv
    void PrintStats() const;

  private:
    // Reward for one learner from FlowMonitor delta (uses m_prevSnapshot)
    double ComputeNodeReward(uint32_t learnerIdx,
                             const FlowMonitor::FlowStatsContainer& stats) const;

    // Extract 4D context for gateway gwIdx into context[]
    void ExtractContext(uint32_t gwIdx, double context[4]) const;

    // Dynamically update static routing of mesh node to point at newGwIdx
    void SwitchGateway(uint32_t meshNodeIdx, uint32_t newGwIdx);

    // Snapshot FlowMonitor stats for next round's delta computation
    void UpdateSnapshot(const FlowMonitor::FlowStatsContainer& stats);

    // DlDataSinr trace callback (static, bound with self+gwIndex via MakeBoundCallback)
    static void SinrCallback(LinUCBGatewaySelector* self,
                             uint32_t               gwIndex,
                             uint16_t               cellId,
                             uint16_t               rnti,
                             double                 avgSinrLinear,
                             uint16_t               bwpId);

    // ---- references to simulation objects ----
    NodeContainer                 m_gatewayNodes;
    NodeContainer                 m_meshOnlyNodes;
    NetDeviceContainer            m_gatewayNrDevs;
    energy::EnergySourceContainer m_gwEnergySources;
    Ipv4InterfaceContainer        m_meshIpIfaces;
    Ptr<FlowMonitor>              m_flowMonitor;
    Ptr<Ipv4FlowClassifier>       m_classifier;
    std::vector<uint32_t>         m_sourceNodeIndices;

    // ---- bandit parameters ----
    uint32_t m_numGateways;
    double   m_alpha;
    Time     m_interval;
    bool     m_firstCall;
    uint32_t m_roundCount;

    // ---- per-node learners ----
    std::vector<NodeLearner> m_learners;

    // ---- shared gateway state (updated every round) ----
    std::vector<double>   m_latestSinr;      // Latest DlDataSinr per gateway (linear)
    std::vector<uint32_t> m_fwdPacketDelta;  // Tx-packet delta per gateway this round

    // ---- FlowMonitor snapshot for delta computation ----
    struct FlowSnapshot
    {
        uint32_t rxPackets;
        uint32_t txPackets;
        double   delaySumMs; // stored as double milliseconds
    };
    std::map<uint32_t, FlowSnapshot> m_prevSnapshot;

    // ---- per-round decision log (for CSV and PrintStats) ----
    struct DecisionRecord
    {
        double   timeSec;
        uint32_t roundNum;
        uint32_t sourceNodeIdx; // index into meshOnlyNodes
        uint32_t chosenGw;
        double   scores[4];
        double   reward;
        double   ctxLoad;
        double   ctxEnergy;
        double   ctxSinr;
    };
    std::vector<DecisionRecord> m_log;
};
