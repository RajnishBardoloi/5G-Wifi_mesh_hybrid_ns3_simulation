// Copyright (c) 2024 5G-Mesh Integration Project
// SPDX-License-Identifier: GPL-2.0-only

#include "linucb-gateway-selector.h"

#include "ns3/internet-module.h"
#include "ns3/nr-module.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

using namespace ns3;

// ============================================================
// Internal helpers (file-scope)
// ============================================================

static double Det3(const double a[3][3])
{
    return a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
           a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
           a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
}

static void Submatrix3(const double src[4][4], int excludeRow, int excludeCol, double dst[3][3])
{
    int di = 0;
    for (int i = 0; i < 4; i++)
    {
        if (i == excludeRow)
            continue;
        int dj = 0;
        for (int j = 0; j < 4; j++)
        {
            if (j == excludeCol)
                continue;
            dst[di][dj] = src[i][j];
            dj++;
        }
        di++;
    }
}

// ============================================================
// Matrix4x4
// ============================================================

Matrix4x4
Matrix4x4::Identity()
{
    Matrix4x4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r.m[i][j] = (i == j) ? 1.0 : 0.0;
    return r;
}

Matrix4x4
Matrix4x4::Zero()
{
    Matrix4x4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r.m[i][j] = 0.0;
    return r;
}

Matrix4x4
Matrix4x4::operator+(const Matrix4x4& rhs) const
{
    Matrix4x4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r.m[i][j] = m[i][j] + rhs.m[i][j];
    return r;
}

Matrix4x4&
Matrix4x4::operator+=(const Matrix4x4& rhs)
{
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            m[i][j] += rhs.m[i][j];
    return *this;
}

Matrix4x4
Matrix4x4::OuterProduct(const double v[4])
{
    Matrix4x4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r.m[i][j] = v[i] * v[j];
    return r;
}

void
Matrix4x4::MultVec(const double in[4], double out[4]) const
{
    for (int i = 0; i < 4; i++)
    {
        out[i] = 0.0;
        for (int j = 0; j < 4; j++)
            out[i] += m[i][j] * in[j];
    }
}

double
Matrix4x4::QuadForm(const double v[4]) const
{
    // v^T * M * v
    double Mv[4];
    MultVec(v, Mv);
    double result = 0.0;
    for (int i = 0; i < 4; i++)
        result += v[i] * Mv[i];
    return result;
}

double
Matrix4x4::RowDot(int row, const double v[4]) const
{
    double s = 0.0;
    for (int j = 0; j < 4; j++)
        s += m[row][j] * v[j];
    return s;
}

bool
Matrix4x4::Invert(Matrix4x4& inv) const
{
    double cof[4][4];
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            double sub[3][3];
            Submatrix3(m, i, j, sub);
            double sign = ((i + j) % 2 == 0) ? 1.0 : -1.0;
            cof[i][j] = sign * Det3(sub);
        }
    }

    double det = 0.0;
    for (int j = 0; j < 4; j++)
        det += m[0][j] * cof[0][j];

    if (std::abs(det) < 1e-14)
        return false;

    // inv = adjugate / det   (adjugate = transpose of cofactor matrix)
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            inv.m[i][j] = cof[j][i] / det;

    return true;
}

// ============================================================
// UpdateInverse (Sherman-Morrison)
// ============================================================

void
UpdateInverse(Matrix4x4& A_inv, const double x[4])
{
    // Ax = A_inv * x
    double Ax[4];
    A_inv.MultVec(x, Ax);

    // denom = 1 + x^T * A_inv * x  = 1 + x . Ax
    double denom = 1.0;
    for (int i = 0; i < 4; i++)
        denom += x[i] * Ax[i];

    // A_inv -= Ax * Ax^T / denom
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            A_inv.m[i][j] -= Ax[i] * Ax[j] / denom;
}

// ============================================================
// LinUCBArm
// ============================================================

void
LinUCBArm::Reset(uint32_t gwIdx)
{
    gatewayIndex = gwIdx;
    A            = Matrix4x4::Identity();
    A_inv        = Matrix4x4::Identity();
    for (int i = 0; i < 4; i++)
        b[i] = 0.0;
    pullCount = 0;
}

void
LinUCBArm::Update(const double x[4], double reward)
{
    A     += Matrix4x4::OuterProduct(x);
    for (int i = 0; i < 4; i++)
        b[i] += reward * x[i];
    UpdateInverse(A_inv, x);
    pullCount++;
}

double
LinUCBArm::ComputeScore(const double x[4], double alpha) const
{
    // theta = A_inv * b
    double theta[4];
    A_inv.MultVec(b, theta);

    // exploitation = theta^T * x
    double exploitation = 0.0;
    for (int i = 0; i < 4; i++)
        exploitation += theta[i] * x[i];

    // exploration = alpha * sqrt(x^T * A_inv * x)
    double quadratic = A_inv.QuadForm(x);
    double exploration = alpha * std::sqrt(std::max(quadratic, 0.0));

    return exploitation + exploration;
}

// ============================================================
// NodeLearner
// ============================================================

void
NodeLearner::Init(uint32_t nodeIdx, uint32_t numGateways)
{
    meshNodeIndex  = nodeIdx;
    currentGateway = 0; // Start with GW1; UCB exploration covers others quickly
    lastChosenArm  = 0;
    for (int f = 0; f < 4; f++)
        lastContext[f] = 0.0;

    arms.resize(numGateways);
    for (uint32_t g = 0; g < numGateways; g++)
        arms[g].Reset(g);
}

// ============================================================
// LinUCBGatewaySelector -- constructor
// ============================================================

LinUCBGatewaySelector::LinUCBGatewaySelector(
    NodeContainer                 gatewayNodes,
    NodeContainer                 meshOnlyNodes,
    NetDeviceContainer            gatewayNrDevs,
    energy::EnergySourceContainer gwEnergySources,
    Ipv4InterfaceContainer        meshIpIfaces,
    Ptr<FlowMonitor>              monitor,
    Ptr<Ipv4FlowClassifier>       classifier,
    std::vector<uint32_t>         sourceNodeIndices,
    uint32_t                      numGateways,
    double                        alpha,
    Time                          decisionInterval)
    : m_gatewayNodes(gatewayNodes),
      m_meshOnlyNodes(meshOnlyNodes),
      m_gatewayNrDevs(gatewayNrDevs),
      m_gwEnergySources(gwEnergySources),
      m_meshIpIfaces(meshIpIfaces),
      m_flowMonitor(monitor),
      m_classifier(classifier),
      m_sourceNodeIndices(sourceNodeIndices),
      m_numGateways(numGateways),
      m_alpha(alpha),
      m_interval(decisionInterval),
      m_firstCall(true),
      m_roundCount(0)
{
    m_latestSinr.assign(numGateways, 1.0);     // 1.0 linear = 0 dB default
    m_fwdPacketDelta.assign(numGateways, 0);
}

// ============================================================
// LinUCBGatewaySelector::Start
// ============================================================

void
LinUCBGatewaySelector::Start(Time firstDecisionTime)
{
    // Initialise one NodeLearner per source node
    for (uint32_t nodeIdx : m_sourceNodeIndices)
    {
        NodeLearner learner;
        learner.Init(nodeIdx, m_numGateways);
        m_learners.push_back(learner);
    }

    // Connect DlDataSinr trace on each gateway's NR UE PHY
    for (uint32_t g = 0; g < m_numGateways; g++)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(m_gatewayNrDevs.Get(g));
        if (ueDev)
        {
            ueDev->GetPhy(0)->TraceConnectWithoutContext(
                "DlDataSinr",
                MakeBoundCallback(&LinUCBGatewaySelector::SinrCallback, this, g));
        }
    }

    Simulator::Schedule(firstDecisionTime,
                        &LinUCBGatewaySelector::MakeDecisions,
                        this);
}

// ============================================================
// LinUCBGatewaySelector::SinrCallback  (static)
// ============================================================

void
LinUCBGatewaySelector::SinrCallback(LinUCBGatewaySelector* self,
                                     uint32_t               gwIndex,
                                     uint16_t               /*cellId*/,
                                     uint16_t               /*rnti*/,
                                     double                 avgSinrLinear,
                                     uint16_t               /*bwpId*/)
{
    if (gwIndex < self->m_latestSinr.size())
        self->m_latestSinr[gwIndex] = avgSinrLinear;
}

// ============================================================
// LinUCBGatewaySelector::ExtractContext
// ============================================================

void
LinUCBGatewaySelector::ExtractContext(uint32_t gwIdx, double context[4]) const
{
    // Feature 0: Queue load (normalised, 0=empty 1=busiest gateway this round)
    uint32_t maxFwd =
        *std::max_element(m_fwdPacketDelta.begin(), m_fwdPacketDelta.end());
    context[0] = (maxFwd > 0) ? (double)m_fwdPacketDelta[gwIdx] / maxFwd : 0.0;

    // Feature 1: Residual energy (normalised, 0=empty 1=full)
    Ptr<energy::BasicEnergySource> src =
        DynamicCast<energy::BasicEnergySource>(m_gwEnergySources.Get(gwIdx));
    if (src)
    {
        double rem  = src->GetRemainingEnergy();
        double init = src->GetInitialEnergy();
        context[1]  = (init > 0.0) ? rem / init : 1.0;
    }
    else
    {
        context[1] = 1.0; // assume full if source not available
    }

    // Feature 2: 5G SINR (normalised, clipped to [0,1] over 0-30 dB range)
    double sinrLinear = m_latestSinr[gwIdx];
    double sinrDb     = 10.0 * std::log10(std::max(sinrLinear, 1e-10));
    context[2]        = std::min(std::max(sinrDb / 30.0, 0.0), 1.0);

    // Feature 3: Bias term (constant 1.0 -- allows linear model to learn intercept)
    context[3] = 1.0;
}

// ============================================================
// LinUCBGatewaySelector::ComputeNodeReward
// ============================================================

double
LinUCBGatewaySelector::ComputeNodeReward(uint32_t                               learnerIdx,
                                          const FlowMonitor::FlowStatsContainer& stats) const
{
    uint32_t     meshNodeIdx = m_learners[learnerIdx].meshNodeIndex;
    Ipv4Address  sourceIp    = m_meshIpIfaces.GetAddress(meshNodeIdx + m_numGateways);

    double   totalDelay   = 0.0;
    uint32_t totalRxDelta = 0;
    uint32_t totalTxDelta = 0;

    for (const auto& flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple tuple = m_classifier->FindFlow(flow.first);
        if (tuple.sourceAddress != sourceIp)
            continue;

        auto prev = m_prevSnapshot.find(flow.first);
        uint32_t prevRx  = (prev != m_prevSnapshot.end()) ? prev->second.rxPackets  : 0;
        uint32_t prevTx  = (prev != m_prevSnapshot.end()) ? prev->second.txPackets  : 0;
        double   prevDel = (prev != m_prevSnapshot.end()) ? prev->second.delaySumMs : 0.0;

        uint32_t rxDelta  = flow.second.rxPackets - prevRx;
        uint32_t txDelta  = flow.second.txPackets - prevTx;
        double   delDelta = static_cast<double>(flow.second.delaySum.GetMilliSeconds()) - prevDel;

        totalRxDelta += rxDelta;
        totalTxDelta += txDelta;
        totalDelay   += delDelta;
    }

    if (totalRxDelta == 0)
        return 0.0;

    double meanDelayMs = totalDelay / static_cast<double>(totalRxDelta);
    double pdr         = (totalTxDelta > 0)
                           ? static_cast<double>(totalRxDelta) / static_cast<double>(totalTxDelta)
                           : 0.0;

    // PDR-weighted inverse latency, scaled to approx [0,1]
    return pdr / (1.0 + meanDelayMs / 100.0);
}

// ============================================================
// LinUCBGatewaySelector::UpdateSnapshot
// ============================================================

void
LinUCBGatewaySelector::UpdateSnapshot(const FlowMonitor::FlowStatsContainer& stats)
{
    for (const auto& flow : stats)
    {
        FlowSnapshot snap;
        snap.rxPackets  = flow.second.rxPackets;
        snap.txPackets  = flow.second.txPackets;
        snap.delaySumMs = static_cast<double>(flow.second.delaySum.GetMilliSeconds());
        m_prevSnapshot[flow.first] = snap;
    }
}

// ============================================================
// LinUCBGatewaySelector::SwitchGateway
// ============================================================

void
LinUCBGatewaySelector::SwitchGateway(uint32_t meshNodeIdx, uint32_t newGwIdx)
{
    Ipv4StaticRoutingHelper routingHelper;
    Ptr<Ipv4> nodeIpv4 = m_meshOnlyNodes.Get(meshNodeIdx)->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> routing = routingHelper.GetStaticRouting(nodeIpv4);

    // Remove existing default routes (iterate backwards to avoid index shift)
    for (int32_t i = static_cast<int32_t>(routing->GetNRoutes()) - 1; i >= 0; i--)
    {
        Ipv4RoutingTableEntry entry = routing->GetRoute(i);
        if (entry.GetDest() == Ipv4Address("0.0.0.0"))
            routing->RemoveRoute(i);
    }

    // Set new default route to the chosen gateway's mesh IP (interface 1 = mesh)
    Ipv4Address gwMeshIp = m_meshIpIfaces.GetAddress(newGwIdx);
    routing->SetDefaultRoute(gwMeshIp, 1);
}

// ============================================================
// LinUCBGatewaySelector::MakeDecisions
// ============================================================

void
LinUCBGatewaySelector::MakeDecisions()
{
    m_roundCount++;
    double nowSec = Simulator::Now().GetSeconds();

    // Fetch current FlowMonitor stats (single CheckForLostPackets call)
    m_flowMonitor->CheckForLostPackets();
    FlowMonitor::FlowStatsContainer currentStats = m_flowMonitor->GetFlowStats();

    // ------------------------------------------------------------------
    // Compute per-gateway forwarding delta (for queue-load context feature)
    // ------------------------------------------------------------------
    std::fill(m_fwdPacketDelta.begin(), m_fwdPacketDelta.end(), 0);
    for (const auto& flow : currentStats)
    {
        Ipv4FlowClassifier::FiveTuple tuple = m_classifier->FindFlow(flow.first);
        // Find which source node owns this flow
        for (uint32_t ni = 0; ni < m_learners.size(); ni++)
        {
            uint32_t    meshNodeIdx = m_learners[ni].meshNodeIndex;
            Ipv4Address nodeIp = m_meshIpIfaces.GetAddress(meshNodeIdx + m_numGateways);
            if (tuple.sourceAddress == nodeIp)
            {
                uint32_t gwIdx   = m_learners[ni].currentGateway;
                uint32_t prevTx  = 0;
                auto     it      = m_prevSnapshot.find(flow.first);
                if (it != m_prevSnapshot.end())
                    prevTx = it->second.txPackets;
                m_fwdPacketDelta[gwIdx] += flow.second.txPackets - prevTx;
                break;
            }
        }
    }

    // ------------------------------------------------------------------
    // Compute rewards for all learners BEFORE updating snapshot
    // ------------------------------------------------------------------
    std::vector<double> rewards(m_learners.size(), 0.0);
    if (!m_firstCall)
    {
        for (uint32_t ni = 0; ni < m_learners.size(); ni++)
            rewards[ni] = ComputeNodeReward(ni, currentStats);
    }

    // ------------------------------------------------------------------
    // Extract gateway contexts (shared -- same for all learners)
    // ------------------------------------------------------------------
    std::vector<std::vector<double>> ctx(m_numGateways, std::vector<double>(4, 0.0));
    for (uint32_t g = 0; g < m_numGateways; g++)
        ExtractContext(g, ctx[g].data());

    // ------------------------------------------------------------------
    // Console header for this round
    // ------------------------------------------------------------------
    std::cout << "[BANDIT] T=" << std::fixed << std::setprecision(1) << nowSec
              << "s | Round " << m_roundCount
              << " | Alpha=" << std::setprecision(2) << m_alpha << "\n";

    uint32_t switchCount = 0;

    // ------------------------------------------------------------------
    // Per-node loop: update arm, compute scores, make decision
    // ------------------------------------------------------------------
    for (uint32_t ni = 0; ni < m_learners.size(); ni++)
    {
        NodeLearner& learner = m_learners[ni];

        // Update last chosen arm with delayed reward
        if (!m_firstCall)
            learner.arms[learner.lastChosenArm].Update(learner.lastContext, rewards[ni]);

        // Compute LinUCB scores for all arms using current context
        double scores[4] = {0.0, 0.0, 0.0, 0.0};
        for (uint32_t g = 0; g < m_numGateways; g++)
            scores[g] = learner.arms[g].ComputeScore(ctx[g].data(), m_alpha);

        // Pick arm with highest score
        uint32_t bestArm = 0;
        for (uint32_t g = 1; g < m_numGateways; g++)
            if (scores[g] > scores[bestArm])
                bestArm = g;

        bool switched = (bestArm != learner.currentGateway);
        if (switched)
        {
            SwitchGateway(learner.meshNodeIndex, bestArm);
            learner.currentGateway = bestArm;
            switchCount++;
        }

        // Persist context+arm for next round's delayed update
        for (int f = 0; f < 4; f++)
            learner.lastContext[f] = ctx[bestArm][f];
        learner.lastChosenArm = bestArm;

        // Console: per-node line
        std::cout << "  Node " << std::setw(2) << learner.meshNodeIndex << " (src):";
        for (uint32_t g = 0; g < m_numGateways; g++)
            std::cout << " GW" << (g + 1) << "=" << std::setw(5) << std::setprecision(2)
                      << scores[g];
        std::cout << " -> GW" << (bestArm + 1)
                  << (switched ? " (switch!)" : " (keep)   ")
                  << " | reward=" << std::setprecision(3) << rewards[ni] << "\n";

        // Log entry
        DecisionRecord rec;
        rec.timeSec       = nowSec;
        rec.roundNum      = m_roundCount;
        rec.sourceNodeIdx = learner.meshNodeIndex;
        rec.chosenGw      = bestArm;
        rec.reward        = rewards[ni];
        rec.ctxLoad       = ctx[bestArm][0];
        rec.ctxEnergy     = ctx[bestArm][1];
        rec.ctxSinr       = ctx[bestArm][2];
        for (uint32_t g = 0; g < m_numGateways; g++)
            rec.scores[g] = scores[g];
        m_log.push_back(rec);
    }

    std::cout << "  Switches this round: " << switchCount << "/" << m_learners.size()
              << " nodes changed gateway\n\n";

    // Update snapshot AFTER all reward computations are done
    UpdateSnapshot(currentStats);
    m_firstCall = false;

    // Schedule next round
    Simulator::Schedule(m_interval, &LinUCBGatewaySelector::MakeDecisions, this);
}

// ============================================================
// LinUCBGatewaySelector::PrintStats
// ============================================================

void
LinUCBGatewaySelector::PrintStats() const
{
    if (m_log.empty())
    {
        std::cout << "[BANDIT] No decisions recorded.\n";
        return;
    }

    // Per-gateway selection totals
    std::vector<uint32_t> gwTotals(m_numGateways, 0);
    uint32_t              totalDecisions = 0;
    for (const auto& rec : m_log)
    {
        gwTotals[rec.chosenGw]++;
        totalDecisions++;
    }

    // Reward averages: early rounds (1-10) vs late (last 10)
    double earlyRewardSum = 0.0;
    uint32_t earlyCount = 0;
    double lateRewardSum = 0.0;
    uint32_t lateCount = 0;

    for (const auto& rec : m_log)
    {
        if (rec.roundNum <= 10)
        {
            earlyRewardSum += rec.reward;
            earlyCount++;
        }
        if (rec.roundNum > m_roundCount - 10)
        {
            lateRewardSum += rec.reward;
            lateCount++;
        }
    }

    double earlyAvg = (earlyCount > 0) ? earlyRewardSum / earlyCount : 0.0;
    double lateAvg  = (lateCount > 0) ? lateRewardSum / lateCount : 0.0;
    double improve  = (earlyAvg > 0) ? (lateAvg - earlyAvg) / earlyAvg * 100.0 : 0.0;

    double simDuration = m_log.back().timeSec;

    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║         LinUCB Bandit Summary (" << std::fixed << std::setprecision(0)
              << simDuration << "s simulation)                 ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Decision rounds: " << std::setw(3) << m_roundCount
              << "  |  Alpha: " << std::setprecision(2) << m_alpha
              << "  |  Learners: " << m_learners.size() << "                  ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Per-gateway selection totals (across all nodes):              ║\n";
    for (uint32_t g = 0; g < m_numGateways; g++)
    {
        double pct = (totalDecisions > 0) ? 100.0 * gwTotals[g] / totalDecisions : 0.0;
        std::cout << "║   GW" << (g + 1) << ": " << std::setw(5) << gwTotals[g]
                  << " (" << std::setw(5) << std::setprecision(1) << pct << "%)             "
                  << "                              ║\n";
    }
    std::cout << "╠════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Mean reward  Rounds 1-10:  " << std::setw(5) << std::setprecision(3)
              << earlyAvg;
    std::cout << "   |   Last 10 rounds: " << std::setw(5) << lateAvg << "         ║\n";
    std::cout << "║ Reward improvement over learning: " << std::setw(6)
              << std::setprecision(1) << improve << "%                        ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

    // Pull count per arm per learner (convergence check)
    std::cout << "Pull counts per learner (rows=nodes, cols=GW1..GW" << m_numGateways << "):\n";
    for (const auto& learner : m_learners)
    {
        std::cout << "  Node " << std::setw(2) << learner.meshNodeIndex << ": ";
        for (uint32_t g = 0; g < m_numGateways; g++)
            std::cout << std::setw(4) << learner.arms[g].pullCount;
        // Mark dominant gateway
        uint32_t dominant = 0;
        for (uint32_t g = 1; g < m_numGateways; g++)
            if (learner.arms[g].pullCount > learner.arms[dominant].pullCount)
                dominant = g;
        std::cout << "  -> GW" << (dominant + 1) << " dominant\n";
    }
    std::cout << "\n";

    // Export CSV
    const std::string csvFile = "bandit-decisions.csv";
    std::ofstream csv(csvFile);
    if (!csv.is_open())
    {
        std::cerr << "[BANDIT] WARNING: could not open " << csvFile << " for writing\n";
        return;
    }

    csv << "round,time_s,node_idx,chosen_gw";
    for (uint32_t g = 0; g < m_numGateways; g++)
        csv << ",score_gw" << (g + 1);
    csv << ",reward,ctx_load,ctx_energy,ctx_sinr\n";

    for (const auto& rec : m_log)
    {
        csv << rec.roundNum << "," << std::fixed << std::setprecision(3) << rec.timeSec
            << "," << rec.sourceNodeIdx << "," << (rec.chosenGw + 1);
        for (uint32_t g = 0; g < m_numGateways; g++)
            csv << "," << std::setprecision(4) << rec.scores[g];
        csv << "," << std::setprecision(4) << rec.reward
            << "," << rec.ctxLoad
            << "," << rec.ctxEnergy
            << "," << rec.ctxSinr << "\n";
    }

    csv.close();
    std::cout << "[BANDIT] Decision log exported to: " << csvFile
              << " (" << m_log.size() << " records)\n\n";
}
