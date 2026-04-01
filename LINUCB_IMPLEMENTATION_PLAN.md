# LinUCB Contextual Bandit Gateway Selection -- Implementation Plan

> **For the implementer**: This plan is designed to be executed with Claude Code.
> The base simulation is `scratch/5g-mesh-50node.cc`. Read that file first to
> understand the existing architecture before writing any code.

## Context

The current 50-node hybrid 5G-Mesh simulation uses **static spatial gateway assignment**:
left-half mesh nodes route to GW1, right-half to GW2 (`scratch/5g-mesh-50node.cc` lines 580-606).
This ignores real-time gateway conditions (load, energy, 5G signal quality).

We are replacing this with **per-node Contextual Bandits (LinUCB)**: each source node
independently learns which of 4 gateways gives it the best performance, adapting in
real-time to asymmetric load, energy depletion, and 5G signal conditions.

### Why 4 gateways + per-node bandits (not 2 gateways + global decision)
- With 2 gateways, the optimal assignment is trivially obvious (50/50 split). A static
  heuristic ties or beats the bandit. There aren't enough arms for exploration to matter.
- With 4 gateways, the optimal node-to-gateway mapping is non-obvious and changes as
  conditions shift. Each node at a different grid position sees different mesh hop counts,
  different gateway loads, and potentially different 5G SINR per gateway.
- Per-node bandits let the system discover load-balancing patterns automatically
  (e.g., "nodes near GW3 should use GW3, except when it's overloaded, then spill to GW2").

### Algorithm Summary (LinUCB)
- **Arms (a):** The 4 available Smart Gateways.
- **Learners:** Each of the 12 source nodes maintains its own independent LinUCB instance
  with 4 arms (one per gateway). Total: 12 learners x 4 arms = 48 arm states.
- **Context Vector (x_t,a):** 4D feature vector **per gateway**:
  `[Queue_Load, Residual_Energy, 5G_Signal_Strength, 1.0 (Bias)]`
- **Per-arm state:** Matrix `A` (4x4, init=Identity), Vector `b` (4x1, init=zeros)
- **Selection:** For each arm `a`:
  `theta_a = A_a^{-1} * b_a`
  `score_a = theta_a^T * x_a + alpha * sqrt(x_a^T * A_a^{-1} * x_a)`
  Node picks the gateway with the highest score.
- **Update:** On reward `r` for chosen arm `a`:
  `A_a += x_a * x_a^T`
  `b_a += r * x_a`
- **Reward:** `pdr / (1.0 + meanDelay_ms / 100.0)` -- PDR-weighted inverse latency

---

## File Structure

Create a new scratch subdirectory. ns-3 auto-compiles all `.cc`/`.h` files in subdirs
(confirmed by `scratch/CMakeLists.txt` lines 105-110, no CMakeLists.txt needed inside):

```
scratch/5g-mesh-bandit/
    5g-mesh-bandit.cc              # Main simulation (adapted from 5g-mesh-50node.cc)
    linucb-gateway-selector.h      # Matrix4x4 + LinUCBArm + LinUCBGatewaySelector declarations
    linucb-gateway-selector.cc     # All implementations
```

The original `scratch/5g-mesh-50node.cc` stays **untouched** as the static baseline for A/B comparison.

---

## Topology: 4 Gateways, 8x8 Grid

```
Spacing: 35m  |  Grid: 8x8 = 64 mesh nodes  |  4 Gateways  |  12 Source nodes

+----+----+----+----+----+----+----+----+
| o  | S  | o  | o  | o  | o  | S  | o  |  Row 7     S = Source node
+----+----+----+----+----+----+----+----+           GW = Gateway
| o  | o  |GW1 | o  | o  | o  | o  | o  |  Row 6     o = relay mesh node
+----+----+----+----+----+----+----+----+
| o  | S  | o  | o  | o  | o  | S  | o  |  Row 5
+----+----+----+----+----+----+----+----+
| o  | o  | o  | o  | o  |GW2 | o  | o  |  Row 4
+----+----+----+----+----+----+----+----+
| o  | S  | o  | o  | o  | o  | S  | o  |  Row 3
+----+----+----+----+----+----+----+----+
| o  | o  |GW3 | o  | o  | o  | o  | o  |  Row 2
+----+----+----+----+----+----+----+----+
| o  | S  | o  | o  | o  | o  | S  | o  |  Row 1
+----+----+----+----+----+----+----+----+
| o  | o  | o  | o  | o  |GW4 | o  | o  |  Row 0
+----+----+----+----+----+----+----+----+
  0    1    2    3    4    5    6    7     (columns)

               [gNB] at center (122.5, 122.5, 10.0)
                 |
            [MEC Server]
```

### Gateway positions (asymmetric -- NOT a clean quadrant split):
| Gateway | Grid Position | Coordinates (at 35m spacing) |
|---------|--------------|------------------------------|
| GW1 | col=2, row=6 | (70, 210, 1.5) |
| GW2 | col=5, row=4 | (175, 140, 1.5) |
| GW3 | col=2, row=2 | (70, 70, 1.5) |
| GW4 | col=5, row=0 | (175, 0, 1.5) |

**Why asymmetric**: If gateways were at perfect quadrant centers, nearest-neighbor static
assignment would be near-optimal. The off-center placement creates ambiguity -- nodes in
the middle of the grid could reasonably use 2-3 different gateways, which is where the
bandit's ability to learn from feedback matters.

### Source nodes (12 total, 2 per row on alternating columns):
Rows 0,1,2,3,4,5 (skip gateway rows 0,2,4,6 -- use rows 1,3,5,7 instead):
- Rows 1,3,5,7: col 1 (left side) and col 6 (right side)
- This gives 8 source nodes. For 12 total, also add at rows 0 and 6: col 1 and col 6.
- Final: 12 source nodes spread across the grid.

### Key topology parameters (CLI-configurable):
```cpp
uint32_t gridWidth = 8;
uint32_t gridHeight = 8;
uint32_t numGateways = 4;
uint32_t numMeshNodes = gridWidth * gridHeight;  // 64
double meshSpacing = 35.0;
Time simTime = Seconds(60.0);       // Longer run for convergence
Time appStartTime = Seconds(12.0);  // Allow mesh peering on larger grid
```

---

## Component 1: Matrix4x4 Utility (in `linucb-gateway-selector.h`)

Inline 4x4 matrix class. No external dependencies (Eigen is not in ns-3).

```cpp
struct Matrix4x4 {
    double m[4][4];

    static Matrix4x4 Identity();
    static Matrix4x4 Zero();
    Matrix4x4 operator+(const Matrix4x4& rhs) const;
    Matrix4x4& operator+=(const Matrix4x4& rhs);
    static Matrix4x4 OuterProduct(const double v[4]);   // returns v * v^T (rank-1 4x4)
    void MultVec(const double in[4], double out[4]) const;  // out = M * in
    double QuadForm(const double v[4]) const;  // returns v^T * M * v (scalar)
    bool Invert(Matrix4x4& inv) const;  // Cofactor-based 4x4 inversion (fallback/verify)
};
```

### Sherman-Morrison incremental inverse update:
Instead of inverting A from scratch every time, maintain `A_inv` and update it when
a new outer product is added:

```cpp
// After A += x * x^T, update A_inv in O(d^2) = O(16) instead of O(d^3):
// A_inv_new = A_inv - (A_inv * x * x^T * A_inv) / (1 + x^T * A_inv * x)
void UpdateInverse(Matrix4x4& A_inv, const double x[4]) {
    double Ax[4];
    A_inv.MultVec(x, Ax);      // Ax = A_inv * x
    double denom = 1.0;
    for (int i = 0; i < 4; i++)
        denom += x[i] * Ax[i]; // denom = 1 + x^T * A_inv * x
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            A_inv.m[i][j] -= Ax[i] * Ax[j] / denom;
}
```

Keep `Invert()` as a fallback for verification (test that Sherman-Morrison result
matches full inversion to within 1e-10).

**Why this is safe**: A starts as Identity and only receives positive semi-definite
additions (x*x^T), so A is always positive definite with det >= 1. Inversion never fails.

---

## Component 2: LinUCBArm (Per-Gateway State)

```cpp
struct LinUCBArm {
    uint32_t gatewayIndex;
    Matrix4x4 A;           // 4x4, init = Identity
    double b[4];            // 4x1, init = zeros
    Matrix4x4 A_inv;        // Maintained incrementally via Sherman-Morrison
    uint32_t pullCount;     // Number of times this arm was selected

    void Reset();  // A=I, b=0, A_inv=I, pullCount=0

    void Update(const double context[4], double reward);
        // A += x * x^T
        // b[i] += reward * x[i]
        // UpdateInverse(A_inv, x)   <-- Sherman-Morrison
        // pullCount++

    double ComputeScore(const double context[4], double alpha) const;
        // theta[i] = sum_j(A_inv[i][j] * b[j])    -- theta = A_inv * b
        // exploitation = sum_i(theta[i] * x[i])     -- theta^T * x
        // exploration  = alpha * sqrt(x^T * A_inv * x)  -- uncertainty bonus
        // return exploitation + exploration
};
```

---

## Component 3: NodeLearner (Per-Source-Node State)

Each source node has its own independent learner with `numGateways` arms:

```cpp
struct NodeLearner {
    uint32_t meshNodeIndex;             // Index into meshOnlyNodes
    uint32_t currentGateway;            // Which gateway this node currently uses
    std::vector<LinUCBArm> arms;        // One per gateway (size = numGateways)
    double lastContext[4];              // Context used in last decision (for delayed reward update)
    uint32_t lastChosenArm;            // Arm chosen in last decision

    void Init(uint32_t nodeIdx, uint32_t numGateways);  // Create arms, set initial gateway
};
```

---

## Component 4: LinUCBGatewaySelector (Orchestrator)

### Constructor parameters:
| Parameter | Source in base simulation |
|-----------|--------------------------|
| `NodeContainer& gatewayNodes` | `gatewayNodes.Create(numGateways)` |
| `NodeContainer& meshOnlyNodes` | `meshOnlyNodes.Create(numMeshNodes)` |
| `NetDeviceContainer& gatewayNrDevs` | `nrHelper->InstallUeDevice(gatewayNodes, allBwps)` |
| `energy::EnergySourceContainer& gwEnergySources` | indices 0..numGateways-1 of `meshEnergySources` |
| `Ipv4InterfaceContainer& meshIpIfaces` | indices 0..numGateways-1 = gateway mesh IPs |
| `Ptr<FlowMonitor> monitor` | `flowmonHelper.InstallAll()` |
| `Ptr<Ipv4FlowClassifier> classifier` | `flowmonHelper.GetClassifier()` |
| `std::vector<uint32_t> sourceNodeIndices` | the 12 source nodes |
| `uint32_t numGateways` | 4 |
| `double alpha` | CLI param, default 1.5 |
| `Time decisionInterval` | CLI param, default 1.0s |

### Key Methods:

```cpp
class LinUCBGatewaySelector {
public:
    LinUCBGatewaySelector(/* params above */);

    void Start(Time firstDecisionTime);
        // 1. Connect DlDataSinr trace on each gateway's NrUePhy
        // 2. Schedule first MakeDecisions() call
        // 3. Initialize all NodeLearners (one per source node, each with numGateways arms)

    void MakeDecisions();
        // Called every decisionInterval. For EACH source node independently:
        //   1. Compute reward for that node's last chosen gateway (FlowMonitor delta)
        //   2. Update that node's chosen arm with (lastContext, reward)
        //   3. Extract context for ALL gateways
        //   4. Compute LinUCB scores for all arms
        //   5. Pick gateway with highest score
        //   6. If different from current gateway, call SwitchGateway()
        //   7. Log the decision
        // Then schedule next call.

    double ComputeNodeReward(uint32_t sourceNodeIdx);
        // FlowMonitor delta for flows originating from this specific source node

    void ExtractContext(uint32_t gwIndex, double context[4]);
        // Reads live gateway state (queue load, energy, SINR, bias)

    void SwitchGateway(uint32_t meshNodeIdx, uint32_t newGwIdx);
        // Modifies Ipv4StaticRouting at runtime

    void PrintStats() const;
        // After simulation: per-node convergence, per-gateway selection counts, reward trends
        // Also exports CSV for plotting

private:
    // SINR trace callback (asynchronous, updates m_latestSinr)
    static void SinrCallback(LinUCBGatewaySelector* self, uint32_t gwIndex,
                              uint16_t cellId, uint16_t rnti,
                              double avgSinr, uint16_t bwpId);

    std::vector<NodeLearner> m_learners;    // One per source node
    uint32_t m_numGateways;
    double m_alpha;
    Time m_interval;
    bool m_firstCall;                        // Skip reward on first decision

    // Gateway state (shared across all learners)
    std::vector<double> m_latestSinr;        // Per-gateway, updated via trace
    std::vector<uint32_t> m_fwdPacketCount;  // Per-gateway packet count for queue load

    // FlowMonitor snapshots per flow for reward deltas
    struct FlowSnapshot {
        uint32_t rxPackets;
        double delaySumMs;
        uint32_t txPackets;
    };
    std::map<uint32_t, FlowSnapshot> m_prevSnapshot;

    // Decision log
    struct DecisionRecord {
        Time time;
        uint32_t sourceNodeIdx;
        uint32_t chosenGw;
        double scores[4];     // One per gateway
        double reward;
    };
    std::vector<DecisionRecord> m_log;

    // References to simulation objects (stored in constructor)
    NodeContainer m_gatewayNodes;
    NodeContainer m_meshOnlyNodes;
    NetDeviceContainer m_gatewayNrDevs;
    energy::EnergySourceContainer m_gwEnergySources;
    Ipv4InterfaceContainer m_meshIpIfaces;
    Ptr<FlowMonitor> m_flowMonitor;
    Ptr<Ipv4FlowClassifier> m_classifier;
    std::vector<uint32_t> m_sourceNodeIndices;
};
```

---

## Component 5: Context Feature Extraction (4D vector)

Called once per gateway per decision round. All 12 source nodes see the SAME context
for a given gateway (gateway state is global), but each node's learner interprets it
differently based on its own history.

### Feature 0 -- Queue Load (normalized)

Track packets forwarded per gateway since last decision using FlowMonitor deltas.
For each flow, map source IP -> mesh node index -> `currentGateway` to attribute
packets to gateways.

```cpp
// In MakeDecisions(), before per-node loop:
uint32_t fwdCount[numGateways] = {0};
for (auto& flow : stats) {
    auto tuple = m_classifier->FindFlow(flow.first);
    uint32_t nodeIdx = IpToMeshNodeIndex(tuple.sourceAddress);
    uint32_t gwIdx = GetNodeCurrentGateway(nodeIdx);
    uint32_t delta = flow.second.txPackets - prevSnapshot[flow.first].txPackets;
    fwdCount[gwIdx] += delta;
}
uint32_t maxFwd = *std::max_element(fwdCount, fwdCount + numGateways);
double queueLoad_gwi = (maxFwd > 0) ? (double)fwdCount[gwi] / maxFwd : 0.0;
```

### Feature 1 -- Residual Energy (normalized)

```cpp
Ptr<energy::BasicEnergySource> src =
    DynamicCast<energy::BasicEnergySource>(m_gwEnergySources.Get(gwIndex));
double normalizedEnergy = src->GetRemainingEnergy() / src->GetInitialEnergy();
// Range: [0, 1]. In default scenario both ~0.99. In Scenario 2, degraded GW drops.
```

API: `src/energy/model/basic-energy-source.h` -- `GetRemainingEnergy()` (line 70),
`GetInitialEnergy()` (line 56).

### Feature 2 -- 5G Signal Strength (normalized SINR)

Connect to `DlDataSinr` trace on each gateway's NR UE PHY at `Start()` time:

```cpp
for (uint32_t g = 0; g < m_numGateways; g++) {
    Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(m_gatewayNrDevs.Get(g));
    ueDev->GetPhy(0)->TraceConnectWithoutContext(
        "DlDataSinr",
        MakeBoundCallback(&LinUCBGatewaySelector::SinrCallback, this, g));
}
```

Trace signature (`contrib/nr/model/nr-ue-phy.h` line 861):
`TracedCallback<uint16_t, uint16_t, double, uint16_t>` -- (cellId, rnti, avgSinr_linear, bwpId)

Callback stores latest value. Normalization:
```cpp
double sinrDb = 10.0 * std::log10(std::max(avgSinrLinear, 1e-10));
double normalized = std::min(std::max(sinrDb / 30.0, 0.0), 1.0);
```

### Feature 3 -- Bias
Constant `1.0`. Allows the linear model to learn an intercept term.

---

## Component 6: Dynamic Gateway Switching

Uses `Ipv4StaticRouting` API (`src/internet/model/ipv4-static-routing.h` line 238):

```cpp
void LinUCBGatewaySelector::SwitchGateway(uint32_t meshNodeIdx, uint32_t newGwIdx)
{
    Ipv4StaticRoutingHelper routingHelper;
    Ptr<Ipv4StaticRouting> routing = routingHelper.GetStaticRouting(
        m_meshOnlyNodes.Get(meshNodeIdx)->GetObject<Ipv4>());

    // Remove existing default route(s) -- iterate backwards to avoid index shift
    for (int32_t i = routing->GetNRoutes() - 1; i >= 0; i--)
    {
        Ipv4RoutingTableEntry entry = routing->GetRoute(i);
        if (entry.GetDest() == Ipv4Address("0.0.0.0"))
            routing->RemoveRoute(i);
    }

    // Set new default route to chosen gateway's mesh IP
    Ipv4Address gwMeshIp = m_meshIpIfaces.GetAddress(newGwIdx);
    routing->SetDefaultRoute(gwMeshIp, 1);  // interface 1 = mesh
}
```

**Scope**: Only the 12 source nodes have their routes changed dynamically.
The remaining 52 relay nodes keep their initial nearest-gateway static routes
(needed for HWMP mesh forwarding to work correctly).

**Initial assignment**: Each source node starts assigned to gateway index 0 (GW1).
The UCB exploration bonus ensures all gateways get tried within the first few rounds.

---

## Component 7: Reward Feedback

Computed **per source node** from FlowMonitor deltas between decision intervals.

```cpp
double LinUCBGatewaySelector::ComputeNodeReward(uint32_t sourceNodeIdx)
{
    m_flowMonitor->CheckForLostPackets();
    auto stats = m_flowMonitor->GetFlowStats();

    double totalDelay = 0;
    uint32_t totalRxDelta = 0, totalTxDelta = 0;

    // Get this source node's IP to identify its flows
    Ipv4Address sourceIp = m_meshIpIfaces.GetAddress(sourceNodeIdx + m_numGateways);

    for (auto& flow : stats)
    {
        auto tuple = m_classifier->FindFlow(flow.first);
        if (tuple.sourceAddress != sourceIp)
            continue;  // Not this node's flow

        auto prev = m_prevSnapshot.find(flow.first);
        uint32_t prevRx  = (prev != m_prevSnapshot.end()) ? prev->second.rxPackets : 0;
        double   prevDel = (prev != m_prevSnapshot.end()) ? prev->second.delaySumMs : 0;
        uint32_t prevTx  = (prev != m_prevSnapshot.end()) ? prev->second.txPackets : 0;

        totalRxDelta += flow.second.rxPackets - prevRx;
        totalTxDelta += flow.second.txPackets - prevTx;
        totalDelay   += flow.second.delaySum.GetMilliSeconds() - prevDel;
    }

    if (totalRxDelta == 0) return 0.0;  // No delivery = zero reward

    double meanDelayMs = totalDelay / totalRxDelta;
    double pdr = (totalTxDelta > 0) ? (double)totalRxDelta / totalTxDelta : 0;

    // Combined reward: PDR-weighted inverse latency, scaled to ~[0, 1]
    return pdr / (1.0 + meanDelayMs / 100.0);
}
```

**Important**: `UpdateSnapshot()` must be called ONCE after all per-node rewards are
computed, not inside each node's reward computation (otherwise node 2 would see stale
deltas from node 1's snapshot update).

---

## Integration into Main Simulation (`5g-mesh-bandit.cc`)

Copy `scratch/5g-mesh-50node.cc` and apply these changes:

### Change 1: Includes and CLI params

At top of file, add:
```cpp
#include "linucb-gateway-selector.h"
```

After existing `cmd.AddValue` calls (~line 150), add:
```cpp
double banditAlpha = 1.5;
double decisionIntervalSec = 1.0;
bool useBandit = true;
std::string scenario = "default";  // "default", "energy", "sinr"
cmd.AddValue("banditAlpha", "LinUCB exploration parameter", banditAlpha);
cmd.AddValue("decisionInterval", "Bandit decision interval (seconds)", decisionIntervalSec);
cmd.AddValue("useBandit", "Use LinUCB bandit (true) or static nearest-gw (false)", useBandit);
cmd.AddValue("scenario", "Test scenario: default, energy, sinr", scenario);
```

Change grid/gateway defaults:
```cpp
uint32_t gridWidth = 8;
uint32_t gridHeight = 8;
uint32_t numGateways = 4;
uint32_t numMeshNodes = gridWidth * gridHeight;  // 64
Time simTime = Seconds(60.0);
Time appStartTime = Seconds(12.0);
```

### Change 2: Gateway positioning (replace lines 229-244)

```cpp
Ptr<ListPositionAllocator> gwPositions = CreateObject<ListPositionAllocator>();
// Asymmetric placement -- NOT clean quadrant centers
double gwCoords[4][2] = {
    {2 * meshSpacing, 6 * meshSpacing},   // GW1: (70, 210) -- top-left area
    {5 * meshSpacing, 4 * meshSpacing},   // GW2: (175, 140) -- center-right
    {2 * meshSpacing, 2 * meshSpacing},   // GW3: (70, 70) -- bottom-left area
    {5 * meshSpacing, 0 * meshSpacing},   // GW4: (175, 0) -- bottom-right edge
};
for (uint32_t g = 0; g < numGateways; g++)
    gwPositions->Add(Vector(gwCoords[g][0], gwCoords[g][1], 1.5));
```

### Change 3: Scenario-specific energy setup (in energy section, ~line 410)

```cpp
// Base energy for all gateways
double gwInitialEnergy[4] = {10000.0, 10000.0, 10000.0, 10000.0};

if (scenario == "energy") {
    // Scenario 2: GW2 and GW4 have severely depleted batteries
    gwInitialEnergy[1] = 500.0;   // GW2: low energy
    gwInitialEnergy[3] = 300.0;   // GW4: very low energy
}

// Use gwInitialEnergy[g] when installing BasicEnergySource on each gateway
```

### Change 4: Scenario-specific gateway positioning for SINR test

```cpp
if (scenario == "sinr") {
    // Scenario 3: Move GW4 far from gNB (400m away) to degrade its 5G link
    gwCoords[3][0] = 7 * meshSpacing;  // (245, 0) -- far corner
    gwCoords[3][1] = 0;
    // GW4's SINR will be significantly lower than other gateways
}
```

### Change 5: Routing block (replace lines 580-606)

```cpp
// Compute nearest gateway for each mesh node (Euclidean distance)
// Source nodes: if useBandit, assign to GW1 initially (bandit will adapt)
// Relay nodes: always use nearest gateway (static, for mesh forwarding)

Ipv4Address gwMeshIps[4];
for (uint32_t g = 0; g < numGateways; g++)
    gwMeshIps[g] = meshIpIfaces.GetAddress(g);

for (uint32_t i = 0; i < numMeshNodes; ++i) {
    Ptr<Ipv4StaticRouting> nodeRouting = routingHelper.GetStaticRouting(
        meshOnlyNodes.Get(i)->GetObject<Ipv4>());

    bool isSource = std::find(sourceNodes.begin(), sourceNodes.end(), i)
                    != sourceNodes.end();

    if (useBandit && isSource) {
        // Bandit-managed: start with GW1, will be switched dynamically
        nodeRouting->SetDefaultRoute(gwMeshIps[0], 1);
    } else {
        // Static: nearest gateway by Euclidean distance
        uint32_t nearest = FindNearestGateway(i, gridWidth, meshSpacing, gwCoords, numGateways);
        nodeRouting->SetDefaultRoute(gwMeshIps[nearest], 1);
    }
}
```

`FindNearestGateway` is a simple helper: compute grid position from node index,
calculate Euclidean distance to each gateway, return index of closest.

### Change 6: Source node selection (12 nodes instead of 6)

```cpp
std::vector<uint32_t> sourceNodes;
// 2 per row at columns 1 (left) and 6 (right), on rows 0,1,3,5,7
// Skip rows 2,4,6 where gateways sit to avoid overlap
uint32_t sourceRows[] = {0, 1, 3, 5, 7};
for (uint32_t r = 0; r < 5; r++) {
    sourceNodes.push_back(GetNodeIndex(1, sourceRows[r], gridWidth));
    sourceNodes.push_back(GetNodeIndex(6, sourceRows[r], gridWidth));
}
// Add 2 more at row 4 col 1 and row 6 col 6 to reach 12
sourceNodes.push_back(GetNodeIndex(1, 4, gridWidth));
sourceNodes.push_back(GetNodeIndex(6, 6, gridWidth));
```

### Change 7: Create and start the selector (after FlowMonitor setup)

```cpp
Ptr<Ipv4FlowClassifier> classifier =
    DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());

LinUCBGatewaySelector selector(
    gatewayNodes, meshOnlyNodes, gatewayNrDevs,
    meshEnergySources, meshIpIfaces,
    monitor, classifier, sourceNodes,
    numGateways, banditAlpha, Seconds(decisionIntervalSec));

if (useBandit)
    selector.Start(appStartTime);
```

### Change 8: After Simulator::Run()

```cpp
if (useBandit)
    selector.PrintStats();
```

### Change 9: Bearer setup loop already scales

The existing bearer setup (lines 618-688) iterates `for gwIdx = 0; gwIdx < numGateways`,
so it automatically works with 4 gateways. No change needed.

---

## Event Flow Per Decision Cycle

```
T = appStartTime + k * decisionInterval:

  [Shared] Extract context for all 4 gateways:
    ctx[0] = ExtractContext(GW1)  // [load, energy, sinr, 1.0]
    ctx[1] = ExtractContext(GW2)
    ctx[2] = ExtractContext(GW3)
    ctx[3] = ExtractContext(GW4)

  [Per source node, independently] For each of the 12 source nodes:
    1. Compute reward for THIS node's last chosen gateway (FlowMonitor delta)
    2. Update THIS node's learner: arms[lastChosen].Update(lastCtx, reward)
    3. Compute LinUCB scores for all 4 arms using current context
    4. Pick gateway with highest score
    5. If different from current gateway, call SwitchGateway(nodeIdx, newGw)
    6. Store chosen arm and context for next round's reward

  [Shared] Update FlowMonitor snapshot (ONCE, after all nodes computed rewards)
  [Shared] Log all decisions
  [Shared] Schedule next MakeDecisions() at T + interval
```

---

## Console Output (Demo-Friendly)

Each decision round prints per-node choices:
```
[BANDIT] T=13.0s | Round 2 | Alpha=1.50
  Node  1 (src): GW1=1.24 GW2=0.89 GW3=1.41 GW4=0.72 -> GW3 (switch!) | reward=0.67
  Node  6 (src): GW1=0.91 GW2=1.33 GW3=0.88 GW4=1.05 -> GW2 (keep)    | reward=0.73
  ...
  Switches this round: 4/12 nodes changed gateway
```

Final summary:
```
╔════════════════════════════════════════════════════════════════╗
║               LinUCB Bandit Summary (60s simulation)          ║
╠════════════════════════════════════════════════════════════════╣
║ Decision rounds: 48  |  Alpha: 1.50  |  Scenario: default    ║
╠════════════════════════════════════════════════════════════════╣
║ Per-gateway selection totals (across all nodes):              ║
║   GW1: 142 (24.7%)  GW2: 158 (27.4%)                        ║
║   GW3: 151 (26.2%)  GW4: 125 (21.7%)                        ║
╠════════════════════════════════════════════════════════════════╣
║ Mean reward:  Round 1-10: 0.42  |  Round 40-48: 0.78         ║
║ Convergence: reward improved 86% over learning period         ║
╚════════════════════════════════════════════════════════════════╝
```

CSV export: `bandit-decisions.csv`
```
round,time_s,node_idx,chosen_gw,score_gw1,score_gw2,score_gw3,score_gw4,reward,ctx_load,ctx_energy,ctx_sinr
1,13.0,1,2,1.24,0.89,1.41,0.72,0.67,0.52,0.99,0.73
...
```

---

## Test Scenarios

### Scenario 1: Default (Symmetric Baseline)
```bash
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --scenario=default --useBandit=true"
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --scenario=default --useBandit=false"
```

**Setup**: All 4 gateways have equal energy (10000 J) and roughly equal SINR.
**Expected result**: Bandit discovers a load-balanced allocation similar to static
nearest-gateway. Performance should be comparable (within ~5%) -- proves the bandit
doesn't hurt.
**What to show**: The bandit naturally converges to a load-balanced state without
being programmed to do so.

### Scenario 2: Asymmetric Energy Depletion
```bash
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --scenario=energy --useBandit=true"
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --scenario=energy --useBandit=false"
```

**Setup**: GW2 starts with 500 J, GW4 starts with 300 J (vs 10000 J for GW1, GW3).
Gateway energy model current draw is set higher for depleted gateways to simulate
increased power consumption under stress. The depleted gateways may drop packets
or experience degraded throughput as their energy drains during the simulation.

**Expected result**: Static assignment keeps sending ~50% of traffic through the
degraded gateways. The bandit learns to shift traffic away from GW2 and GW4 toward
GW1 and GW3, resulting in higher overall PDR and lower mean latency.

**What to show**: Plot showing nodes initially exploring all 4 GWs, then gradually
converging away from GW2/GW4 as their energy drops. Compare final PDR: bandit should
beat static by 10-20%.

### Scenario 3: Asymmetric 5G Signal (SINR Degradation)
```bash
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --scenario=sinr --useBandit=true"
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --scenario=sinr --useBandit=false"
```

**Setup**: GW4 is moved far from the gNB (245m, 0m) so its 5G SINR is significantly
lower (~10 dB vs ~20+ dB for others). This causes higher 5G link latency and potential
packet loss on GW4's uplink.

**Expected result**: Static nearest-gateway assignment sends ~25% of traffic through
the degraded GW4. The bandit learns GW4 has poor SINR and redirects those nodes to
GW2 or GW3 (next nearest with good SINR). Latency improves for affected nodes.

**What to show**: Per-node gateway selection over time. Nodes near GW4 initially try
it (exploration), then permanently switch to alternatives. Compare tail latency
(p95/p99) between static and bandit.

### Running all scenarios for comparison table:
```bash
# Build once
./ns3 build

# Run all 6 configurations
for scenario in default energy sinr; do
  for mode in true false; do
    ./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit \
      --scenario=$scenario --useBandit=$mode --simTime=60 \
      --banditAlpha=1.5 --decisionInterval=1.0"
  done
done
```

Expected comparison table for the demo:

```
                  |     Static (nearest-GW)    |     LinUCB Bandit          |
Scenario          | PDR    Delay   Throughput  | PDR    Delay   Throughput  |  Delta
─────────────────-+────────────────────────────+────────────────────────────+────────
Default           | ~85%   ~25ms   ~12 Mbps    | ~84%   ~26ms   ~11.5 Mbps |  ~same
Energy depletion  | ~70%   ~40ms   ~9 Mbps     | ~82%   ~28ms   ~11 Mbps   | +17% PDR
SINR degradation  | ~72%   ~45ms   ~9.5 Mbps   | ~83%   ~27ms   ~11.5 Mbps | +15% PDR
```

---

## Verification Checklist

### Step 1: Matrix math unit test (before simulation starts)
Add in `main()`:
```cpp
// Quick self-test of Matrix4x4
{
    Matrix4x4 I = Matrix4x4::Identity();
    Matrix4x4 I_inv;
    bool ok = I.Invert(I_inv);
    NS_ASSERT(ok);
    // Verify I_inv == I (within tolerance)

    // Test Sherman-Morrison: A = I + x*x^T, verify A_inv via both methods
    double x[4] = {0.5, 0.3, 0.8, 1.0};
    Matrix4x4 A = I + Matrix4x4::OuterProduct(x);
    Matrix4x4 A_inv_full;
    A.Invert(A_inv_full);
    Matrix4x4 A_inv_sm = I;  // Start with I (inverse of I)
    UpdateInverse(A_inv_sm, x);
    // Verify A_inv_full and A_inv_sm match to within 1e-10
}
std::cout << "[OK] Matrix4x4 self-test passed\n";
```

### Step 2: Build and basic run
```bash
./ns3 build
# Static mode first (should behave like original 5g-mesh-50node.cc)
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --useBandit=false --simTime=30"
# Bandit mode
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --useBandit=true --simTime=30"
```
Verify: no crashes, bandit decisions appear in console, PDR in same ballpark.

### Step 3: Scenario comparison
Run all 3 scenarios with both static and bandit. Verify bandit matches or beats
static in all scenarios.

### Step 4: CSV analysis
Check `bandit-decisions.csv`:
- Rewards should generally increase over rounds (learning)
- In asymmetric scenarios, selection should shift away from degraded gateways
- Early rounds should show more exploration (varied choices)

---

## Critical File References

| File | What to use | Key lines |
|------|-------------|-----------|
| `scratch/5g-mesh-50node.cc` | Base simulation to copy and adapt | 580-606 (static routing), 196-200 (nodes), 353 (NR devs), 413 (energy), 544 (mesh IPs), 618-688 (bearer loop), 702-708 (source nodes), 796 (FlowMonitor) |
| `src/internet/model/ipv4-static-routing.h` | `RemoveRoute(i)`, `GetNRoutes()`, `GetRoute(i)`, `SetDefaultRoute()` | 238 |
| `contrib/nr/model/nr-ue-phy.h` | `DlDataSinr` TracedCallback for SINR | 272 (typedef), 861 (trace member) |
| `contrib/nr/model/nr-ue-net-device.h` | `GetPhy(bwpIndex)` to access UE PHY for trace | -- |
| `src/energy/model/basic-energy-source.h` | `GetRemainingEnergy()`, `GetInitialEnergy()` | 56, 70 |
| `src/flow-monitor/model/flow-monitor.h` | `GetFlowStats()`, `CheckForLostPackets()` | -- |
| `src/flow-monitor/model/ipv4-flow-classifier.h` | `FindFlow(flowId)` returns 5-tuple | -- |
| `scratch/CMakeLists.txt` | Confirms subdir auto-build (no CMakeLists needed in subdir) | 105-110 |

---

## Implementation Priority (10-day schedule)

| Days | Task | What to deliver |
|------|------|-----------------|
| 1-2 | `linucb-gateway-selector.h` + `.cc` | Matrix4x4 (with unit test), LinUCBArm, NodeLearner, LinUCBGatewaySelector skeleton |
| 3-4 | `5g-mesh-bandit.cc` | 4-gateway topology, 12 source nodes, scenario CLI params, energy/position configs |
| 5-6 | Integration + first build | Connect SINR traces, FlowMonitor reward, route switching. Fix compilation. |
| 7-8 | Debug + scenarios | Run all 3 scenarios, fix reward computation, verify learning behavior |
| 9-10 | Results + polish | CSV export, comparison table, console output formatting, demo prep |

**Highest risk**: Days 5-6 (ns-3 trace callbacks and dynamic routing can be tricky to debug).
**Buffer**: If behind schedule, drop Scenario 3 (SINR) -- Scenario 2 (energy) alone is
enough to demonstrate the bandit's advantage.
