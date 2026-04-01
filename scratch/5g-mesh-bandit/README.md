# 5G-Mesh Bandit: LinUCB Adaptive Gateway Selection

## What This Is

This simulation extends a prior 50-node 5G-WiFi mesh project (in `previous_project/older_sims/5g-mesh-50node.cc`) by replacing static nearest-gateway assignment with **per-node contextual bandit (LinUCB) learning**. Each source node independently learns which gateway to route through based on real-time context: queue load, residual battery energy, and 5G SINR.

The work was developed and validated across three test scenarios. All results are final and the code compiles and runs cleanly.

---

## Results Summary

| Scenario | Static PDR | Bandit PDR | Gain |
|----------|-----------|-----------|------|
| default  | 79.2%     | 83.6%     | +4.4% |
| sinr     | 77.6%     | 81.6%     | +4.0% |
| energy   | 66.4%     | **85.2%** | **+18.8%** |

The energy scenario is the headline result: when GW4 fails at t=50s and GW2 at t=67s, static-assigned nodes drop to 0% PDR. The bandit detects declining rewards and reroutes proactively, maintaining 70–96% PDR across all flows.

---

## File Structure

```
scratch/5g-mesh-bandit/
    README.md                       <- this file
    5g-mesh-bandit.cc               <- main simulation
    linucb-gateway-selector.h       <- LinUCB declarations
    linucb-gateway-selector.cc      <- LinUCB implementation

ns-3.42 root:
    LINUCB_IMPLEMENTATION_PLAN.md   <- original design spec (reference)

previous_project/older_sims/
    5g-mesh-50node.cc               <- baseline paper simulation (untouched)
```

---

## File Descriptions

### `5g-mesh-bandit.cc`

The main ns-3 simulation. Sets up the full topology and runs either static or bandit mode.

**Topology:**
- 8x8 mesh grid (64 nodes), 35m spacing
- 4 asymmetrically placed gateways:
  - GW1: col=2, row=6 → (70m, 210m)
  - GW2: col=5, row=4 → (175m, 140m)
  - GW3: col=2, row=2 → (70m, 70m)
  - GW4: col=5, row=0 → (175m, 0m)
- 12 source nodes at rows {0,1,3,5,7} × cols {1,6} (10 nodes) plus (col=1,row=4) and (col=6,row=6)
- 1 gNB at grid center (122.5m, 122.5m, 10m height)
- 1 MEC server connected to gNB via P2P link

**Traffic:** 3 QoS classes per source node (critical/video/best-effort), 10/30/30 kbps each = 70 kbps/node, 0.84 Mbps total.

**Key parameters:**
```
simTime       = 90s
appStartTime  = 30s    (HWMP needs ~25s to converge in 68-node mesh)
banditAlpha   = 0.3    (low exploration — high alpha causes too many switches → HWMP disruption)
decisionInterval = 1s
```

**Scenarios (`--scenario=`):**
- `default`: All gateways equal. Bandit should match or slightly beat static.
- `energy`: GW4 starts with 300J (fails at t=50s), GW2 starts with 500J (fails at t=67s). Drain modeled via `SimpleDeviceEnergyModel`.
- `sinr`: GW4 moved to (400m, 400m), far from gNB → degraded 5G SINR. Bandit should learn to avoid it.

**`DisableGatewayForwarding` helper** (before `main()`): Called via `Simulator::Schedule` to disable IP forwarding on a gateway at depletion, and simultaneously sets drain current to 0 to prevent `BasicEnergySource` from asserting below 0J.

**NR iface index:** All 4 gateways use iface 1 for NR (NR device installed before mesh device).

**Mesh IP indexing:** Gateways at indices 0..3, mesh-only nodes at indices 4..67 in `meshIpIfaces`.

### `linucb-gateway-selector.h`

Declares all LinUCB types. Key points for future work:
- `energy::EnergySourceContainer` and `energy::BasicEnergySource` are in the `ns3::energy` namespace
- `BasicEnergySourceHelper` is in the `ns3` namespace (no `energy::` prefix)
- `SinrCallback` is static with signature `(LinUCBGatewaySelector*, uint32_t gwIndex, uint16_t cellId, uint16_t rnti, double avgSinrLinear, uint16_t bwpId)`
- Context vector is always 4D: `[queue_load, residual_energy, sinr_normalized, bias=1.0]`

### `linucb-gateway-selector.cc`

Full LinUCB implementation. Key design decisions:

**`Matrix4x4`:** No external dependencies (no Eigen). Cofactor-based 4x4 inverse used as fallback/verification. Sherman-Morrison is the primary inverse update path.

**`UpdateInverse` (Sherman-Morrison):**
```
A_inv_new = A_inv - (A_inv*x * x^T*A_inv) / (1 + x^T * A_inv * x)
```
O(16) update instead of O(64) full inversion. Numerically stable because we start from Identity and do rank-1 updates.

**`MakeDecisions()` call order (important):**
1. Fetch FlowMonitor stats once
2. Compute per-gateway forwarding packet deltas
3. Compute all rewards BEFORE updating snapshot (rewards use delta from last round)
4. Extract contexts for each gateway
5. Per-node: update winning arm with last context+reward, pick new best arm via UCB score, switch gateway if different
6. Call `UpdateSnapshot()` (sets baseline for next round)
7. Schedule next call

**`SwitchGateway()`:** Removes all default routes (dest 0.0.0.0) from a mesh node's static routing table, then adds a new default route pointing to the new gateway's mesh IP via interface 1. HWMP re-discovers paths after the switch (1–2s convergence delay).

**`ExtractContext()`:** Pulls queue load from forwarded-packet delta normalized to [0,1], residual energy from `EnergySource::GetRemainingEnergy()` normalized against initial energy, and SINR from the latest `DlDataSinr` trace callback value.

**`PrintStats()`:** Prints a summary box and exports `bandit-decisions.csv` containing per-round decisions for all 12 nodes. Columns: `time_s, round, node_idx, chosen_gw, score_gw1..4, reward, ctx_load, ctx_energy, ctx_sinr`.

---

## Build and Run

```bash
# from ns-3.42 root

# Build
./ns3 build

# Run all 6 scenario combinations
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --useBandit=false --scenario=default"
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --useBandit=true  --scenario=default"
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --useBandit=false --scenario=energy"
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --useBandit=true  --scenario=energy"
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --useBandit=false --scenario=sinr"
./ns3 run "scratch/5g-mesh-bandit/5g-mesh-bandit --useBandit=true  --scenario=sinr"
```

Output files written to the ns-3.42 root:
- `bandit-decisions.csv` — per-round gateway selection log (bandit mode only)

---

## Known Issues and Tuning Notes

**HWMP convergence:** The 68-node mesh needs ~25s to build routing tables before apps start. `appStartTime=30s` is the safe minimum. Reducing it causes 0% PDR on distant nodes.

**Alpha sensitivity:** `banditAlpha=0.3` is critical. With `alpha=1.5` the bandit performs *worse* than static (71% vs 79%) because excessive exploration triggers too many gateway switches, each causing 1–2s of HWMP path re-discovery during which packets are dropped. Do not increase alpha without also increasing `decisionInterval`.

**`SimpleDeviceEnergyModel` init order:** `SetEnergySource()` must be called before `SetCurrentA()`. `SetCurrentA()` immediately accesses `m_source->GetSupplyVoltage()`. Calling in wrong order causes `NS_ASSERT` crash at startup.

**`BasicEnergySource` lower bound:** If drain is not stopped before energy hits 0J, ns-3 asserts `m_remainingEnergyJ >= energyToDecreaseJ`. The `DisableGatewayForwarding` callback calls `dm->SetCurrentA(0.0)` before disabling IP forwarding to prevent this.

**`decisionInterval=3s`:** Was tried and gave 63% PDR (worse than 1s). Longer intervals mean the bandit holds bad gateway assignments longer after a gateway degrades. 1s is the right value.

---

## What to Do Next

The simulation is complete and validated. Remaining work for the paper:

1. **Plotting script** — `bandit-decisions.csv` contains per-round data for all 12 nodes. A Python/matplotlib script showing gateway selection convergence over time (especially for the energy scenario showing nodes migrating away from GW4 before t=50s) would be a strong figure.

2. **Per-class PDR breakdown** — The current summary aggregates all 3 traffic classes. Breaking out critical vs video vs best-effort PDR would show whether the bandit prioritizes high-value flows.

3. **Paper writing** — The three scenarios map cleanly to three subsections: symmetric baseline, battery failure, SINR degradation. The energy scenario (+18.8%) is the headline number.

---

## Environment

- ns-3.42 with NR module
- Build system: CMake via `./ns3` wrapper
- Working directory for all commands: `/home/rajnish/ns-allinone-3.42/ns-3.42/`
- Build target: `scratch/5g-mesh-bandit/5g-mesh-bandit`
- Last build: 651/651 targets, 0 warnings
