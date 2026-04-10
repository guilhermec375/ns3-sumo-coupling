# ns3-sumo-coupling

A radically minimalist, zero-dependency `ns-3` contrib module for bidirectional co-simulation with SUMO via TraCI.

This project focuses on one goal: make ns-3/SUMO coupling easy to understand, easy to run, and easy to extend. The codebase stays intentionally compact, avoids middleware layers, and does not depend on SUMO source internals.

**Compatibility:** The module is broadly compatible with older versions of both simulators (such as SUMO 1.8 and earlier ns-3 releases). It has been thoroughly tested and validated on **ns-3.47** and **SUMO 1.26**.


## Highlights

* Lean architecture: direct TraCI client with deterministic leader-follower stepping.
* No SUMO source dependency: simple setup and lower maintenance burden.
* Practical examples included: `minimal-vanet` for smoke tests, `simple-vanet` for validation.
* Bidirectional integration: read state from SUMO and issue control commands back.

## 30-Second Start

If you already have `ns-3` and SUMO installed, this is the fastest path:

```bash
# Terminal 1
sumo -c contrib/ns3-sumo-coupling/sumo-scenarios/simple/simple.sumocfg --remote-port 1337 --no-step-log

# Terminal 2 (from ns-3 root)
./ns3 build sumo-coupling
./ns3 run minimal-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=20
```

This runs the minimal coupling example with compact output and periodic active-vehicle counters.

If this is your first run, prefer `minimal-vanet` first, then move to `simple-vanet`.

## Architecture Overview

`ns-3` acts as a lightweight TraCI client, establishing a deterministic, leader-follower synchronization with SUMO.

* **Strict Synchronization:** `ns-3` commands SUMO to advance by a fixed time step (`dt`).
* **Static Node Pool:** A pre-allocated pool of `ns-3` nodes is dynamically mapped to active SUMO vehicles. Inactive nodes are parked out-of-bounds to prevent radio interference.
* **Automated Mobility:** Vehicle states are retrieved at each step and applied to the corresponding `ns-3` `MobilityModel`, seamlessly integrating with standard radio propagation models.
* **Bidirectional Control:** User-defined callbacks enable `ns-3` applications to issue commands back to SUMO with minimal overhead.


## Vehicle API

All vehicle functions are accessible via `sumoManager->GetClient()` inside the callback.

### GET — State Retrieval

| Function | Returns | TraCI var |
|---|---|---|
| `GetVehicleState(vid)` | Full `VehicleState` struct (batched) | — |
| `GetVehicleAngle(vid)` | `double` — heading in degrees (0=North, clockwise) | `0x43` |
| `GetVehicleAcceleration(vid)` | `double` — longitudinal acceleration in m/s² | `0x72` |
| `GetVehicleRoadId(vid)` | `string` — current edge ID | `0x50` |
| `GetVehicleLaneId(vid)` | `string` — current lane ID | `0x51` |
| `GetVehicleLanePosition(vid)` | `double` — distance from lane start in meters | `0x56` |

`GetVehicleState()` retrieves all fields in a single pipelined request, minimising TCP round-trips. Use individual getters only when a single field is needed.

The `VehicleState` struct contains:
```cpp
struct VehicleState {
    double x, y;           // position in meters
    double speed;          // m/s
    double angle;          // degrees
    double acceleration;   // m/s²
    double lanePosition;   // meters from lane start
    std::string roadId;
    std::string laneId;
};
```

### SET — Vehicle Commands

| Function | Effect | TraCI var |
|---|---|---|
| `SetVehicleSpeed(vid, speed)` | Instant speed override. Pass `-1.0` to restore autonomous control. | `0x40` |
| `SetVehicleSlowDown(vid, speed, duration)` | Linearly reduces speed to `speed` m/s over `duration` seconds. | `0x14` |
| `SetVehicleColor(vid, color)` | Changes vehicle colour in the SUMO GUI (`TraCIColor{r,g,b,a}`). | `0x45` |


## Installation

### Prerequisites

* `ns-3` with CMake workflow (`./ns3` helper script available)
* SUMO available in `PATH` (`sumo` or `sumo-gui`)
* This repository cloned into `contrib/ns3-sumo-coupling`

Clone the repository into the `contrib/` directory of your `ns-3` installation and reconfigure the build system:

```bash
cd <ns3-root>/contrib
git clone https://github.com/Balzakrez/ns3-sumo-coupling
cd ..
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

## Quick Start

Below is a minimal example demonstrating how to initialize the coupling within an `ns-3` simulation script.

```cpp
// 1. Initialize a node pool sized for the peak number of simultaneous vehicles
NodeContainer pool;
pool.Create(20);

MobilityHelper mob;
mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
mob.Install(pool);

// 2. Configure the SumoManager application
auto sumoManager = CreateObject<SumoManager>();
sumoManager->SetAttribute("Host", StringValue("127.0.0.1"));
sumoManager->SetAttribute("Port", IntegerValue(1337));
sumoManager->SetAttribute("StepSize", DoubleValue(0.1));
sumoManager->SetNodePool(pool);

// 3. Define a bidirectional callback (optional)
sumoManager->SetVehicleCallback(
    [&](double t, const std::string& vid, const VehicleState& s) {
        // Example: limit vehicle speed if it exceeds 10 m/s
        if (s.speed > 10.0) {
            sumoManager->GetClient().SetVehicleSpeed(vid, 5.0);
        }
    });

// 4. Attach to a host node and schedule
auto host = CreateObject<Node>();
host->AddApplication(sumoManager);
sumoManager->SetStartTime(Seconds(0.0));
sumoManager->SetStopTime(Seconds(60.0));

Simulator::Run();
Simulator::Destroy();
```

For an even smaller runnable example, see [contrib/ns3-sumo-coupling/examples/minimal-vanet.cc](contrib/ns3-sumo-coupling/examples/minimal-vanet.cc).

## Examples

The repository includes:

* `minimal-vanet` ([contrib/ns3-sumo-coupling/examples/minimal-vanet.cc](contrib/ns3-sumo-coupling/examples/minimal-vanet.cc)): ultra-minimal coupling example with compact runtime output (one line per vehicle per second, plus active-vehicle counters at start/every 10 s/end).
* `simple-vanet` ([contrib/ns3-sumo-coupling/examples/simple-vanet.cc](contrib/ns3-sumo-coupling/examples/simple-vanet.cc)): full validation scenario (position updates, coordinate bounds, average speed, and bidirectional commands).

### Which example should I run?

| Example | Best for | Runtime output | Typical duration |
|---|---|---|---|
| `minimal-vanet` | First smoke test and quick integration checks | Very compact + periodic active-vehicle counters | 20 s |
| `simple-vanet` | Functional validation and API behavior checks | Detailed event/report output + active-vehicle counters (start/10s/final) | 300 s |

`simple-vanet` reports command tests (`SetVehicleSpeed`, `SetVehicleSlowDown`, `SetVehicleColor`) as `OK` only when the corresponding TraCI command succeeds.

### What success looks like

Use this quick checklist after running:

* `minimal-vanet`: you see telemetry lines (`[t=...] veh...`) and counter lines (`### T=... ACTIVE VEHICLES=... ###`).
* `minimal-vanet`: final counter can be `0` even before `simTime` if scenario traffic ends early.
* `simple-vanet`: final report shows most or all tests as `OK` and prints `Result: X/8`.

### Minimal example output format

`minimal-vanet` prints two kinds of lines:

* Active vehicle counters: `### T=0.0s INITIAL: ACTIVE VEHICLES=<N> ###`, `### T=10.0s PERIODIC: ACTIVE VEHICLES=<N> ###`, `### T=<simTime>s FINAL: ACTIVE VEHICLES=<N> ###`
* Vehicle telemetry (max one line per second for each vehicle): `[t=12.1s] veh0 x=198.4 y=273.2 v=13.9`

When scenario traffic ends before `simTime`, telemetry lines stop, while periodic/final counters still print and can reach `0`.

Example snippet:

```text
### T=0.0s INITIAL: ACTIVE VEHICLES=0 ###
[t=0.1s] veh0 x=198.4 y=394.9 v=0.0
[t=1.1s] veh0 x=198.4 y=393.6 v=2.3
### T=10.0s PERIODIC: ACTIVE VEHICLES=1 ###
[t=32.0s] veh1 x=201.6 y=351.7 v=13.0
### T=60.0s FINAL: ACTIVE VEHICLES=0 ###
```

### Option A: Manual SUMO Execution

**Terminal 1 (Start SUMO):**
```bash
sumo -c contrib/ns3-sumo-coupling/sumo-scenarios/simple/simple.sumocfg --remote-port 1337 --no-step-log

# If you prefer the graphical interface, use `sumo-gui` instead:
sumo-gui -c contrib/ns3-sumo-coupling/sumo-scenarios/simple/simple.sumocfg --remote-port 1337 --no-step-log
```

**Terminal 2 (Run ns-3):**
```bash
./ns3 build sumo-coupling && ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300

# Minimal example:
./ns3 build sumo-coupling && ./ns3 run minimal-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=20

# If you want to enable detailed logging for debugging, use:
./ns3 build sumo-coupling && NS_LOG="SumoManager=info:SimpleVanet=info" ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300
```

### Option B: Automated Execution

```bash
# Full validation example (single command):
./ns3 build sumo-coupling && ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300 --sumoConfig=contrib/ns3-sumo-coupling/sumo-scenarios/simple/simple.sumocfg

# Minimal example (with explicit SUMO config):
./ns3 build sumo-coupling && ./ns3 run minimal-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=20 --sumoConfig=contrib/ns3-sumo-coupling/sumo-scenarios/simple/simple.sumocfg

# Full validation example with NS_LOG enabled:
./ns3 build sumo-coupling && NS_LOG="SumoManager=info:SimpleVanet=info" ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300 --sumoConfig=contrib/ns3-sumo-coupling/sumo-scenarios/simple/simple.sumocfg
```

## Troubleshooting

* **Connection refused on port 1337:** Ensure SUMO is running first with `--remote-port 1337`.
* **No vehicle updates in ns-3 output:** Check that the `.sumocfg` scenario contains active traffic and simulation time is long enough.
* **`minimal-vanet` not found:** Re-run `./ns3 configure --enable-examples --enable-tests` then `./ns3 build`.

## Limitations

* **Polling Overhead:** TraCI subscriptions are currently unimplemented.
* **Single Client:** The `CMD_SETORDER` TraCI command is currently unsupported.
* **Additional features and TraCI commands are planned for future releases.**

## License

This project is distributed under the MIT License.
