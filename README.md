# ns3-sumo-coupling

A radically minimalist, zero-dependency `ns-3` contrib module for bidirectional co-simulation with SUMO via TraCI. 

Designed with simplicity at its core, this project demonstrates that robust coupling can be achieved with a remarkably small codebase. It eschews complex middleware and avoids any SUMO source dependencies, making it highly accessible for study, education, and rapid experimentation.

**Compatibility:** The module is broadly compatible with older versions of both simulators (such as SUMO 1.8 and earlier ns-3 releases). It has been thoroughly tested and validated on **ns-3.47** and **SUMO 1.26**.

*Disclaimer: This module is intended as an educational toy project and a foundation for study, rather than a production-ready solution.*

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

## Validation Example

The repository includes a `simple-vanet` scenario that validates position updates, coordinate bounding, average speed measurements, and bidirectional commands.

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

# If you want to enable detailed logging for debugging, use:
./ns3 build sumo-coupling && NS_LOG="SumoManager=info:SimpleVanet=info" ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300
```

### Option B: Automated Execution

```bash
./ns3 build sumo-coupling && ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300 --sumoConfig=contrib/ns3-sumo-coupling/sumo-scenarios/simple/simple.sumocfg

# If you want to run the entire scenario with a single command and see detailed logs, use:
./ns3 build sumo-coupling && NS_LOG="SumoManager=info:SimpleVanet=info" ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300 --sumoConfig=contrib/ns3-sumo-coupling/sumo-scenarios/simple/simple.sumocfg
```

## Limitations

* **Polling Overhead:** TraCI subscriptions are currently unimplemented.
* **Single Client:** The `CMD_SETORDER` TraCI command is currently unsupported.
* **Additional features and TraCI commands are planned for future releases.**

## License

This project is distributed under the MIT License.
