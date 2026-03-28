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

## Installation

Clone the repository into the `contrib/` directory of your `ns-3` installation and reconfigure the build system:

```bash
cd <ns3-root>/contrib
git clone [https://github.com/Balzakrez/ns3-sumo-coupling](https://github.com/Balzakrez/ns3-sumo-coupling)
cd ..
./ns3 configure
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
sumo -c contrib/sumo-coupling/sumo-scenarios/simple/simple.sumocfg --remote-port 1337 --no-step-log
```

**Terminal 2 (Run ns-3):**
```bash
./ns3 build sumo-coupling && NS_LOG=SumoManager=info ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300
```

### Option B: Automated Execution

```bash
./ns3 build sumo-coupling && NS_LOG=SumoManager=info ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300 --sumoConfig=contrib/sumo-coupling/sumo-scenarios/simple/simple.sumocfg
```

## Limitations

* **Polling Overhead:** TraCI subscriptions are currently unimplemented. Position and speed are polled sequentially per step for each vehicle.
* **Single Client:** The `CMD_SETORDER` TraCI command is currently unsupported.

## License

This project is distributed under the MIT License.
