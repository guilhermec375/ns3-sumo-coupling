# ns3-sumo-coupling
A minimal ns-3 contrib module for bidirectional SUMO co-simulation via TraCI. 

Compatible with ns-3.47 and SUMO 1.26. Requires no SUMO source dependencies. 

Please note that this module is intended as a proof-of-concept and educational toy project, rather than a production-ready solution. 

The entire integration is achieved with a remarkably small codebase, making it highly accessible for study and experimentation.

## Architecture Overview
Please note that this module is intended as a proof-of-concept and educational toy project, rather than a production-ready solution.

This module establishes a deterministic, leader-follower synchronization between the ns-3 network layer and the SUMO mobility layer. ns-3 acts as the TraCI client, dictating the simulation time steps.

* **Synchronization:** ns-3 commands SUMO to advance by `dt`, ensuring time alignment between both simulators.
* **Static Node Pool:** Since ns-3 requires nodes to be instantiated prior to simulation, the module utilizes a pre-allocated pool of ns-3 nodes. These are dynamically mapped to active SUMO vehicles. Inactive nodes are parked at out-of-bounds coordinates.
* **Automated Mobility Updates:** SUMO vehicle positions and speeds are retrieved at each step and applied to the corresponding ns-3 `MobilityModel`. This natively supports standard ns-3 radio propagation models without additional configuration.
* **Bidirectional Control:** User-defined callbacks allow ns-3 applications to issue commands back to SUMO, such as dynamic speed alterations based on network events.

## Installation

Clone the repository into the `contrib/` directory of your ns-3 installation and reconfigure the build system.

```
cd <ns3-root>/contrib
git clone https://github.com/Balzakrez/ns3-sumo-coupling
cd ..
./ns3 configure
./ns3 build
```

## Quick Start

```cpp
// Initialize a node pool sized for the peak number of simultaneous vehicles
NodeContainer pool;
pool.Create(20);
MobilityHelper mob;
mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
mob.Install(pool);

// Configure the SumoManager application
auto sumoManager = CreateObject<SumoManager>();
sumoManager->SetAttribute("Host", StringValue("127.0.0.1"));
sumoManager->SetAttribute("Port", IntegerValue(1337));
sumoManager->SetAttribute("StepSize", DoubleValue(0.1));
sumoManager->SetNodePool(pool);

// Define a bidirectional callback (optional)
sumoManager->SetVehicleCallback(
    [&](double t, const std::string& vid, const VehicleState& s) {
        if (s.speed > 10.0) {
            sumoManager->GetClient().SetVehicleSpeed(vid, 5.0);
        }
    });

// Attach to a host node and schedule
auto host = CreateObject<Node>();
host->AddApplication(sumoManager);
sumoManager->SetStartTime(Seconds(0.0));
sumoManager->SetStopTime(Seconds(60.0));

Simulator::Run();
Simulator::Destroy();
```

## Validation Example

The included `simple-vanet` scenario validates position updates, coordinate bounds, average speed, and bidirectional commands.

**Option A: Manual SUMO Execution**
```bash
# Terminal 1: Start SUMO
sumo -c contrib/sumo-coupling/sumo-scenarios/simple/simple.sumocfg --remote-port 1337 --no-step-log

# Terminal 2: Run ns-3
./ns3 build sumo-coupling && NS_LOG=SumoManager=info ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300
```

**Option B: Automated Execution**
```bash
./ns3 build sumo-coupling && NS_LOG=SumoManager=info ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=300 --simTime=60 --sumoConfig=contrib/sumo-coupling/sumo-scenarios/simple/simple.sumocfg
```

## Limitations

* **Polling Overhead:** TraCI subscriptions are currently unimplemented. Position and speed are polled back-to-back sequentially per step.
* **Single Client:** `CMD_SETORDER` is unsupported.
* **2D Coordinate System:** The Z coordinate is fixed to 0.

## License

MIT License
