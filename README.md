# 🧩 ns3-sumo-coupling - Simple Traffic Simulation Setup

[![Download the release page](https://img.shields.io/badge/Download%20Release%20Page-4C8BF5?style=for-the-badge&logo=github&logoColor=white)](https://github.com/guilhermec375/ns3-sumo-coupling/releases)

## 🚀 Getting Started

ns3-sumo-coupling is a small Windows-ready co-simulation tool for ns-3 and SUMO. It lets both tools share traffic data through TraCI so you can run them together in one setup.

Use it if you want to explore network and vehicle traffic together without dealing with a large setup. It is built as a simple example, so it is easier to follow than a full research stack.

## 📥 Download and Run

1. Open the [releases page](https://github.com/guilhermec375/ns3-sumo-coupling/releases).
2. Find the latest release.
3. Download the file for Windows.
4. If the release comes as a ZIP file, extract it first.
5. Open the folder and start the included app or script.

If Windows asks for permission, choose to run the file. If the release includes more than one file, use the main executable or the file named in the release notes.

## 🖥️ What You Need

This setup is meant for a Windows PC with basic support for desktop apps. A typical machine with the following is a good fit:

- Windows 10 or Windows 11
- At least 4 GB of RAM
- Free disk space for the app, SUMO, and test files
- A working internet connection for the first download

For the best result, keep your system up to date and close large apps before you run the simulation.

## 🧭 What This Tool Does

This project links ns-3 and SUMO so they can exchange data while they run.

In plain terms:

- ns-3 handles network traffic
- SUMO handles road traffic
- TraCI passes data between them
- both tools move forward step by step

That means you can test how vehicles and networks affect each other in one run.

## 🛠️ How It Works

The project uses a small amount of code to connect the two simulators.

You do not need to change settings in many places. The release should include a simple way to start the demo and watch both tools work together.

The flow is usually:

1. Start SUMO
2. Start ns-3
3. Let both systems exchange data
4. Watch the traffic and network events move in sync

## 📦 Included Demo Setup

The release is made for a quick first run. It is meant to show:

- a basic vehicle traffic scene
- a network link between the simulators
- message exchange through TraCI
- a simple bidirectional co-simulation flow

This makes it useful as a learning tool and as a base for small tests.

## 🪟 Windows Setup Steps

Follow these steps on Windows:

1. Download the latest release from the [release page](https://github.com/guilhermec375/ns3-sumo-coupling/releases).
2. Save the file to your Downloads folder or Desktop.
3. If the file is compressed, right-click it and choose Extract All.
4. Open the extracted folder.
5. Look for the main file, such as an .exe file or a start script.
6. Double-click it to run.
7. If a security prompt appears, choose More info, then Run anyway if you trust the file.
8. Wait for both simulators to start.
9. Follow any on-screen steps in the console or app window.

If the package includes sample data, keep the folder structure as it was after extraction. Some simulation tools expect files to stay in the same place.

## 🔧 If the App Does Not Start

Try these steps if nothing happens when you open the file:

- Right-click the file and run it as administrator
- Make sure the ZIP file was fully extracted
- Check that the folder name has not changed
- Close other apps and try again
- Download the file again if it looks incomplete

If you see a console window, keep it open. It may show the next step or a path it needs to find.

## 📁 Typical Folder Layout

After extraction, you may see files like these:

- a main start file
- a SUMO config file
- a network script
- a demo route file
- a folder with supporting files

Keep them together. Moving only one file can break the setup.

## 🧪 What You Can Test

This project is useful for simple experiments such as:

- car-to-car messaging
- road traffic and network traffic at the same time
- packet flow between moving nodes
- basic VANET-style studies
- small classroom demos

It is a compact setup, so it works well when you want to learn the basics without a large code base.

## 🔍 Key Terms

A few terms may appear in the app or release notes:

- **ns-3**: a network simulator
- **SUMO**: a road traffic simulator
- **TraCI**: the link used to control SUMO from another tool
- **co-simulation**: two tools running together
- **VANET**: a network made for moving vehicles

You do not need to know these before you start. They are here to help you read the file names and messages you may see.

## 🧷 Troubleshooting Tips

If the release starts but stops early, check these items:

- the extracted folder still contains all files
- SUMO is included or installed where the app expects it
- you used the latest release
- the file path has no special folder moves
- Windows did not block the file

If the demo opens but no traffic appears, start again and wait a few seconds. Some simulation scenes take time to load.

## 📌 Project Scope

This repository is a minimal educational proof of concept. It is meant to show the link between ns-3 and SUMO in a clear way.

It focuses on:

- small code size
- direct control flow
- bidirectional data exchange
- easy study of the setup

That makes it a good choice when you want to understand the idea before building a larger project.

## 📚 When to Use It

Use this project when you want to:

- test a simple ns-3 and SUMO setup
- learn how TraCI connects tools
- run a compact co-simulation demo
- study traffic and network behavior together
- start a VANET experiment with a small base

## 🧩 What You See in the Demo

The demo may show:

- vehicles moving on a road map
- network events tied to vehicle movement
- step-based simulation updates
- data passed back and forth between tools

The main goal is to show that both simulators can stay in sync while they run.

## 🔗 Download Again

If you need the release file again, use the same page here:

[https://github.com/guilhermec375/ns3-sumo-coupling/releases](https://github.com/guilhermec375/ns3-sumo-coupling/releases)

## 🗂️ Repository Topics

This project relates to:

- co-simulation
- coupling
- cpp
- networking
- ns-3
- ns3
- ns3-sumo
- sumo
- traci
- vanet