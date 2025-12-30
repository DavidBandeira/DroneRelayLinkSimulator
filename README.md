# DroneRelayLinkSimulator

This repository contains MATLAB code used to generate simulations on UAV-based communication relays. It provides a modular and flexible simulation framework for evaluating both backhaul 5G NR links and local Wi-Fi access, including trajectory planning, user distribution, link budget computation, and throughput estimation.

## Main components

- **Drone trajectory generation:**  
  Function `generateDroneTrajectory` generates discrete UAV trajectories based on input parameters such as altitude, step size, total distance, and trajectory type.  
  Auxiliary function `lawnmowerPath` creates a lawnmower-style flight path for area coverage.

- **User distribution:**  
  Function `generateUsersInArea` randomly places ground users within a defined geographical area and assigns heights, creating receiver position structures.

- **Link budget and propagation analysis:**  
  Function `computeLink` computes received power, SNR, Shannon limit, and link availability for given transmitter and receiver positions. Supports different propagation models including Longley-Rice and free-space.

- **Throughput estimation:**  
  Functions `snr2throughput5G` and `snr2throughputWiFi` map SNR values to realistic effective throughput using modulation, coding, and layer parameters.

- **Simulation scripts:**  
  `propagation_study` evaluates path loss, rain attenuation, and frequency-dependent effects.  
  Example scripts illustrate backhaul and local link performance, visualization of Shannon vs. effective throughput, and link availability analysis.




