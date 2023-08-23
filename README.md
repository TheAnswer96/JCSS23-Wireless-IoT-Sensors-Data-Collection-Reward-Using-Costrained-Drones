# Multiple-Drone Data Collection Maximization Problem (MDMP) Repository

This GitHub repository hosts the code implementation for the research paper titled "Wireless IoT Sensors Data Collection Reward Maximization by Leveraging Multiple Energy- and Storage-Constrained UAVs" recently published at Journal of Computer and System Sciences: an extended version of the original paper "Optimal and Heuristic Algorithms for Data Collection by Using an Energy- and Storage-Constrained Drone" presented at International Symposium on Algorithms and Experiments for Wireless Sensor Networks (ALGOSENSORS 22). 
The research addresses the crucial challenge of efficient data collection from Internet of Things (IoT) sensors deployed within a designated area using energy and storage-constrained drones.

## Abstract

We consider Internet of Things (IoT) sensors deployed inside an area to be monitored.
Drones can be used to collect the data from the sensors, but they are constrained in energy and storage.
Therefore, all drones need to select a subset of sensors whose data are the most relevant to be acquired, modeled by assigning a reward.
We present an optimization problem called Multiple-drone Data-collection Maximization Problem (MDMP) whose objective is to plan a set of drones' missions aimed at maximizing the overall reward from the collected data, and such that each individual drone's mission energy cost and total collected data are within the energy and storage limits, respectively.
We optimally solve MDMP by proposing an Integer Linear Programming based algorithm.
Since MDMP is NP-hard, we devise suboptimal algorithms for single- and multiple-drone scenarios.". Remark that the paper is the extended version of the following paper "Optimal and Heuristic Algorithms for Data Collection by Using an Energy- and Storage-Constrained Drone

## Key Contributions

- Formulation of the MDMP: The paper formulates the MDMP, considering the energy and storage limitations of drones while aiming to maximize the collective reward obtained from sampled sensor data.
- Integer Linear Programming (ILP) Solution: An optimal solution approach based on ILP is proposed to solve the MDMP, offering insights into the best possible outcomes.
- Suboptimal Algorithms: Recognizing the NP-hard nature of MDMP, the research presents approximation algorithms and heuristics tailored for both single-drone and multi-drone scenarios, striking a balance between performance and computational complexity.
- Algorithm Evaluation: A comprehensive evaluation of the proposed algorithms is conducted using synthetic data generated through random simulations.

## Repository Contents



## Getting Started

To get started with using the codebase and exploring the algorithms proposed in the MDMP paper:

1. Clone this repository: `git clone https://github.com/yourusername/mdmp-repository.git`


## Citation

If you find this work useful in your research, please consider citing the following paper:

"Francesco Betti Sorbelli, Alfredo Navarra, Lorenzo Palazzetti, Cristina M. Pinotti, and Giuseppe Prencipe. 'Wireless IoT Sensors Data Collection Reward Maximization by Leveraging Multiple Energy- and Storage-Constrained UAVs.' Journal of Computer and System Sciences, 2023."

## Contact

For any inquiries or feedback regarding the code or the research, please feel free to contact Lorenzo Palazzetti (lorenzo.palazzetti@unifi.it) or Francesco Betti Sorbelli (francesco.bettisorbelli@unipg.it).

We hope that the code provided in this repository proves valuable in advancing your understanding of efficient data collection using drones in IoT applications.
