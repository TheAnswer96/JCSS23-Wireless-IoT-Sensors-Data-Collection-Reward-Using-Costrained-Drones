# Multiple-Drone Data Collection Maximization Problem (MDMP) Repository

This GitHub repository hosts the code implementation for the research paper titled "Wireless IoT Sensors Data Collection Reward Maximization by Leveraging Multiple Energy- and Storage-Constrained UAVs" recently published at Journal of Computer and System Sciences: an extended version of the original paper "[Optimal and Heuristic Algorithms for Data Collection by Using an Energy- and Storage-Constrained Drone](https://link.springer.com/chapter/10.1007/978-3-031-22050-0_2)" presented at [International Symposium on Algorithms and Experiments for Wireless Sensor Networks (ALGOSENSORS 22)](https://algo-conference.org/2022/algosensors/). 
The research addresses the crucial challenge of efficient data collection from Internet of Things (IoT) sensors deployed within a designated area using energy and storage-constrained drones.

## Abstract

We consider Internet of Things (IoT) sensors deployed inside an area to be monitored.
Drones can be used to collect the data from the sensors, but they are constrained in energy and storage.
Therefore, all drones need to select a subset of sensors whose data are the most relevant to be acquired, modeled by assigning a reward.
We present an optimization problem called Multiple-drone Data-collection Maximization Problem (MDMP) whose objective is to plan a set of drones' missions aimed at maximizing the overall reward from the collected data, and such that each individual drone's mission energy cost and total collected data are within the energy and storage limits, respectively.
We optimally solve MDMP by proposing an Integer Linear Programming based algorithm.
Since MDMP is NP-hard, we devise suboptimal algorithms for single- and multiple-drone scenarios.". Remark that the paper is the extended version of the following paper "Optimal and Heuristic Algorithms for Data Collection by Using an Energy- and Storage-Constrained Drone

## Key Contributions

:white_check_mark: **Definition of the MDMP:** The paper formulates the MDMP, considering the energy and storage limitations of drones while aiming to maximize the collective reward obtained from sampled sensor data.

:white_check_mark: **Integer Linear Programming (ILP) Solution:** An optimal solution approach based on ILP is proposed to solve the MDMP, offering insights into the best possible outcomes.

:white_check_mark: **Suboptimal Algorithms:** Recognizing the NP-hard nature of MDMP, the research presents approximation algorithms and heuristics tailored for both single-drone and multi-drone scenarios, striking a balance between performance and computational complexity.

:white_check_mark: **Algorithm Evaluation:** A comprehensive evaluation of the proposed algorithms is conducted using synthetic data generated through random simulations.

## Repository Contents

- `/main.py`: the script contains the main of our library where we run every function.
- `/ILP.py`: the script contains the Integer Linear Programming formulation of the MDMP based on the well-known CPLEX library.
- `/algorithms.py`: the script contains the definition of every algorithm presented in our journal paper.
- `/inputs.py`: the script contains the function definitions implemented to better manage problem instances, and results.
- `/tests.py`: the script contains all the functions used to perform journal's experiments with tunable parameters, e.g., altitude, deployment area size, and so on.
- `/utilis.py`: the script contains any kind of support function used as subroutine in core functions.
- `/img`: the directory contains the images created for the papers and the conference presentation related to repository.
- `/problems/`: the directory contains the different problem instances genereted for the paper simulations. The directory `/problems/old/` contains the problem instances of the previous conference paper.
- `/results/` : the directory containts the result file of the simulations presented in the journal paper.  The directory `/results/old/` contains the simulation results of the previous conference paper.

## Python Libraries Required
```
numpy
pandas
sympy
networkx
mknapsack
matplotlib
pickle
scipy
docplex
```
:warning: Make sure to have installed correctly a valid software copy of [CPLEX](https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-installing).

## Run Example

To get started with using the codebase and exploring the algorithms proposed in the MDMP paper:

1. Clone this repository: `git clone https://github.com/TheAnswer96/JCSS.git`.
2. Install the required libraries cited into the above section.
3. Uncomment one or more code example written in the `main.py` script.
4. Run the `main.py` script.


## Citation

If you find this work useful in your research, please consider citing the following paper:

```
@article{MDMP2023,
	title = {Wireless IoT Sensors Data Collection Reward Maximization by Leveraging Multiple Energy- and Storage-Constrained UAVs},
	author={Betti Sorbelli, Francesco and Navarra, Alfredo and Palazzetti, Lorenzo and Pinotti, Cristina M. and Prencipe, Giuseppe},
	journal = {Journal of Computer and System Sciences},
	volume = {--},
	pages = {--},
	year = {2023},
	issn = {--},
	doi = {--},
}
```
You can find our paper here:

["Francesco Betti Sorbelli, Alfredo Navarra, Lorenzo Palazzetti, Cristina M. Pinotti, and Giuseppe Prencipe. 'Wireless IoT Sensors Data Collection Reward Maximization by Leveraging Multiple Energy- and Storage-Constrained UAVs.' Journal of Computer and System Sciences, 2023."](https://www.researchgate.net/publication/372992488_Wireless_IoT_Sensors_Data_Collection_Reward_Maximization_by_Leveraging_Multiple_Energy-and_Storage-Constrained_UAVs)

## Contact Us

For any inquiries or feedback regarding the code or the research, please feel free to contact [Lorenzo Palazzetti](lorenzo.palazzetti@unifi.it) or [Francesco Betti Sorbelli](francesco.bettisorbelli@unipg.it).

We hope that the code provided in this repository proves valuable in advancing your understanding of efficient data collection using drones in IoT applications.
