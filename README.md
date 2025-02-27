# Design_Optimization
This repository inclued the MATLAB functions and analysis for the journal paper "Topology and Morphology Design of Spherically Reconfigurable Homogeneous Modular Soft Robots," as [published in Soft Robotics](https://www.liebertpub.com/doi/10.1089/soro.2021.0125). This paper can be cited as follows:

C. Freeman, M. Maynard, and V. Vikas, “Topology and Morphology Design of Spherically Reconfigurable Homogeneous Modular Soft Robots,” Soft Robotics, vol. 10, Jul. 2022, doi: 10.1089/soro.2021.0125.

The arXiv version can be accessed [here](https://arxiv.org/abs/2205.00544). 

The accompanying youtube video describing this process can be found (here)[https://www.youtube.com/watch?v=K-ZRhlJ1r1A].


## Overview
The objective of this repository is to use numerical optimization to deisgn soft modular reconfigurable robots (MSoRos) that are capable of both individual locomotion and reconfiguration of multiple robot modules into a sphere for future rolling locomotion. Due to the nature of transformations between planar and spherical topology, it is impossible to achieve this reconfiguration without distortion. The [main script (Design_Optimization.m)](/Design_Optimization.m) finds the optimal robot design (given a sinusoidal limb shape) to minimize this distortion, facilitating spherical reconfiguration. This optimization process is performed for five different robot module types, based on the Platonic solids. 

## Details

This repository includes a [main script (Design_Optimization.m)](/Design_Optimization.m) and it's accompanying functions and data. The script requires the following files:
* [InverseOrtho.m](/InverseOrtho.m): applies the inverse orthographic projection to the input base sketch, thereby creating a spherical sketch in (lat,long) cooridnates
* [EqAzimuthal.m](/EqAzimuthal.m): applies the equidistant azimuthal projection to the spherical sketch, thereby creating the planar  module sketch
* [CompleteSketch.m](/CompleteSketch.m): appropriately rotates a single limb sketch to create a full module sketch
* [distort_plot.mat](/distort_plan.m):  data file containing:
  * A_sweep (1x101 vector of ampltidue values 0:0.01:1)
  * D_inter (5x101 matrix of intramodular distortion values): row value: Platonic solids ordered by # faces, column value: corresponds to amplitude index
  
and consists of four major sections:
1.  Intermodular Distortion v. Amplitude
2.  Locomotion Cost v. Amplitude
3.  Final Optimization Problem
4.  Plots of Optimal Amplitude Sketches

The outputs are
1. Optimal amplitudes
2. Intermodular distortion v. amplitude plot
3. Locomotion cost v. amplitude plot
4. Objective function v. amplitude plots
5. Optimal amplitude module sketch plots

The locomotion cost is based on the inverse of the maximum "arm" length, which is shown in the picture below for reference:


![arm](/description/arm_fig.PNG)

