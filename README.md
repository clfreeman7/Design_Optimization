# Design Optimization

This repository includes a [main script (Design_Optimization.m)](/Design_Optimization.m) and it's accompanying functions and data. The script requires the following files:
* InverseOrtho.m: applies the inverse orthographic projection to the input base sketch, thereby creating a spherical sketch in (lat,long) cooridnates
* EqAzimuthal.m: applies the equidistant azimuthal projection to the spherical sketch, thereby creating the planar  module sketch
* CompleteSketch.m: appropriately rotates a single limb sketch to create a full module sketch
* distort_plot.mat:  data file containing:
  * A_sweep (1x101 vector of ampltidue values 0:0.01:1)
  * D_inter (5x101 matrix of intramodular distortion values): row value: Platonic solids ordered by # faces, column value: corresponds to amplitude index
![arm](/description/arm_fig.PNG)

# Design Exploration
 Coming soon.
