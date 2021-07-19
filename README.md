# bluePlot_Matlab
This package plots a 3D model of the UGV  ([Blue](https://github.com/AUROVA-LAB/robot_blue)) in MATLAB for controller simulation.

<p align="center">
  <img src="/blue_3d.png" width="325"  />
  <a href="https://github.com/AUROVA-LAB/robot_blue">
    <img src="/blue.png" width="425" /> 
  <a>
</p>

## Files
- **SL_BLUE** : BLUE model in stl format.
- **bluePlot.m** : Function to plot model.
- **blueSTL.mat**: Strcuts (faces and vertices of STL files).
- **save_stl.m**: Read STL files and convert them to structures.
- [**stlRead.m**](https://es.mathworks.com/matlabcentral/fileexchange/22409-stl-file-reader) : Required function to read STL files.
- **test.m**: Model plot test.

## Test
The following figure represents the results of the test.m code.

<p align="center">
  <img src="/test_plot.png" width="600"  />
</p>

## Test Control Simulation
The following video represents the results of the control_simulation.m code with the kinematic model forwards and backwards.

https://user-images.githubusercontent.com/35384396/126191936-b079298f-cf7c-44bc-92bd-4b975d47da5f.mp4
