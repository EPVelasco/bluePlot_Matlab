# bluePlot_Matlab
This package plots a 3D model of the UGV  ([Blue](https://github.com/AUROVA-LAB/robot_blue)) in MATLAB for controller simulation.

<p align="center">
  <img src="/blue_matlab.png" width="335"  />
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


### Publications
del Pino, I., Munoz-Banon, M. A., Cova-Rocamora, S., Contreras, M. A., Candelas, F. A., & Torres, F. (2020). Deeper in BLUE. Journal of Intelligent & Robotic Systems, 98(1), 207-225.

Citation:
``` 
@article{del2020deeper,
  title={Deeper in BLUE},
  author={del Pino, Ivan and Munoz-Banon, Miguel A and Cova-Rocamora, Saul and Contreras, Miguel A and Candelas, Francisco A and Torres, Fernando},
  journal={Journal of Intelligent \& Robotic Systems},
  volume={98},
  number={1},
  pages={207--225},
  year={2020},
  publisher={Springer}
}
``` 

Muñoz–Bañón, M. Á., del Pino, I., Candelas, F. A., & Torres, F. (2019). Framework for fast experimental testing of autonomous navigation algorithms. Applied Sciences, 9(10), 1997.

Citation:
``` 
@article{munoz2019framework,
  title={Framework for fast experimental testing of autonomous navigation algorithms},
  author={Mu{\~n}oz--Ba{\~n}{\'o}n, Miguel {\'A} and del Pino, Iv{\'a}n and Candelas, Francisco A and Torres, Fernando},
  journal={Applied Sciences},
  volume={9},
  number={10},
  pages={1997},
  year={2019},
  publisher={Multidisciplinary Digital Publishing Institute}
}
```

## ✨ License

This work is licensed under a [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-nc-sa/4.0) and is intended for non-commercial academic use. If you are interested in using the dataset for commercial purposes please contact us via [email](mailto:edison.velasco@ua.es).


