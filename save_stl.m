close all;clear;clc

%% Load a STL files
base = stlRead('STL_BLUE/base.stl'); 
eje_trasero = stlRead('STL_BLUE/eje_trasero.stl'); 
r_trasera = stlRead('STL_BLUE/rueda_trasera.stl'); 
velodyne = stlRead('STL_BLUE/velodyne.stl'); 
r_delantera_d = stlRead('STL_BLUE/rueda_d.stl'); 
r_delantera_i = stlRead('STL_BLUE/rueda_i.stl'); 
eje_delantero = stlRead('STL_BLUE/eje_delantero.stl'); 
aro_delantero = stlRead('STL_BLUE/aro_delantero.stl');

%% Save strcuts (faces and vertices of STL files)
save('blueSTL.mat');
