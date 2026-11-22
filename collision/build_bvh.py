"""Compile the triangle intersection helper using VTK from its Python wheel (Linux)."""
from pathlib import Path
import os
import subprocess
import vtkmodules
from vtkmodules.vtkCommonCore import vtkVersion

here=Path(__file__).resolve().parent
vtk_lib=Path(vtkmodules.__file__).resolve().parent
v=vtkVersion()
name=f'vtkCommonDataModel-{v.GetVTKMajorVersion()}.{v.GetVTKMinorVersion()}'
subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-O3','-fPIC','-shared',
                str(here/'bvh.cpp'),f'-L{vtk_lib}',f'-l{name}',f'-Wl,-rpath,{vtk_lib}',
                '-o',str(here/'bvh.so')],check=True)
