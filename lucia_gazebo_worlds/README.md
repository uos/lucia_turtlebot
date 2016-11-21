How to edit the mesh files
==========================

The .fcstd files can be edited with freecad. After the file has been exported to a .stl, it has to be post-processed in Meshlab:

* Filters - Remeshing, Simplification and Reconstruction - Quadric Edge Collapse Decimation (Target number of faces: 4000)

* Filters - Normals, Curvature and Orientation - Transform: Scale (X Axis 0.001, Uniform Scaling)
