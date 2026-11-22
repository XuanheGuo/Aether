# Iris visual meshes

The 3DR Iris model is credited upstream to Thomas Gubler and Lorenz Meier. These
meshes come from the PX4 SITL Gazebo Classic submodule pinned by PX4 v1.17.0
at `6697ab169ceab512dc706acea63df4c882662c60`. The upstream package declares BSD.

- `iris.stl` is verbatim.
- `iris_prop_cw.obj` and `iris_prop_ccw.obj` are geometry-only conversions of the
  corresponding DAE assets using PyCollada 0.9.3. The DAE files define Z-up and
  meters. The conversion uses scene-bound primitives, `triangleset()`, and
  writes positions and one-indexed triangle faces in that order.
- `scene.py` attaches the assemblies at the source SDF's existing local
  origins. The MuJoCo Viewer supplies render materials. The assets are loaded
  only by the observer process.

[Upstream model and author credit](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/6697ab169ceab512dc706acea63df4c882662c60/models/iris/model.config)
· [Pinned source meshes](https://github.com/PX4/PX4-SITL_gazebo-classic/tree/6697ab169ceab512dc706acea63df4c882662c60/models/iris/meshes)
· [Upstream package license declaration](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/6697ab169ceab512dc706acea63df4c882662c60/package.xml)
