# Project layout

- Repository root: Mini Pupper control and search tools.
- `sources/`: upstream projects retained as Git submodules.
- `patches/`: uncommitted local customizations captured from those submodules.
- `experiments/`: small Docker, Gazebo, and simulation experiments.
- `workspaces/`: local ROS workspaces; generated logs and diagnostics are ignored.
- `local/downloads/`: machine-specific installers, ignored by Git.
- `local/archives/`: packaged snapshots, ignored by Git.

Clone the repository and its upstream sources with:

```sh
git clone --recurse-submodules https://github.com/joey12725/minipupper.git
```

For an existing clone, initialize the sources with:

```sh
git submodule update --init --recursive
```
