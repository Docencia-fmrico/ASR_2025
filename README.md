# ASR_2025

[![main](https://github.com/Docencia-fmrico/ASR_2025/actions/workflows/main.yaml/badge.svg)](https://github.com/Docencia-fmrico/ASR_2025/actions/workflows/main.yaml)

## Build this package

1. Clone this repo in your workspace
```
mkdir -p asr_ws/src
cd asr_ws/src
git clone https://github.com/Docencia-fmrico/ASR_2025.git
```

2. Install third party packages and dependencies

```
cd asr_ws/src
vcs-import . < ASR_2025/thirdparty.repos
cd ..
rosdep install --from-paths src --ignore-src -r
```

3. Build the workspace

```
cd asr_ws/src
colcon build --symlink-install
```

for installint the kobuki driver and simulator, go to https://github.com/IntelligentRoboticsLabs/kobuki