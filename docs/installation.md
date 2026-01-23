# Installation

The installation steps for the Bandu application are straightforward. Follow these steps to set up your environment and run the application.

> Note: Follow commands carefully

## Prerequisites

```bash
# use uv to install deps
uv sync

# source ros
source /opt/ros/jazzy/setup.bash # change jazzy with your ros distro

# start app
uv run chainlit run scripts/app.py --host 0.0.0.0
```

It will start the localhost server at 8000 port. Then you can access the application at `http://localhost:8000` in your web browser.
