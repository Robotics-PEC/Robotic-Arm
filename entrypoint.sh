#!/bin/bash

# Add sourcing of the setup.bash to .bashrc if it's not already there
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi
if ! grep -q "source ~/workspace/install/setup.bash" ~/.bashrc; then
    echo "source ~/workspace/install/setup.bash" >> ~/.bashrc
fi

# Source the .bashrc to load environment variables and aliases
if [ -f ~/.bashrc ]; then
    source ~/.bashrc
fi

# Execute the passed command or fall back to bash if no command is passed
exec "$@"
