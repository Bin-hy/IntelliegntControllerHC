#!/bin/bash
# Script to install Desktop Shortcut for Intelligent Controller

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_PATH="$WORKSPACE_DIR/scripts/run_app.sh"
ICON_NAME="utilities-terminal" # Default system icon, change if you have a custom icon
APP_NAME="Intelligent Controller"
DESKTOP_FILENAME="intelligent_controller.desktop"

# Ensure run_app.sh is executable
chmod +x "$SCRIPT_PATH"

# Define .desktop file content
generate_desktop_file() {
    cat <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=$APP_NAME
Comment=Launch ROS2 Intelligent Controller System
Exec=$SCRIPT_PATH
Icon=$ICON_NAME
Terminal=true
StartupNotify=true
Categories=Robotics;Science;Development;
Path=$WORKSPACE_DIR
EOF
}

# 1. Install to User Applications (~/.local/share/applications)
APP_DIR="$HOME/.local/share/applications"
mkdir -p "$APP_DIR"
generate_desktop_file > "$APP_DIR/$DESKTOP_FILENAME"
chmod +x "$APP_DIR/$DESKTOP_FILENAME"
echo "Installed shortcut to $APP_DIR/$DESKTOP_FILENAME"

# 2. Install to Desktop (~/Desktop)
DESKTOP_DIR="$HOME/Desktop"
if [ -d "$DESKTOP_DIR" ]; then
    generate_desktop_file > "$DESKTOP_DIR/$DESKTOP_FILENAME"
    chmod +x "$DESKTOP_DIR/$DESKTOP_FILENAME"
    echo "Installed shortcut to $DESKTOP_DIR/$DESKTOP_FILENAME"
else
    echo "Desktop directory not found, skipping Desktop shortcut."
fi

echo "Installation Complete!"
echo "You can now search for '$APP_NAME' in your applications menu or double-click the icon on your Desktop."
