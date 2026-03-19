#!/bin/bash
TARGET="./install/lhandpro_service/lib/lhandpro_service/lhandpro_service"

if [ -L "$TARGET" ]; then
    REAL_PATH=$(readlink -f "$TARGET")
    echo "Symlink detected. Setting capabilities on real path: $REAL_PATH"
    sudo setcap cap_net_raw,cap_net_admin+ep "$REAL_PATH"
else
    if [ -f "$TARGET" ]; then
        sudo setcap cap_net_raw,cap_net_admin+ep "$TARGET"
    else
        echo "Error: File $TARGET not found."
        exit 1
    fi
fi
