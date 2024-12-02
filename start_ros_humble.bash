#!/bin/bash

# Define the path to the storage.conf file
CONFIG_FILE="$HOME/.config/containers/storage.conf"

# Check if the file exists
if [ ! -f "$CONFIG_FILE" ]; then
  # Create the directory if it does not exist
  mkdir -p "$(dirname "$CONFIG_FILE")"

  # Create the file with the specified content
  cat <<EOF > "$CONFIG_FILE"
[storage]
  driver = "vfs"
  rootless_storage_path = "/tmp/dc"
EOF

  echo "File $CONFIG_FILE created to use /tmp/dc"
else
  echo "File $CONFIG_FILE already exists."
fi

echo "Now starting VSCode..."
echo "Accept the 'Dev Containers' extension by clicking Install at bottom right, and then 'Reopen in Container'"
echo "..."
code "$(dirname "$(realpath "$0")")"
