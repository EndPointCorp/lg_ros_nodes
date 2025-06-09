#!/bin/bash
# Variables: adjust these as necessary.
REMOTE="42-a"
REMOTE_TMP="/tmp"
# Set your docker container name (or ID) here.
DOCKER_CONTAINER="lg-ros"

# Get the list of modified files in the current git repo.
# (This example uses HEAD. You could change this to, e.g., "git status --porcelain" if desired.)
MODIFIED_FILES=$(git diff --name-only HEAD)

# Loop over each modified file.
for file in $MODIFIED_FILES; do
    # Determine if the file matches one of our two patterns.
    scripts_regex="^([^/]+)/scripts/(.*)"
    src_regex="^([^/]+)/src/\1/(.*)"
    if [[ "$file" =~ $scripts_regex ]]; then
        # For files in <directory>/scripts/*
        dir="${BASH_REMATCH[1]}"
        relative_path="${BASH_REMATCH[2]}"
        target="/catkin/install/lib/${dir}/${relative_path}"
    elif [[ "$file" =~  $src_regex ]]; then
        # For files in <directory>/src/<directory>/*
        dir="${BASH_REMATCH[1]}"
        relative_path="${BASH_REMATCH[2]}"
        target="/catkin/install/lib/python3/dist-packages/${dir}/${relative_path}"
    else
        echo "Skipping file (pattern not recognized): $file"
        continue
    fi

    echo "Processing $file -> target: $target"

    # Prepare the remote destination file path.
    remote_file="${REMOTE_TMP}/${file}"

    # Make sure the destination directory exists on the remote system.
    ssh "$REMOTE" "mkdir -p \"$(dirname "$remote_file")\""

    # Copy the file to the remote system.
    scp "$file" "${REMOTE}:${remote_file}"
    if [ $? -ne 0 ]; then
        echo "Error copying $file to ${REMOTE}:${remote_file}"
        continue
    fi

    # On the remote system, copy the file into the Docker container.
    echo "copying ${remote_file} into ${DOCKER_CONTAINER}:${target}"
    ssh "$REMOTE" "docker cp \"${remote_file}\" ${DOCKER_CONTAINER}:\"${target}\""
    if [ $? -ne 0 ]; then
        echo "Error docker copying ${remote_file} to container ${DOCKER_CONTAINER}:${target}"
    fi
done
