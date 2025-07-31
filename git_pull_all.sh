#!/bin/bash

# Iterate over all subdirectories from the current directory
for dir in $(find ../ -type d -name ".git" | sed 's/\/.git//'); do
    # Navigate to the directory
    echo "Entering directory: $dir"
    cd "$dir" || continue
    
    # Perform git pull with recurse-submodule
    echo "Running 'git pull --recurse-submodule' in $dir"
    git pull --recurse-submodule
    
    # Return to the original directory
    cd - > /dev/null
done

echo "Finished updating all git repositories."
