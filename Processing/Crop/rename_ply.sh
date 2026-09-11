#!/bin/bash

# Target directory to search (defaults to current directory if not provided)
TARGET_DIR="${1:-.}"

# Find files matching the specific pattern and rename them
find "$TARGET_DIR" -type f -name "*_*_r*_a*_cropped.ply" -print0 | while IFS= read -r -d $'\0' file; do
    # Strip '_cropped.ply' from the end of the filepath and append '.ply'
    new_file="${file%_cropped.ply}.ply"
    
    # Rename the file
    mv "$file" "$new_file"
    
    # Output the action to the console
    echo "Renamed: $(basename "$file") -> $(basename "$new_file")"
done

echo "Done."
