#!/bin/bash

Timestamps=$(date +"%Y-%m-%d_%H-%M-%S")
backup_dir="backups"
backup_file="$backup_dir/backup_$Timestamps.zip"

mkdir -p "$backup_dir"

echo "Creating backup $backup_file ..."
if zip -r "$backup_file" src/ data/ config/ logs/ > /dev/null; then
    echo "The backup is successful"
    exit 0
else
    echo "The backup failed"
    exit 1
fi
