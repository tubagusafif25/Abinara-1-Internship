#!/bin/bash

Timestamps=$(date +"%Y-%m-%d_%H-%M-%S")
backup_file="backups/backup_$Timestamps.zip"

zip -r "$backup_file" src/ data/ config/ logs/

mkdir -p backups

if [ -f "$backup_file" ]; then
    echo "The backup is successful"
    exit 0
else
    echo "The backup failed"
    exit 1
fi
