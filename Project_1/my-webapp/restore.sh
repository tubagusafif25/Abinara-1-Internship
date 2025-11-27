#!/bin/bash

latest_backup=$(ls backups/*.zip | head -1)

if [ -f "$latest_backup" ]; then
    unzip -o $latest_backup
    echo "Restore succeed"
    exit 0
else 
    echo "Restore failed"
    exit 1
fi