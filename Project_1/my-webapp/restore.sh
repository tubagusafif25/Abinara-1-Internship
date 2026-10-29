#!/bin/bash

backup_dir="backups"
latest_backup=$(ls -t "$backup_dir"/backup_*.zip 2>/dev/null | head -1)

if [ ! -f "$latest_backup" ]; then
    echo "Restore failed: no backup found in $backup_dir"
    exit 1
fi

echo "Latest backup: $latest_backup"

mkdir -p "$backup_dir/safety"
snapshot="$backup_dir/safety/before_restore_$(date +"%Y-%m-%d_%H-%M-%S").zip"
echo "Saving safety snapshot of current files to $snapshot ..."
if ! zip -r "$snapshot" src/ data/ config/ logs/ > /dev/null; then
    echo "Restore aborted: could not create safety snapshot"
    exit 1
fi

echo "Restoring ..."
if unzip -o "$latest_backup" > /dev/null; then
    echo "Restore succeed"
    exit 0
else
    echo "Restore failed"
    exit 1
fi
