Hi, I'd like to walk you through my Bash Backup System. The goal was to create a robust CLI tool for managing this specific web application directory structure.

I broke the project down into two scripts:

1. The Backup Script (backup.sh): I utilized the date command to generate unique timestamps for every execution. The script targets the src, data, config, and logs folders. I used zip -r to bundle them, and the script checks the exit code of the zip command. If it returns 0, we get a success message; otherwise, it alerts the user that the backup failed.

2. The Restore Script (restore.sh): This script handles the logic of finding the latest file using ls -t piped into head -1. I built in a few safeguards:

First, it checks if a backup file actually exists.

Second, before overwriting anything, it takes a 'safety snapshot' of the current live files.

Finally, it uses unzip -o to restore the application state.

Both scripts focus heavily on User Experience—providing clear echo messages for every step so the user is never guessing if the script is hanging or working."
