# Bash Backup & Restore System

Two Bash scripts for backing up and restoring the `my-webapp` directory.

## Scripts

### `backup.sh`
- Creates `my-webapp/backups/` if it doesn't exist
- Zips `src/`, `data/`, `config/` and `logs/` into `backup_<YYYY-MM-DD_HH-MM-SS>.zip`
- Checks the result of `zip` and prints whether the backup succeeded

### `restore.sh`
- Finds the **newest** backup (`ls -t ... | head -1`)
- Stops with an error if there is no backup
- Takes a **safety snapshot** of the current live files into `my-webapp/backups/safety/` before overwriting anything
- Restores the backup with `unzip -o`

Every step prints a clear message, so it is always obvious what the script is doing.

## Usage

Run from this folder (`Project_1/`):

```bash
./backup.sh     # create a new backup
./restore.sh    # restore the latest backup
```

`my-webapp/` also contains copies of both scripts that are meant to be run from inside `my-webapp/`.

## Requirements

`bash`, `zip`, `unzip`
