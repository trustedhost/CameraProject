#!/bin/bash

# 설정
CAMERA_DIR="$(pwd)/videos"
REMOTE_USER="yourusername"
REMOTE_HOST="xxx.xxx.xxx.xxx" 
REMOTE_DIR="/video/upload"
LOG_FILE="$(pwd)/upload_log.txt"

# 업로드 후 삭제
for file in "$CAMERA_DIR"/*; do
    if sftp $REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR <<< "put $file"; then
        echo "[$(date)] Uploaded and deleted: $file" >> $LOG_FILE
        rm "$file"  # 파일 삭제
    else
        echo "[$(date)] Failed to upload: $file" >> $LOG_FILE
    fi
done
