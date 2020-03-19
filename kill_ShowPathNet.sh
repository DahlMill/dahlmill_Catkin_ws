ps -ef |grep slamPath | awk '{print $2}' | xargs kill -9

lsof -i:7777
