#!/usr/bin/env bash

scp admin@10.20.36.2:/home/lvuser/deploy/tuning.json backups/tuning_$(date +%Y_%m_%d:%T).json
