#! /bin/bash
#
# standalone_run.sh
# Copyright (C) 2018 vidushi <vidushi@octa>
#
# Distributed under terms of the MIT license.
#

# make -C ~/Documents/ss-stack/ clean-ss-config clean-ss-scheduler  -j8
# make -C ../../../ss-stack/ ss-config ss-scheduler -j8
make -C ../../../ss-stack/ ss-scheduler -j8
./standalone_sim /home/vidushi/Documents/ss-stack/ss-tools/configs/revel.sbmodel test_standalone.dfg


