#! /bin/bash
#
# standalone_run.sh
# Copyright (C) 2018 vidushi <vidushi@octa>
#
# Distributed under terms of the MIT license.
#


make -C ../../../ss-stack/ ss-scheduler -j8
./standalone_sim /home/vidushi/Documents/ss-stack/ss-tools/configs/revel.sbmodel test_standalone.dfg


