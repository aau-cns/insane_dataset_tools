#!/bin/bash

# The command will recursively find all *.bag in subdirectorys and pass it to rqt_bag

find ./ -name '*.bag' -exec rqt_bag {} +
