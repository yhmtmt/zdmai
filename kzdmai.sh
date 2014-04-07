#!/bin/sh

module="zdmai"
device="zdmai"

/sbin/rmmod $module.ko $* || exit 1

rm -f /dev/${device}0
