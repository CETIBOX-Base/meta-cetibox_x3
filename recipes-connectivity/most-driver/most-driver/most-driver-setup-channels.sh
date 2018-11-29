#!/bin/sh

sys=/sys

devpath_control_rx=/devices/virtual/most/mostcore/devices/mdev0/ep8f
devpath_control_tx=/devices/virtual/most/mostcore/devices/mdev0/ep0f

devpath_async_rx=/devices/virtual/most/mostcore/devices/mdev1/ep8e
devpath_async_tx=/devices/virtual/most/mostcore/devices/mdev1/ep0e

/lib/udev/most-setup "$sys$devpath_control_rx" control rx
/lib/udev/most-setup "$sys$devpath_control_tx" control tx
/lib/udev/most-setup "$sys$devpath_async_rx" async rx
/lib/udev/most-setup "$sys$devpath_async_tx" async tx
