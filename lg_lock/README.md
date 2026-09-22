lg\_lock
-------

Locks a Liquid Galaxy so it ignores input until it is unlocked with a
password.

## Nodes

### service.py

#### Parameters

* `~password` [string] - password that unlocks the system. Read with no default; the node logs an error and exits if it is unset.
* `~suppress_spacenav` [bool] - also suppress the spacenav while locked, through `/spacenav_wrapper/suppress`. Default: `true`
* `~locked` [bool] - start in the locked state. Default: `false`

#### Published Topics

* `/lg_lock/locked` [`lg_msg_defs/LockState`] - Current lock state, latched.

#### Services

* `/lg_lock/is_locked` [`lg_msg_defs/IsLocked`] - Report the current state.
* `/lg_lock/lock` [`lg_msg_defs/Lock`] - Lock the system.
* `/lg_lock/unlock` [`lg_msg_defs/UnLock`] - Unlock, given the password.
