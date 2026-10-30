# Security

The receiver drives a vehicle's motor controller. Report anything that lets
someone else drive, stop or confuse a board privately: *Security* tab →
*Report a vulnerability*. Do not open public issues for throttle, braking,
pairing, BLE or firmware-update flaws.

Pairing model: the receiver advertises only to remotes on its allowlist
(added from the Wi-Fi setup page) and only acts on writes from an encrypted
link (`ESP_GATT_PERM_WRITE_ENCRYPTED`). The static passkey is
`CONFIG_GB_BLE_PASSKEY`; its default is public, so set your own for builds you
ship, with the same value in the remotes. On link loss without a shutdown
command from the remote, the failsafe brakes.
