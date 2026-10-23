# D455 → OpenVINS → PX4 1.17

详细步骤见 [接入与地面验证说明](../../../docs/openvins_d455.md)。

默认只发布预览里程计。仅有出厂标定时，不会输出到 MAVROS。
`px4_ground.launch` 要求联合标定和实测机体外参；解锁时停发，不能作为飞行版本。
