
Ur script driver
-----

Get the message types <https://github.com/sequenceplanner/ur_script_msgs>, put them in your workspace with this repo then `colcon build`.

Then test the action:

```
ros2 action send_goal /ur_script ur_script_msgs/action/ExecuteScript '{script: "def code():\nmovej([1.0, 0.0, 0.0, 0.0, 0.0, 0.0],a=0.5,v=0.2)\nend\n"}'
```

You can use ctrl-c to cancel the goal request or leave it to let the goal succeed.
