# my_controller

Here's a simple ROS2 package that demonstrates how integration test
(aka level 2 unit test) can be done using GoogleTest.

Note: The integration test used in this example is purposely kept
simple.  It makes system() calls with the following assumption:

  - Linux OS
  - pkill command

For a more portable ROS2 integration test framework, consider using catch_ros2:

  https://github.com/ngmor/catch_ros2


This ROS2 package depends on the "my_model" module.  This dependency
is specified in the package's package.xml file:

```
  <depend>my_model</depend>
```


