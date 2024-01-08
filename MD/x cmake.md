```
cmake_minimum_required(VERSION 3.0.2)
```

//指定了`CMake`的最低版本要求为3.0.2。

```
project(hello)
```

//使用`project`命令定义了项目名称为"`hello`"。

```
# add_compile_options(-std=c++11)
```

//告诉`CMake`在编译代码时使用`C++11`标准。

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
```

//使用`find_package`命令查找所需的`ROS`依赖项，包括`roscpp`、`rospy`和`std_msgs`。
通过`find_package`(`catkin` `REQUIRED` `COMPONENT`这个命令，可以声明所需的`catkin`组件，例如`roscpp`、`rospy`和`std_msgs`等。

```
# find_package(Boost REQUIRED COMPONENTS system)
```

//使用`find_package`()命令来查找所需的系统依赖项。这个命令会自动查找并包含 `Boost` 库所需的头文件和库，并设置相关的变量，如 ${`Boost_INCLUDE_DIRS`} 和 ${`Boost_LIBRARIES`} 等。这些变量可以在后续的构建步骤中使用，以正确地引用和链接 `Boost` 库。

```
# catkin_python_setup()
```

//通过取消注释`catkin_python_setup`()宏，可以确保`setup.py`文件中声明的模块和全局脚本会被安装到`ROS`软件包的安装目录中，使其可以在其他`ROS`节点中使用。

```
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```

//在`ROS`软件包的`CMakeLists.txt`文件中，有一个`add_message_files`()命令用于声明要生成的消息文件。

```
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
```

//`add_service_files`()命令用于声明要生成的服务文件。注释掉了`add_service_files`()的调用，因此默认情况下不会生成任何服务文件。要生成服务文件，你需要取消注释这个命令，并列出要处理的每个.`srv`文件。

```
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
```

//`add_action_files`()命令用于声明要生成的动作文件。注释掉了`add_action_files`()的调用，因此默认情况下不会生成任何动作文件。要生成动作文件，你需要取消注释这个命令，并列出要处理的每个.`action`文件。

```
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
```

//`generate_messages`()命令用于生成已添加的消息和服务的源代码，并指定它们所依赖的其他消息类型。注释掉了`generate_messages`()的调用，因此默认情况下不会生成任何消息和服务的源代码。要生成源代码，你需要取消注释这个命令，并在`DEPENDENCIES`部分列出已添加的消息和服务所依赖的其他消息类型。

```
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )
```

//`generate_dynamic_reconfigure_options`是一个用于生成动态重新配置源代码的`CMake`函数。它会读取指定的.`cfg`文件，并根据其内容生成相应的源代码文件。

```
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
```

//在`ROS`软件包的`CMakeLists.txt`文件中使用`catkin_package`宏来生成用于构建配置的`CMake`文件。

//`INCLUDE_DIRS`: 如果软件包包含头文件（.`h`文件），需要取消注释这一行，并指定包含这些头文件的目录路径。

//`LIBRARIES`：如果在该项目中创建了库文件，且其他依赖项目也需要使用这些库文件，需要取消注释这一行，并指定要链接的库文件名。

// `CATKIN_DEPENDS`：如果该软件包依赖其他`ROS`软件包，需要取消注释这一行，并列出所依赖的这些软件包。

```
include_directories(
  /home/tx-deepocean/Library/gsl_gcc/include
  ${catkin_INCLUDE_DIRS}
)
```

//`include_directories()` 函数用于将指定的路径添加到编译器的头文件搜索路径中。

在这里，使用了 `include_directories()` 函数来指定两个路径，分别是 `/home/tx-deepocean/Library/gsl_gcc/include` 和 `${catkin_INCLUDE_DIRS}`。

`/home/tx-deepocean/Library/gsl_gcc/include` 是一个具体的路径，表示将该路径添加到头文件搜索路径中。

`${catkin_INCLUDE_DIRS}` 是一个变量，表示将 catkin 提供的头文件搜索路径添加到编译器中。这个变量由 catkin 构建系统提供，并包含了一些系统和库的头文件路径。

```
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hello.cpp
# )
```

//使用 `add_library()` 函数添加一个库目标，将 `src/${PROJECT_NAME}/hello.cpp` 添加到库的源代码文件列表中，以便在构建时将该源文件编译成库文件，并在链接其他目标时使用该库中的函数和变量。

```
add_executable(hello1 src/spline.cpp)
link_directories(
  ${catkin_LIB_DIRS}
  /home/tx-deepocean/Library/gsl_gcc/lib/libgsl.so
  )
  link_directories(
  ${catkin_LIB_DIRS}
  /home/tx-deepocean/Library/gsl_gcc/lib/libgslcblas.so
  )
```

//创建了一个可执行目标 `hello1`，而且指定的额外库文件路径只包含了 `/home/tx-deepocean/Library/gsl_gcc/lib/libgsl.so` 和 `/home/tx-deepocean/Library/gsl_gcc/lib/libgslcblas.so` 两个库文件。`link_directories()` 函数用于指定链接器搜索库文件的路径，`${catkin_LIB_DIRS}` 是一个变量，表示将 `catkin` 提供的库搜索路径添加到链接器中。

```
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
```

//这行代码用于设置目标（`Target`）的属性。`${PROJECT_NAME}_node` 是目标的名称，`${PROJECT_NAME}` 是一个变量，表示项目的名称。`OUTPUT_NAME` 指定了目标生成的可执行文件的名称，`node` 是生成的可执行文件的名称。`PREFIX` 则指定了生成的可执行文件的前缀，在这里设为空字符串，表示没有前缀。

```
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

```

//这行代码用于添加目标（`Target`）之间的依赖关系。`add_dependencies()` 函数用于添加依赖关系，第一个参数是目标的名称，后面的参数表示该目标所依赖的其他目标。在这里，`${PROJECT_NAME}_node` 目标依赖于 `${${PROJECT_NAME}_EXPORTED_TARGETS}` 和 `${catkin_EXPORTED_TARGETS}` 这两个目标。

```
target_link_libraries(hello1
  ${catkin_LIBRARIES}
  gsl gslcblas
)
```

//这行代码用于将库文件链接到目标（`Target`）中。`target_link_libraries()` 函数用于指定目标需要链接的库文件。第一个参数是目标的名称，在这里是 `hello1`。后面的参数表示需要链接的库文件。

`${catkin_LIBRARIES}` 是一个变量，表示 `catkin` 提供的库文件。它包含了所有由 `catkin` 构建系统提供的库文件路径。通过将 `${catkin_LIBRARIES}` 添加到 `hello1` 目标的链接库列表中，可以确保在构建时能够正确地链接所需的 catkin 库文件。

`gsl` 和 `gslcblas` 是额外的库文件，它们将被链接到 `hello1` 目标中。这两个库文件可能是自定义的或者来自其他库。

```
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
```

//`ROS`中的一个编程规范，建议所有的安装目标（如可执行文件、库文件等）都应该使用`catkin`的`DESTINATION`变量。这样做可以使得安装目标被正确地安装到`catkin`工作空间中的正确位置，避免出现错误。具体来说，`DESTINATION`变量指定了安装目标的安装路径。

```
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
```

//在`CMakeLists.txt`中使用`catkin_install_python`命令来安装一个`Python`脚本。具体来说，这个命令会将指定的`Python`脚本（如`scripts/my_python_script`）安装到`${CATKIN_PACKAGE_BIN_DESTINATION}`目录下，该目录是catkin工作空间中bin目录的绝对路径，因此这个脚本可以被作为可执行文件直接调用。

```
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
```

//在`CMakeLists.txt`中使用`install`命令来安装一个可执行文件，该可执行文件的名字为`${PROJECT_NAME}_node`。具体来说，该命令将`${PROJECT_NAME}_node`安装到`${CATKIN_PACKAGE_BIN_DESTINATION}`目录下，该目录是`catkin`工作空间中`bin`目录的绝对路径，因此该可执行文件可以被作为可执行文件直接调用。`${PROJECT_NAME}`是`CMakeLists.txt`中定义的项目名称。

```
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
```

//使用`install`命令来安装一个库和一个可执行文件，该库和可执行文件的名字都为`${PROJECT_NAME}`。具体来说，该命令将`${PROJECT_NAME}`库安装到`${CATKIN_PACKAGE_LIB_DESTINATION}`目录下，`${PROJECT_NAME}`可执行文件安装到`${CATKIN_GLOBAL_BIN_DESTINATION}`目录下。`${CATKIN_PACKAGE_LIB_DESTINATION}`是catkin工作空间中lib目录的绝对路径，`${CATKIN_GLOBAL_BIN_DESTINATION}`是全局bin目录的绝对路径。

```
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
```

//在`CMakeLists.txt`中使用`install`命令来安装一个目录下的头文件。具体来说，该命令将`${PROJECT_NAME}`包含的头文件安装到`${CATKIN_PACKAGE_INCLUDE_DESTINATION}`目录下，该目录是`catkin`工作空间中include目录的绝对路径。`${PROJECT_NAME}`是`CMakeLists.txt`中定义的项目名称。

```
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
```

//这是一个 `CMakeLists.txt` 文件中的一段代码，用于在构建 `ROS` 软件包时安装文件。

`FILES` 表示要安装的文件列表。由于 `FILES` 中的内容都被注释掉了，因此并没有实际要安装的文件。

`DESTINATION` 指定了文件的目标安装路径。`${CATKIN_PACKAGE_SHARE_DESTINATION}` 是一个变量，表示软件包共享目录的路径。在 `ROS` 中，共享目录通常用于存储与软件包相关的数据文件、配置文件等。

```
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hello.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
```

//这是一个 `CMakeLists.txt` 文件中的一段代码，用于在 `ROS` 软件包中添加测试。

`catkin_add_gtest(${PROJECT_NAME}-test test/test_hello.cpp)` 表示在软件包中添加一个 `GTest` 测试。`${PROJECT_NAME}-test` 是测试的名称，`test/test_hello.cpp` 是测试文件的路径。

`if(TARGET ${PROJECT_NAME}-test)` 表示如果存在 `${PROJECT_NAME}-test` 目标（即测试），则执行下面的代码块。

`target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})` 表示将 `${PROJECT_NAME}`（即软件包）作为依赖链接到 `${PROJECT_NAME}-test` 目标上。这样做可以确保测试能够访问软件包中的函数、类等。

```
# catkin_add_nosetests(test)
```

//用于在 `ROS` 软件包中添加 `Python` `Nose` 测试。

`catkin_add_nosetests(test)` 表示在软件包中添加一个 `Nose` 测试。`test` 是测试文件夹的路径。`Nose` 是 `Python` 中的一个测试框架，用于编写和运行测试。
