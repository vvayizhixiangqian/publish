cmake

### 一、基础介绍

Catkin 需要 3.0.2 或更高版本，首先需要在文件开头包含版本声明：

```
cmake_minimum_required(VERSION 3.0.2)
```

CMake项目函数指定包的名称，指定了功能包的名称后，可以在文件中使用${PROJECT_NAME } 宏随时使用该变量。

```
project(hello)
```

表示为所有编译器添加c++11支持

```
# add_compile_options(-std=c++11)
```



### 二、依赖包

#### find_package负责找依赖路径，catkin_package负责声明编译构建时的依赖信息

#### 1、find_package

对于这个新建的catkin软件包，将它依赖的其他catkin packages(roscpp、rospy、std_msgs)指定为catin的组件，这样操作会使它们的include路径、libraries路径等附件到catkin_variables中。例如，catkin_INCLUDE_DIRS 不仅包含了catkin package自己的include路径，还包含组件（例如上面的roscpp、rospy、std_msgs）的include路径，在后续的使用中会很方便！
查找编译所需的其他 CMake/Catkin 包，也就是通过find_package()可以完成找到头文件和库的目的。

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
```

```
# find_package(Boost REQUIRED COMPONENTS system)
```

这是因为`find_package()`一次只能查找一个package，boost组件库可以作为单独的包，catkin是指ROS工程中使用catkin工具编译的packages，而且凡是有catkin编译的包都可以作为catkin的组件，也可以作为单独的package，属于第三方package的要单独写。这里查找C++ Boost库的组件库system、thread。

#### 2、catkin_package

catkin_package()是catkin提供的CMake宏，用于为catkin提供构建、生成pkg-config和CMake文件所需要的信息。在使用add_library()和add_executable()函数声明目标之前，必须的调用这个函数。这个函数有5个可选的参数。

是给依赖本包的其他包使用的。可以向其他package导出依赖，这些依赖可能包含头文件、库，或者本package依赖的其他package。其他软件包package使用find_package(…)时就会加载配置文件了，从而获得依赖项。

```
catkin_package(
#  INCLUDE_DIRS include// 包的导出包含路径
#  LIBRARIES hello//从项目中导出的库
#  CATKIN_DEPENDS roscpp rospy std_msgs//本项目依赖的其他catkin项目
#  DEPENDS system_lib//此项目所依赖的非 catkin CMake 项目
)//指定构建包时的信息输出
```

### 二、构建消息、服务、动作

ROS 中的消息 (.msg)、服务 (.srv) 和动作 (.action) 文件在被 ROS 包构建和使用之前需要一个特殊的预处理器构建步骤。这些宏是生成指定编程语言的文件的关键，以便可以使用所选编程语言中的消息，服务和操作。

提供了三个宏来分别处理消息，服务和操作：

```
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )//Message生成文件
```

```
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )//serive生成文件
```

```
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )//action生成文件
```

启动message生成器，调用`消息`/`服务`/`动作`生成特定语言的接口文件，生成对应的.h文件

```
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
```

add_dependencies面向的是msg，service或者dynamic reconfigure，编译的程序使用到了自己构建消息、服务、动作，则需要创建对自动生成的消息目标的显式依赖，以便它们以正确的顺序构建。

已经通过find_package()引入了这个package。

定义目标文件 target 依赖的其他目标文件 target ,确保在编译本 target 之前,其他的 target 已经被构建。

```
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

### 三、构建目标

构建目标一般有以下两种情况：

- 构建可执行目标 - 我们可以运行的程序
- 构建库目标 - 可执行目标在构建和/或运行时可以使用的库

**include_directories()**

 include_directories的参数应该是由 find_package 调用生成的 *_INCLUDE_DIRS 变量以及需要包含的任何其他目录（一般是我们自己功能包下的include文件夹）

```
include_directories(
  /home/tx-deepocean/Library/gsl_gcc/include
  ${catkin_INCLUDE_DIRS}
)
```

**add_executable()**

要指定必须构建的可执行目标，我们必须使用add_executable() CMake 函数。

经过尝试发现add_executable必须是第一个，add_dependencies和target_link_libraries谁先谁后没有关系。

```
add_executable(hello1 src/spline.cpp)
```

**target_link_libraries()**

通常在调用add_executable()之后，使用target_link_libraries()函数指定可执行目标链接到哪些库。

```
target_link_libraries(hello1
  ${catkin_LIBRARIES}
  gsl gslcblas
)
```

**add_library**

生成库文件

```
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hello.cpp
# )
```

**link_directories()**，官网建议不要使用相对路径，直接使用target_link_libraries()，不影响程序运行。测试正常。

```
link_directories(

  ${catkin_LIB_DIRS}

  /home/tx-deepocean/Library/gsl_gcc/lib

  )

  link_directories(

  ${catkin_LIB_DIRS}

  /home/tx-deepocean/Library/gsl_gcc/lib

  )
```

catkin新加宏，生成测试

```
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hello.cpp)
```

