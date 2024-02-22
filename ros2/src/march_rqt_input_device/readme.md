# March RQT Input Device
**Author: Andrew Hutani, MIX**
## Overview
This package is the implementation of the RQT input device, which allows the user to control the exoskeleton using a virtual button interface. In essence, the user can relay instructions to the whole ROS architecture using this packages. Note how the whole layout is dependant on the modes defined in the `modes.json` file in the `march_mode_machine` package. In this file, all possible exoModes are defined, which will all be tied to a button on the RQT Input Device. Note that exoMode::Error and exoMode::BootUp do **not** have a button in the RQT Input Device, since these should not be accessible by the user.

## User Interface Structure
The user interface of this plugin follows the classic Model-View-Controller (MVC) architectural pattern, which separates the application into three interconnected components. This allows for efficient code organization, modular design, and easier maintenance. Blame old Marchies for the naming convention, but in this case the following holds:
- Controller: `InputDevicePlugin`
- View: `InputDeviceView`
- Model:  `InputDeviceController`

I will henceforth refer to these by their class names (i.e. `InputDevicePlugin`, `InputDeviceView`, `InputDeviceController`)

### InputDevicePlugin
_The `InputDevicePlugin` class should acts as the controller in the MVC structure. It is responsible for handling the logic and coordinating between the view and the model. It communicates user actions from the view to the model, and updates the view when the model changes. The controller contains a reference to the InputDeviceController (the model) and manages the settings of the plugin._

In my humble opinion this class can be viewed as a passthrough/wrapper, since it really does not seem to do anything.

### InputDeviceView
The InputDeviceView class represents the view component in the MVC structure. It is responsible for presenting the data to the user and capturing user inputs. The view contains a layout of buttons, each of which is associated with a callback function in the controller. The view updates the state of these buttons based on the current possible modes provided by the controller.

In this view buttons are defined that communicate with the controller

### InputDeviceController
The InputDeviceController class acts as the model in the MVC structure. It manages the data, logic, and rules of the application. In this case, it handles the communication with the ROS 2 network, including publishing mode changes and storing available modes. The controller is independent of the user interface and can be used without the view and the plugin.

This MVC structure allows for a clear separation of concerns. Each component has a specific responsibility, making the code easier to understand and maintain. Changes in one component (for example, a change in the user interface design) can be made independently of the others, reducing the risk of introducing bugs.

## Possible dependencies
PyQt5 version <= 5.11.2
## Node structure
### Client
- `'get_exo_mode_array'` ([GetExoModeArray.srv](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/srv/GetExoModeArray.srv))

### Publishers
- `"/march/input_device/alive"` ([Alive](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/msg/Alive.msg))

## Construction of the node
Since the relation between Plugin, View and Controller can be a bit confusing, a sequence diagram has been made to visually aid with comprehending the construction of this node.

![Sequence diagram of constructor/init](resource/img/ipd_construction.png)

- On construction, first the plugin is created (which can be largely ignored). 
- Afterwards, the controller class is initialized including all the callback functions for each button/exoMode
- Subsequently, the view class is created, and all buttons defined in the layout file are created, linked to a callback function and displayed.
- Lastly, the view class is linked to the controller to allow communication between the two.

To avoid repeating a lot of code for all the different modes, a lambda notation is used to (more or less) tie the button to a specific mode and sending that mode.

> **Using copilot to explain the lambda expression**:
> - _In Python, lambda is used to create small anonymous functions at runtime. These functions are called "anonymous" because they are not declared with the standard def keyword._
> - _The general syntax of a lambda function is: `lambda arguments: expression`._
> - _In your case, `lambda: self._controller.publish_mode(mode)` is a lambda function that takes no arguments (hence the empty : before the colon) and calls `self._controller.publish_mode(mode)` when it's invoked._
> - _The reason we use a lambda function here is because the button.clicked.connect() method expects a function object that it can call when the button is clicked. However, we want to call `self._controller.publish_mode(mode)`, which requires an argument (mode)._
> - _If we simply passed `self._controller.publish_mode` to `button.clicked.connect()`, it wouldn't know what value to use for mode. By using a lambda function, we create a new function that takes no arguments and calls `self._controller.publish_mode(mode)` with the correct value for mode._
> - _So when the button is clicked, it calls the lambda function (which takes no arguments), and the lambda function in turn calls `self._controller.publish_mode(mode)` with the correct value for mode. This is a common pattern when working with GUIs in Python, because often you need to respond to an event (like a button click) with a function that takes some specific arguments._

## Button presses
**This functionality might seem needlessly convoluted, but so far I have not been able to circumvent this.**
The main problem with correctly relaying the button presses to the ModeMachine, is the alive message that needs to continuouosly be published.

In a usual client/service relation between nodes, the request is sent asyncronously to the server, after which the client waits for the future to be completed. The issues arises from the fact that this is blocking. 

After this block is lifted (i.e. the response has been received), the alive publishing does not continue for some reason?? 
**Note that** the alive message is being published from the Controller class

To circumvent this, it is possible to do the waiting in the View class, but this does create some stupid behaviour between the controller and viewer, since the button presses are handled in the controller. A sequence diagram is shown below to visualize the interactions.

![Sequence diagram of button presses](resource/img/ipd_button_press.png)
\