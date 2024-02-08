# March RQT Input Device
## Overview
This packages is the implementation of the RQT input device, which allows the user to control the exoskeleton using a virtual button interface. In essence, the user can relay instructions to the whole ROS architecture using this packages.

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

## Node structure
### Client
- `'get_exo_mode_array'` ([GetExoModeArray.srv](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/srv/GetExoModeArray.srv))

### Publishers
- `"/march/input_device/alive"` ([Alive](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/msg/Alive.msg))

## Construction of the node
Since the relation between Plugin, View and Controller can be a bit confusing, a sequence diagram has been made to visually aid with comprehending the construction of this node.

![Sequence diagram of constructor/init](../resource/img/ipd_construction.png)

- On construction, first the plugin is created (which can be largely ignored). 
- Afterwards, the controller class is initialized including all the callback functions for each button/exoMode
- Subsequently, the view class is created, and all buttons defined in the layout file are created, linked to a callback function and displayed.
- Lastly, the view class is linked to the controller to allow communication between the two.

## Button presses
**This functionality might seem needlessly convoluted, but so far I have not been able to circumvent this.**
The main problem with correctly relaying the button presses to the ModeMachine, is the alive message that needs to continuouosly be published.

In a usual client/service relation between nodes, the request is sent asyncronously to the server, after which the client waits for the future to be completed. The issues arises from the fact that this is blocking. 

After this block is lifted (i.e. the response has been received), the alive publishing does not continue for some reason?? 
**Note that** the alive message is being published from the Controller class

To circumvent this, it is possible to do the waiting in the View class, but this does create some stupid behaviour between the controller and viewer, since the button presses are handled in the controller. A sequence diagram is shown below to visualize the interactions.

![Sequence diagram of button presses](../resource/img/ipd_button_press.png)