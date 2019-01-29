cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp std_srvs std_msgs actionlib_msgs message_generation)

add_message_files(	DIRECTORY msg
				 	FILES
				 	ECtoIPD.msg
				 	ECtoUG.msg
				 	ECtoLG.msg
				 	ECtoJH.msg
				 	ECtoPDB.msg
				 	errorMsg.msg
				 	IPDtoEC.msg
				 	PDBtoDL.msg
				 	JHtoEC.msg
				 	RADPos.msg
				 	IUPos.msg
				 	register16Msg.msg
				 	register32Msg.msg
				 	float32Msg.msg
				 	SDOMsg.msg
				 	temperatureMsg.msg
				 	data8Msg.msg
				 	CustomSetpoint.msg
				 	playSound.msg
				 )

add_service_files(	DIRECTORY srv
					FILES
					twoString.srv
					integer.srv
				 )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

#add_executable(custom_msgs)

#add_dependencies(custom_msgs ${catkin_EXPORTED_TARGETS})