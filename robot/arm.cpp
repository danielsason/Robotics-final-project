moveit::planning_interface::MoveGroup group("arm");
  //ADDED: from robotican arm manipulation tutorial
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPoseReferenceFrame("kinect2_link");
  group.setStartStateToCurrentState();

  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


//ARM CODE
        group_variable_values[0] = 0.0; //base turn
        group_variable_values[1] = 1.0; //base elbow
        group_variable_values[2] = -0.5; //first elbow
        group_variable_values[3] = 0.0; //twist
        group_variable_values[4] = 0.0; //second elbow
        group.setJointValueTarget(group_variable_values);

        bool success = group.plan(my_plan)
        if (success)
        {
          ROS_INFO("Moving...");
          group.move();
        }

        group.clearPoseTargets();
        group.stop();