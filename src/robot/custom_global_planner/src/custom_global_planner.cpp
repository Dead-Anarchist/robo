#include <pluginlib/class_list_macros.h>
#include "custom_global_planner/custom_global_planner.h"

#include <cmath>

#include <tf/tf.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(custom_global_planner::CustomGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace custom_global_planner {

  CustomGlobalPlanner::CustomGlobalPlanner ()
    : path_directory_added(false), costmap(nullptr), pModule(nullptr), pFunc(nullptr) {

  }

  CustomGlobalPlanner::CustomGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
   : path_directory_added(false), costmap(nullptr), pModule(nullptr), pFunc(nullptr) {
    initialize(name, costmap_ros);
  }


  void CustomGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    costmap = costmap_ros;
  }
/*
  // pure C++ implementation of the planner for demonstation 
  bool CustomGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                                     const geometry_msgs::PoseStamped& goal,
                                     std::vector<geometry_msgs::PoseStamped>& plan ){
    //  initial pose
    plan.push_back(start);
    ROS_INFO("custom planner: first point is %f, %f", start.pose.position.x, start.pose.position.y);
    //  number of points in the trajectory
    unsigned int NP = 20;
    //  deltas to calculate intermediate positions
    float dx = (goal.pose.position.x - start.pose.position.x) / (NP - 1);
    float dy = (goal.pose.position.y - start.pose.position.y) / (NP - 1);
    // orientation
    float traj_yaw = atan2(goal.pose.position.y - start.pose.position.y,
                           goal.pose.position.x - start.pose.position.x);
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(traj_yaw);
    geometry_msgs::PoseStamped interm = goal;
    interm.pose.orientation.x = goal_quat.x();
    interm.pose.orientation.y = goal_quat.y();
    interm.pose.orientation.z = goal_quat.z();
    interm.pose.orientation.w = goal_quat.w();
    
    for (unsigned int i = 1; i < NP-1; i++) {
      interm.pose.position.x = start.pose.position.x + dx*i;
      interm.pose.position.y = start.pose.position.y + dx*i;
      ROS_INFO("custom planner: point %i is %f, %f", i+1,  interm.pose.position.x, interm.pose.position.y);
      plan.push_back(interm);
    }
    plan.push_back(goal);
    ROS_INFO("custom planner: final point is %f, %f", goal.pose.position.x, goal.pose.position.y);
    return true;
  }
*/


  bool CustomGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
				     const geometry_msgs::PoseStamped& goal,
				     std::vector<geometry_msgs::PoseStamped>& plan) {
    
    PyObject *pName;
    PyObject *pArgs, *pValue,  *pValue2,  *pValue3;
    PyObject *pCostmap, *pRow;
    int i;
    if(pModule == nullptr) {
    
        // initialize Python
        Py_Initialize();
        //  add path to the navigation module TODO now export 
        
        //  if it is not already added to PYTHONPATH
        /*
        if (!path_directory_added) {
            wchar_t* current_path = Py_GetPath();
            ROS_INFO("current path: %ls",  current_path);
            wchar_t new_path[wcslen(current_path) + 100] = L"";   //  TODO calculate precise length
            wcscpy(new_path, current_path);
            wcscat(new_path, L":/home/sanya/test_ws/src/robot/custom_global_planner/src/custom_global_planner");
            Py_SetPath(new_path);
            wchar_t* current_path2 = Py_GetPath();
            ROS_INFO("new path: %ls",  current_path2);
            path_directory_added = true;
        }
        */
        
        // load Python script from text file
        PyRun_SimpleString("import sys");
        PyRun_SimpleString("sys.path.append(\"/home/sanya/test_ws/src/robot/custom_global_planner/src/custom_global_planner\")"); 
        const char script_filename[] = "custom_global_planner_function";
        pName = PyUnicode_DecodeFSDefault(script_filename);
        // Error checking of pName left out;
        // import script in python interpreter
        //pModule = PyImport_Import(pName);
        pModule = PyImport_ImportModule(script_filename);
        Py_DECREF(pName);
        // if loading is not successfull
        if (pModule == NULL) {
            // print error message
            //PyErr_Print();
            ROS_ERROR("Failed to load \"%s\"", script_filename);
            return false;
        }
        
        // create function handler
        const char function_name[] = "make_trajectory";
        pFunc = PyObject_GetAttrString(pModule, function_name);
        // pFunc is a new reference;
        // check if function is correct
        if (!pFunc || !PyCallable_Check(pFunc)) {
            ROS_ERROR("Cannot find function \"%s\"", function_name);
            Py_XDECREF(pFunc);
            Py_DECREF(pModule);
            return false;
        }
    }    
    // calculate orientation in start and finish points
    double ori_start = 2*atan2(start.pose.orientation.z, start.pose.orientation.w);
    double ori_goal = 2*atan2(goal.pose.orientation.z, goal.pose.orientation.w);
    
    // get costmap parameters
    double resolution = costmap->getCostmap()->getResolution();
    double origin_x = costmap->getCostmap()->getOriginX();
    double origin_y = costmap->getCostmap()->getOriginY();
    // prepare costmap for python TODO check for errors
    pCostmap = PyList_New(costmap->getCostmap()->getSizeInCellsY());
    for (int i = 0; i < costmap->getCostmap()->getSizeInCellsY(); ++i) {
        pRow = PyList_New(costmap->getCostmap()->getSizeInCellsX());
        for (int j = 0; j < costmap->getCostmap()->getSizeInCellsX(); ++j) {
            // add value from costmap to row
            pValue = PyLong_FromLong(costmap->getCostmap()->getCost(j, i));
            PyList_SetItem(pRow, j, pValue);
            ROS_ERROR("elem [%i , %i] = %i", i,j,costmap->getCostmap()->getCost(j, i));
        }
        // add row to costmap
        PyList_SetItem(pCostmap, costmap->getCostmap()->getSizeInCellsY() - i -  1, pRow);
    }
    
    // create tuple of input arguments
    pArgs = PyTuple_New(10);
    // create 6 variables for arguments and send them to tuple
    pValue = PyFloat_FromDouble(start.pose.position.x);
    PyTuple_SetItem(pArgs, 0, pValue);
    pValue = PyFloat_FromDouble(start.pose.position.y);
    PyTuple_SetItem(pArgs, 1, pValue);
    pValue = PyFloat_FromDouble(ori_start);
    PyTuple_SetItem(pArgs, 2, pValue);
    pValue = PyFloat_FromDouble(goal.pose.position.x);
    PyTuple_SetItem(pArgs, 3, pValue);
    pValue = PyFloat_FromDouble(goal.pose.position.y);
    PyTuple_SetItem(pArgs, 4, pValue);
    pValue = PyFloat_FromDouble(ori_goal);
    PyTuple_SetItem(pArgs, 5, pValue);
    PyTuple_SetItem(pArgs, 6, pCostmap);
    pValue = PyFloat_FromDouble(resolution);
    PyTuple_SetItem(pArgs, 7, pValue);
    pValue = PyFloat_FromDouble(origin_x);
    PyTuple_SetItem(pArgs, 8, pValue);
    pValue = PyFloat_FromDouble(origin_y);
    PyTuple_SetItem(pArgs, 9, pValue);
    // call function
    pValue = PyObject_CallObject(pFunc, pArgs);
    Py_DECREF(pArgs);
    Py_DECREF(pCostmap);
    Py_DECREF(pRow);
    // if call is not successfull, print error message
    if (pValue == NULL) {
      ROS_ERROR("Call failed");
      return false;
    }
    
    // PROCESS RESULTS
    if (!PyTuple_Check(pValue)) {
        Py_DECREF(pValue);
        Py_DECREF(pArgs);
        ROS_ERROR("Wrong type of return value (it is not a tuple)");
        return false;
    }
    Py_ssize_t sz = PyTuple_Size(pValue);
    if (sz < 2) {
        Py_DECREF(pValue);
        Py_DECREF(pArgs);
        ROS_ERROR("The returned tuple is too little (%li instead of 2)",  sz);
        return false;
    }
    
    pValue2 = PyTuple_GetItem(pValue,  0);
    //  check planning error
    if (pValue2 != Py_True) {
        Py_DECREF(pValue);
        Py_DECREF(pValue2);
        Py_DECREF(pArgs);
        ROS_ERROR("Call failed");        
        return false;
    }
    Py_DECREF(pValue2);
    //  read trajectory points
    pValue2 = PyTuple_GetItem(pValue,  1);
    // check type of the second element of the returned tuple
    if (!PyList_Check(pValue2)) {
        Py_DECREF(pValue);
        Py_DECREF(pValue2);
        Py_DECREF(pArgs);
        ROS_ERROR("Second element of the returned tuple is not a list");        
        return false;
    }
    int traj_size = PyList_Size(pValue2);
    plan.push_back(start);
    for (int i = 0; i < traj_size; ++i) {
        // read data from Python list of tuples
        // TODO make reading safe to protect plugin from errors in Python part
        pValue3 = PyList_GetItem(pValue2,  i);
        // check type and size of the element of the list
        if (!PyTuple_Check(pValue3)) {
            Py_DECREF(pValue);
            Py_DECREF(pValue2);
            Py_DECREF(pValue3);
            Py_DECREF(pArgs);
            ROS_ERROR("Wrong type of element %i in the trajectory (it is not a tuple)",  i);
            return false;
        }
        sz = PyTuple_Size(pValue3);
        if (sz < 3) {
            Py_DECREF(pValue);
            Py_DECREF(pValue2);
            Py_DECREF(pValue3);
            Py_DECREF(pArgs);
            ROS_ERROR("The returned trajectory element %i is too short (%li instead of 3)", i, sz);
            return false;
        }        
        float t_x = PyFloat_AsDouble(PyTuple_GetItem(pValue3,  0));
        if (PyErr_Occurred()) {
            Py_DECREF(pValue);
            Py_DECREF(pValue2);
            Py_DECREF(pValue3);
            Py_DECREF(pArgs);
            ROS_ERROR("Error trying to convert element 0 in trajectory point %i to float",  i);
            return false;
        }
        float t_y = PyFloat_AsDouble(PyTuple_GetItem(pValue3,  1));
        if (PyErr_Occurred()) {
            Py_DECREF(pValue);
            Py_DECREF(pValue2);
            Py_DECREF(pValue3);
            Py_DECREF(pArgs);
            ROS_ERROR("Error trying to convert element 1 in trajectory point %i to float",  i);
            return false;
        }
        float t_orient = PyFloat_AsDouble(PyTuple_GetItem(pValue3,  2));
        if (PyErr_Occurred()) {
            Py_DECREF(pValue);
            Py_DECREF(pValue2);
            Py_DECREF(pValue3);
            Py_DECREF(pArgs);
            ROS_ERROR("Error trying to convert element 2 in trajectory point %i to float",  i);
            return false;
        }
        // save data in C++ list
        tf::Quaternion interm_quaternion = tf::createQuaternionFromYaw(t_orient);
        geometry_msgs::PoseStamped interm = goal;
        interm.pose.orientation.x = interm_quaternion.x();
        interm.pose.orientation.y = interm_quaternion.y();
        interm.pose.orientation.z = interm_quaternion.z();
        interm.pose.orientation.w = interm_quaternion.w();
        interm.pose.position.x = t_x;
        interm.pose.position.y = t_y;
        ROS_INFO("POINT :%f,  %f",  t_x,  t_y);
        plan.push_back(interm);
        Py_DECREF(pValue3);
    }
    plan.push_back(goal);
    
    //printf("Result of call: %ld\n", PyLong_AsLong(pValue));
    ROS_INFO("call is successfull\n");
    Py_DECREF(pValue2);
    Py_DECREF(pValue);
    return true;
  }
 
  CustomGlobalPlanner::~CustomGlobalPlanner(){
    Py_FinalizeEx();
  }     

     
 
}; 
