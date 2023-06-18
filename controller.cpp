#define _GLIBCXX_USE_NANOSLEEP
#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdio>

// roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

#define M_PI  3.14159265358979323846 

std::string readXML(std::string filename){
    std::ifstream inFile;
    std::string line;
    std::string out;

    inFile.open(filename, std::ios::out | std::ios::app | std::ios::binary);
    while (getline(inFile, line)){
        out.append(line);
    }

    return out;
}

void printVector(std::vector<int>& grid){
    for (int i=0; i<grid.size(); i++){    
        std::cout << grid[i] << ' ';     
    }
    std::cout << '\n';
}

void printVector(std::vector<uint8_t>& grid, int& N){
    for (int i=0; i<grid.size()/N; i++){
        for (int j=0; j<N; j++){
            std::cout << grid[i*N + j];
        }
        std::cout << '\n';
    }
}

gazebo_msgs::SpawnModel getModelSpawn(int& x, int& y, int& cubeIndex){
    gazebo_msgs::SpawnModel model;

    model.request.model_name = "Cube" + std::to_string(cubeIndex);
    model.request.model_xml = readXML("C:\\ws\\catkin_ws\\src\\turtlebot3_gazebo\\models\\cube\\model.sdf");
    model.request.initial_pose.orientation.w = 0;
    model.request.initial_pose.orientation.x = 0;
    model.request.initial_pose.orientation.y = 0;
    model.request.initial_pose.orientation.z = 1;
    model.request.initial_pose.position.x = x;
    model.request.initial_pose.position.y = y;
    model.request.initial_pose.position.z = 0.5;
    model.request.reference_frame = "world";
    model.request.robot_namespace = "gazebo";

    return model;
}

void spawnBlock(int& x, int& y, int& cubeIndex, ros::NodeHandle& n){
    gazebo_msgs::SpawnModel model = getModelSpawn(x, y, cubeIndex);
    ros::ServiceClient spawner = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

    if(!spawner.call(model)){
        std::runtime_error("Terminating due to failed spawn call\n");
    }
}

bool checkPossibility(int robotIndex, int& goalIndex, std::vector<int>& listCubes){
    return true;
}

std::vector<uint8_t> generateGrid(int N, int M, double blockDensity, ros::NodeHandle& n, int& goalIndex){
    bool goalPossible = false;
    std::vector<uint8_t> grid;
    grid.resize(N * M);

    while (!goalPossible) {
        std::vector<int> listCubes;
        int cubeIndex = 0;
        
        std::pair<int, int> goal;
        bool change = rand() % 2;
        if (change){
            goal.first = rand() % N;
            goal.second = M - 1;
        }
        else {
            goal.first = N - 1;
            goal.second = rand() % M;
        }
        goalIndex = goal.first + goal.second*N;

        for (int i =0; i < N*M; i++){
            int x = i % N;
            int y = i / N;

            if (rand() < RAND_MAX * blockDensity && x*x + y*y > 2 && y*N + x != goalIndex){
                spawnBlock(x, y, cubeIndex, n);
                listCubes.push_back(i);
                cubeIndex ++;      
                grid[i] = 'C';
            }
            else {
                grid[i] = 'S';
            }
        
        }
        grid[goal.first + goal.second*N] = 'G';

        goalPossible = checkPossibility(0, goalIndex, listCubes);
    }

    grid[0] = 'R';
    return grid;
}

std::vector<uint8_t> buildRobotPerception(std::vector<uint8_t>& grid, int N){
    std::vector<uint8_t> robotGrid;
    robotGrid.resize(grid.size());
    int rx;
    int ry;
    for (int i=0; i<robotGrid.size(); i++){
       
        if (grid[i] == 'R'){
            robotGrid[i] = 'R';
            rx = i % N;
            ry = i / N;
        }
        else if (grid[i] == 'G'){
            robotGrid[i] = 'G';
        }
        else {
            robotGrid[i] = 'X';
        }  
    }

    for (int i=0; i<4*N; i++){
        int x;
        int y;
        if (i < N){
            x = N;
            y = 0;
        }
        else if (i < 2*N){
            x = N - 1;
            y = i - N;
        }
        else if (i < 3*N){
            x = i - 2*N;
            y = N - 1;
        }
        else if (i < 4*N){
            x = 0;
            y = i - 3*N;
        }
   

        int xDist = x - rx;
        int yDist = y - ry;
        if (xDist >= yDist){
            for (int j=1; j<=xDist; j++){
                int tempX = rx + j;
                int tempY = ry + j * ((double) yDist/xDist);
                int gridIndex = tempX + N*tempY;
                if (grid[gridIndex] == 'S'){
                    robotGrid[gridIndex] = 'S';
                }
                else if (grid[gridIndex] == 'C'){
                    robotGrid[gridIndex] = 'C';
                    break;
                }
            }
        }
        else if(yDist > xDist){
            for (int j=1; j<=yDist; j++){
                int tempY = rx + j;
                int tempX = ry + j * ((double) xDist/yDist);
                int gridIndex = tempX + N*tempY;
                if (grid[gridIndex] == 'S'){
                    robotGrid[gridIndex] = 'S';
                }
                else if (grid[gridIndex] == 'C'){
                    robotGrid[gridIndex] = 'C';
                    break;
                }
            }
        }
    }

    printVector(robotGrid, N);
    return robotGrid;
}

void rebuildRobotPerception(int rx, int ry, std::vector<uint8_t>& grid, std::vector<uint8_t>& robotGrid, int N){

    for (int i=0; i<4*N - 2; i++){
        int x;
        int y;
        if (i < N){
            x = i;
            y = 0;
        }
        else if (i < 2*N - 1){
            x = N - 1;
            y = i - (N - 1);
        }
        else if (i < 3*N - 1){
            x = i - (2*N - 1);
            y = N - 1;
        }
        else if (i < 4*N - 2){
            x = 0;
            y = i - (3*N - 1);
        }
   

        int xDist = abs(x - rx);
        int yDist = abs(y - ry);
        if (xDist >= yDist){
            for (int j=1; j<=xDist; j++){
                int tempX = rx + j * (x > rx) - j * (x < rx);
                int tempY = ry + j * ((double) yDist/xDist) * (y > ry) - j * ((double) yDist/xDist) * (y < ry);
                int gridIndex = tempX + N*tempY;
                if (grid[gridIndex] == 'S'){
                    robotGrid[gridIndex] = 'S';
                }
                else if (grid[gridIndex] == 'C'){
                    robotGrid[gridIndex] = 'C';
                    break;
                }           
            }
        }
        else if(yDist > xDist){
            for (int j=1; j<=yDist; j++){
                int tempY = ry + j * (y > ry) - j * (y < ry);
                int tempX = rx + j * ((double) xDist/yDist) * (x > rx) - j * ((double) xDist/yDist) * (x < rx);
                int gridIndex = tempX + N*tempY;
                if (grid[gridIndex] == 'S'){
                    robotGrid[gridIndex] = 'S';
                }
                else if (grid[gridIndex] == 'C'){
                    robotGrid[gridIndex] = 'C';
                    break;
                }
            }
        }
    }

    printVector(robotGrid, N);
    std::cout << '\n';
}

int decideMove(int& rx, int& ry, std::vector<uint8_t>& robotGrid, int& N, int goalIndex, std::vector<int>& robotListCubes, std::vector<int>& path, bool& valid){
    int move = path[0];

    if (rx == 0 && move == 2){
        valid = false;
    }
    else if (rx == N - 1 && move == 0){
        valid = false;
    }
    else if (ry == N - 1 && move == 1){
        valid = false;
    }
    else if (ry == 0 && move == 3){
        valid = false;
    }
    else if (move == 0 && (robotGrid[rx + 1 + ry*N] == 'C' || robotGrid[rx + 1 + ry*N] == 'X')){
        valid = false;
    }
    else if (move == 1 && (robotGrid[rx + (ry + 1)*N] == 'C' || robotGrid[rx + (ry + 1)*N] == 'X')){
        valid = false;
    }
    else if (move == 2 && (robotGrid[rx - 1 + ry*N] == 'C' || robotGrid[rx - 1 + ry*N] == 'X')){
        valid = false;
    }
    else if (move == 3 && (robotGrid[rx + (ry - 1)*N] == 'C' || robotGrid[rx + (ry - 1)*N] == 'X')){
        valid = false;
    }
    else {
        valid = true;
        path.erase(path.begin());
    }
     
    return move;
}

void rotateRobot(double toRotate, ros::NodeHandle& n){
    if (toRotate > 180){
        toRotate -= 360;
    }
    if (toRotate <= -180){
        toRotate += 360;
    }

    ros::Publisher robotRotation = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Duration rotateTime(abs(toRotate) * M_PI / 180.0);
    geometry_msgs::Twist robotDetails;
    if (toRotate > 0){
        robotDetails.angular.z = 1;
    } 
    else {
        robotDetails.angular.z = -1;
    }

    robotRotation.publish(robotDetails);
    rotateTime.sleep();
    robotDetails.angular.z = 0;
    robotRotation.publish(robotDetails);
    
}

geometry_msgs::Point getTarget(int& rx, int& ry, int& N, int& move){
    int x = rx;
    int y = ry;

    if (move == 0){
        x++;
    }
    if (move == 1){
        y++;
    }
    if (move == 2){
        x--;
    }
    if (move == 3){
        y--;
    }

    geometry_msgs::Point out;
    out.x = x;
    out.y = y;

    return out;
}

double groundDistance(geometry_msgs::Point currentPosition, geometry_msgs::Point targetPosition){
    return sqrt((targetPosition.x - currentPosition.x)*(targetPosition.x - currentPosition.x) + (targetPosition.y - currentPosition.y)*(targetPosition.y - currentPosition.y));
}

double getTargetRotation(geometry_msgs::Point currentPosition, geometry_msgs::Point targetPosition, double& distance){
    double xDelta = targetPosition.x - currentPosition.x;
    double yDelta = targetPosition.y - currentPosition.y;
    distance = groundDistance(targetPosition, currentPosition);
    double rotation;

    if (yDelta >= 0){
        rotation = acos(xDelta/distance) * 180 / M_PI;
    }
    else if (yDelta < 0){
        rotation = -acos(xDelta/distance) * 180 / M_PI;
    }
    
    return rotation;
}

void updateGrid(int& rx, int& ry, int& move, std::vector<uint8_t>& grid, std::vector<uint8_t>& robotGrid, int& N){
    grid[rx + ry*N] = 'S';
    robotGrid[rx + ry*N] = 'S';
    if (move == 0){
        rx ++;
    }
    else if (move == 1){
        ry ++;
    }
    else if (move == 2){
        rx --;
    }
    else if (move == 3){
        ry --;
    }

    grid[rx + ry*N] = 'R';
    robotGrid[rx + ry*N] = 'R';
}

void checkGoal(int& rx, int& ry, int& goalIndex, bool& goal, int& N){
    if (rx + ry*N == goalIndex){
        goal = true;
    }
}

std::vector<int> getCubeList (std::vector<uint8_t>& grid){
    std::vector<int> cubeList;

    for (int i=0; i<grid.size(); i++){
        if (grid[i] == 'C'){
            cubeList.push_back(i);
        }     
    }

    return cubeList;
}

gazebo_msgs::ModelState initaliseRobotState(int rx = 0, int ry = 0){
    gazebo_msgs::ModelState robotState;

    robotState.model_name = "turtlebot3_burger";
    robotState.pose.position.x = rx;
    robotState.pose.position.y = ry;
    robotState.pose.orientation.w = 1;
    robotState.pose.orientation.x = 0;
    robotState.pose.orientation.y = 0;
    robotState.pose.orientation.z = 1;
    robotState.twist.linear.x = 0;
    robotState.twist.linear.y = 0;
    robotState.twist.linear.z = 0;
    robotState.twist.angular.x = 0;
    robotState.twist.angular.y = 0;
    robotState.twist.angular.z = 0;

    return robotState;
}

double errorDistance(geometry_msgs::Point v1, geometry_msgs::Point v2){
    return sqrt((v2.x - v1.x)*(v2.x - v1.x) + (v2.y - v1.y)*(v2.y - v1.y));
}

std::ostream& operator <<(std::ostream& os, geometry_msgs::Point p){
    os << p.x << ' ' << p.y << ' ' << p.z << ' ';

    return os;
}

void orientRobot(double& robotRotation, double& targetRotation, geometry_msgs::Twist& velocity, ros::Publisher& robotVelocity, ros::ServiceClient& getModelState, gazebo_msgs::GetModelState& getRobotState, ros::NodeHandle& n, ros::Rate& rate, double coefficient, double factor){
    while (abs(robotRotation - targetRotation) > 1){
            double toRotate = targetRotation - robotRotation;
            if (toRotate > 180){
                toRotate -= 360;
            }
            else if (toRotate <= -180){
                toRotate += 360;
            }
            if (toRotate > 0){
                velocity.angular.z = coefficient + factor * toRotate;
            }
            else {
                velocity.angular.z = -coefficient + factor * toRotate;
            }

            robotVelocity.publish(velocity);
            getModelState.call(getRobotState);

            ros::spinOnce();
            rate.sleep();
            if (getRobotState.response.pose.orientation.w > 0){
                robotRotation = asin(getRobotState.response.pose.orientation.z) * 360 / M_PI; 
            }
            else{
                robotRotation = asin(-getRobotState.response.pose.orientation.z) * 360 / M_PI; 
            }
                    
            std::cout << getRobotState.response.pose.orientation.w << ' ' << getRobotState.response.pose.orientation.x << ' ' << getRobotState.response.pose.orientation.y << ' ';
            std::cout << getRobotState.response.pose.orientation.z << ' ' << robotRotation << ' ' << targetRotation << '\n';
        }

        while(abs(getRobotState.response.twist.angular.z) > 0.001){
            velocity.angular.z = -0.5 * getRobotState.response.twist.angular.z;
            
            robotVelocity.publish(velocity);
            getModelState.call(getRobotState);
            ros::spinOnce();
            rate.sleep();
            std::cout << getRobotState.response.twist.angular.x << ' ' << getRobotState.response.twist.angular.y << ' ' << getRobotState.response.twist.angular.z << '\n';
        }
        std::cout << getRobotState.response.twist.angular.x << ' ' << getRobotState.response.twist.angular.y << ' ' << getRobotState.response.twist.angular.z << '\n';
}

void moveRobot(geometry_msgs::Point& currentPosition, geometry_msgs::Point& targetPosition, geometry_msgs::Twist& velocity, ros::Publisher& robotVelocity, ros::ServiceClient& getModelState, gazebo_msgs::GetModelState& getRobotState, ros::NodeHandle& n, ros::Rate& rate, double coefficient){
    while(errorDistance(currentPosition, targetPosition) > 0.25){
        velocity.linear.x = coefficient;
        robotVelocity.publish(velocity);
        ros::spinOnce();
        rate.sleep();

        getModelState.call(getRobotState);


        currentPosition = getRobotState.response.pose.position;

        std::cout << currentPosition << ' ' << targetPosition << '\n';
    }
}

void stopRobot(geometry_msgs::Twist& velocity, ros::Publisher& robotVelocity, ros::ServiceClient& getModelState, gazebo_msgs::GetModelState& getRobotState, ros::NodeHandle& n, ros::Rate& rate, double threshold, double baseCoefficient){
    getRobotState.request.model_name = "turtlebot3_burger";
    getModelState.call(getRobotState);
    ros::spinOnce();
    rate.sleep();
    double coefficient = baseCoefficient;
    while(abs(getRobotState.response.twist.linear.x) > threshold){
        if (getRobotState.response.twist.linear.x > 0){
            velocity.linear.x = -0.003 -coefficient*getRobotState.response.twist.linear.x;
        }
        else if (getRobotState.response.twist.linear.x < 0){
            velocity.linear.x = 0.003 -coefficient*getRobotState.response.twist.linear.x;
            
        }
        velocity.linear.y = 0;
        
        std::cout << velocity.linear.x << ' ' << velocity.linear.y << ' ' << velocity.linear.z <<  '\n';
        robotVelocity.publish(velocity);
        getModelState.call(getRobotState);
        ros::spinOnce();
        rate.sleep();
        std::cout << getRobotState.response.pose.orientation.w << ' ' << getRobotState.response.twist.linear.x << ' ' << getRobotState.response.twist.linear.y << ' ' << getRobotState.response.twist.linear.z << '\n';
    }
    coefficient = baseCoefficient;
    while(abs(getRobotState.response.twist.linear.y) > threshold){
        if (getRobotState.response.twist.linear.y > 0){
            velocity.linear.y = -0.003 -coefficient*getRobotState.response.twist.linear.y;
        }
        else if (getRobotState.response.twist.linear.y < 0){
            velocity.linear.y = 0.003 -coefficient*getRobotState.response.twist.linear.y;
        }

        velocity.linear.x = 0;
        
        std::cout << velocity.linear.x << ' ' << velocity.linear.y << ' ' << velocity.linear.z <<  '\n';
        robotVelocity.publish(velocity);
        getModelState.call(getRobotState);
        ros::spinOnce();
        rate.sleep();
        std::cout << getRobotState.response.pose.orientation.w << ' ' << getRobotState.response.twist.linear.x << ' ' << getRobotState.response.twist.linear.y << ' ' << getRobotState.response.twist.linear.z << '\n';
    }
}

void getStateInformation(ros::ServiceClient& getModelState, gazebo_msgs::GetModelState& getRobotState, ros::NodeHandle& n, ros::Rate& rate, int& rx, int& ry, int& N, int& move, double& robotRotation, geometry_msgs::Point& currentPosition, geometry_msgs::Point& targetPosition){
    getModelState.call(getRobotState);
    if (getRobotState.response.pose.orientation.w > 0){
        robotRotation = asin(getRobotState.response.pose.orientation.z) * 360 / M_PI; 
    }
    else{
        robotRotation = asin(-getRobotState.response.pose.orientation.z) * 360 / M_PI; 
    }
    
    currentPosition = getRobotState.response.pose.position;
    targetPosition = getTarget(rx, ry, N, move);  
    rate.sleep();
    ros::spinOnce();
}

void initialiseRobot(geometry_msgs::Twist& velocity, ros::Publisher& robotVelocity, ros::ServiceClient& getModelState, gazebo_msgs::GetModelState& getRobotState, ros::NodeHandle& n, ros::Rate& rate, ros::ServiceClient& setModelState, gazebo_msgs::SetModelState& setRobotState, gazebo_msgs::ModelState& robotState){
    std::cout << "Stopping robot\n";
    stopRobot(velocity, robotVelocity, getModelState, getRobotState, n, rate, 0.002, 0.1);
    setModelState.call(setRobotState);
  
    rate.sleep();
    ros::spinOnce();
    std::cout << "Stopping robot\n";
    stopRobot(velocity, robotVelocity, getModelState, getRobotState, n, rate, 0.002, 0.1);
}

void clearGrid(ros::ServiceClient& getWorldProperties, gazebo_msgs::GetWorldProperties& world, gazebo_msgs::DeleteModel& cube, ros::ServiceClient& deleteModel, ros::NodeHandle& n){
    if (!getWorldProperties.call(world)){
        std::cout << "Terminating due to failed get world properties call\n";
    }

    std::vector<std::string> models = world.response.model_names;
    for (std::string model : models){
        std::cout << model << ' ';
        if (model.substr(0,4) == "Cube"){
            cube.request.model_name = model;
            deleteModel.call(cube);
        }
    }
}

void defineVariable(std::string varName, int val, std::ofstream& file){
    std::string add;
    std::string define = "#define ";

    add.append(define);
    add.append(varName);
    add.append(std::to_string(val));
    add.append(";\n");
    file << add;
    return;
}

void addArray(std::string varName, std::vector<int> arr, std::ofstream& file){

    std::string add;
    add.append("var ");
    add.append(varName);
    add.append(" = "); 
    add.append("[");
    for (int i=0; i<arr.size(); i++){
        add.append(std::to_string(arr[i]));

        if (i != arr.size() - 1){
            add.append(",");
        }
        
    }
    add.append("];\n");

    file << add;
    return;
}

void callCommand(const char* command){
    system(command);
}

std::vector<int> PATMove(int rx, int ry, int N, int M, int goalIndex, std::vector<int>& robotListCubes, double& generationTime){
    if (robotListCubes.size() == 0){
        robotListCubes.push_back(999);
    }

    std::clock_t start = clock();
    int robotIndex = rx + N*ry;
    std::vector<int> path;
    int outputLength;
    
    std::string inputPAT;
    std::ofstream outFile("C:/ws/catkin_ws/src/assignment3/src/mazeNavHeader.csp");
    std::ifstream skeleton("C:/ws/catkin_ws/src/assignment3/src/mazeNavSkeleton.csp");

    inputPAT = "#import \"PAT.Lib.List\";\n";
    outFile << inputPAT;

    defineVariable("W ", N, outFile);
    defineVariable("H ", M, outFile);
    defineVariable("goal ", goalIndex, outFile);
    defineVariable("source ", robotIndex, outFile);

    addArray("obst_array", robotListCubes, outFile);
    char letter;
    

    std::string line;
    int index = 0;
    while(std::getline(skeleton, line)){
        if (index  != 0){
            outFile << line << '\n';         
        }
        index++;
    }

    skeleton.close();
    outFile.close();
    
    const char command[] = "PAT3.Console.exe -csp -engine 0 C:/ws/catkin_ws/src/assignment3/src/mazeNavHeader.csp C:/ws/catkin_ws/src/assignment3/src/outputPAT.txt";
    std::cout << command << '\n';
    callCommand(command);
    
    std::ifstream inFile("C:/ws/catkin_ws/src/assignment3/src/outputPAT.txt");
    for (int i=0; i<5; i++){
        std::getline(inFile, line);
    }
    std::getline(inFile, line);

    std::string delimiter = "->";
    std::string space = " ";
    bool last = false;
    index = 0;
    while(true){
        if (index != 0){
            std::string unit;
            if (!last){
                unit = line.substr(0, line.find(delimiter) - 1);
            }
            else{
                unit = line.substr(0, line.size() - 1);
            }

            if (unit == "Right"){
                path.push_back(0);
            }
            if (unit == "Up"){
                path.push_back(1);
            }
            if (unit == "Left"){
                path.push_back(2);
            }
            if (unit == "Down"){
                path.push_back(3);
            }
        }
        index++;
        line = line.substr(line.find(delimiter) + 3, line.size() - line.find(delimiter));

        if (last){
            break;
        }
        if (line.find(space) == std::string::npos){
            last = true;
        }
    }
    generationTime += (double) (clock() - start) / CLOCKS_PER_SEC;

    std::cout << path.size() << ' ';
    printVector(path);

    return path;
}

std::vector<int> AStarMove(int rx, int ry, int N, int M, int goalIndex, std::vector<int> robotListCubes, double& generationTime){
    std::clock_t start = clock();
    int robotIndex = rx + N*ry;
    std::vector<int> path;
    int outputLength;
    
    std::string command= "C:\\ws\\catkin_ws\\src\\assignment3\\src\\MazeNavAStar.exe";

    std::string args;
    args.append(" ");
    args.append(std::to_string(robotIndex));
    args.append(" ");
    args.append(std::to_string(goalIndex));
    args.append(" ");
    args.append(std::to_string(N));
    args.append(" ");
    args.append(std::to_string(M));
    args.append(" ");

    for (int val : robotListCubes){
        args.append(std::to_string(val));
        args.append(" ");
    }

    command.append(args);
    const char* commandCStr = command.c_str();
    std::cout << commandCStr << '\n';
    callCommand(commandCStr);
    
    std::ifstream inFile("C:\\ws\\catkin_ws\\src\\assignment3\\src\\AStarOutput.txt");
    std::string line;

    while (getline(inFile, line)){
        int val = std::stoi(line);
        int robotIndex = ry*N + rx;
        if (val - robotIndex == 1){
            path.push_back(0);
            rx++;
        }
        else if (val - robotIndex == N){
            path.push_back(1);
            ry++;
        }
        else if (val - robotIndex == -1){
            path.push_back(2);
            rx--;
        }
        else if (val - robotIndex == -N){
            path.push_back(3);
            ry--;
        }

    }
    inFile.close();
    std::ofstream outFile("C:\\ws\\catkin_ws\\src\\assignment3\\src\\AStarOutput.txt");
    outFile.close();

    generationTime += (double) (clock() - start) / CLOCKS_PER_SEC;
    return path;
}

int main(int argc, char** args){
    srand(time(0));

    ros::init(argc, args, "assignment3Controller");
    ros::NodeHandle n;
    ros::Rate rate(200);
    ros::ServiceClient deleteModel = n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    ros::ServiceClient getWorldProperties = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
    ros::ServiceClient getModelState = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::ServiceClient setModelState = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::Publisher robotVelocity = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    gazebo_msgs::DeleteModel cube;
    gazebo_msgs::GetWorldProperties world;
    gazebo_msgs::GetModelState getRobotState;   
    gazebo_msgs::SetModelState setRobotState;   
    geometry_msgs::Twist velocity;

    std::vector<int> NRange = {14};
    std::vector<double> densities = {0.2, 0.25, 0.3, 0.35, 0.4};
    int trials = 1;
    std::vector<bool> PATMoves = {false};
    std::ofstream csvFile ("C:\\ws\\catkin_ws\\src\\assignment3\\src\\data.csv");

    const char header[] = "N, density, generationTime, success, pathLength, PAT\n";
    csvFile << header;


    for (int N : NRange){
        for (double density : densities){
            for (bool pat : PATMoves){
                for (int i=0; i<trials; i++){

                    gazebo_msgs::ModelState robotState = initaliseRobotState();
                    setRobotState.request.model_state = robotState;
                    getRobotState.request.model_name = "turtlebot3_burger";

                    std::cout << "Initialising robot\n";
                    //initialiseRobot(velocity, robotVelocity, getModelState, getRobotState, n, rate, getModelState, setRobotState, robotState);
                    std::cout << "Stopping robot\n";
                    stopRobot(velocity, robotVelocity, getModelState, getRobotState, n, rate, 0.005, 0.2);
                    setModelState.call(setRobotState);
                
                    rate.sleep();
                    ros::spinOnce();
                    std::cout << "Stopping robot\n";
                    stopRobot(velocity, robotVelocity, getModelState, getRobotState, n, rate, 0.005, 0.2);
                    clearGrid(getWorldProperties, world, cube, deleteModel, n);

                    std::cout << "Generating grid: \n";
                    
                    int rx = 0;
                    int ry = 0;
                    int nodesTravelled = 0;
                    int goalIndex;
                    std::vector<uint8_t> grid = generateGrid(N, N, density, n, goalIndex);
                    std::vector<uint8_t> robotGrid = buildRobotPerception(grid, N); // 'X', 'R', 'S', 'C', 'E'
                    std::vector<int> robotListCubes;

                    bool goal = false;
                    bool teleport = true;
                    double targetRotation;
                    double robotRotation;
                    double distance;
                    bool valid;
                    bool first;
                    geometry_msgs::Point currentPosition;
                    geometry_msgs::Point targetPosition;
                    double generationTime = 0;
                    double PATTime = 0;

                    while(!goal && ros::ok()){      
                        robotListCubes = getCubeList(robotGrid);
                        std::vector<int> path;
                        printVector(robotListCubes);
                        if (pat){
                            path = PATMove(rx, ry, N, N, goalIndex, robotListCubes, generationTime);  
                        }
                        else {
                            path = AStarMove(rx, ry, N, N, goalIndex, robotListCubes, generationTime); 
                        }
                        

                        valid = true; 
                        first = true; 

                        while (valid && path.size() > 0){
                            int move = decideMove(rx, ry, robotGrid, N, goalIndex, robotListCubes, path, valid);

                            std::cout << valid << '\n';
                        
                            if (valid){
                                first = false;
                                getStateInformation(getModelState, getRobotState, n, rate, rx, ry, N, move, robotRotation, currentPosition, targetPosition);
                                targetRotation = getTargetRotation(currentPosition, targetPosition, distance);

                                if (!teleport){
                                    orientRobot(robotRotation, targetRotation, velocity, robotVelocity, getModelState, getRobotState, n, rate, 0.02, 0.004);
                                    moveRobot(currentPosition, targetPosition, velocity, robotVelocity, getModelState, getRobotState, n, rate, 0.1);
                                    stopRobot(velocity, robotVelocity, getModelState, getRobotState, n, rate, 0.001, 0.02);
                                }
                                else {
                                    robotState = initaliseRobotState(rx, ry);
                                    setRobotState.request.model_state = robotState;
                                    setModelState.call(setRobotState);
                                }
                                
                                nodesTravelled++;
                                updateGrid(rx, ry, move, grid, robotGrid, N);
                                checkGoal(rx, ry, goalIndex, goal, N);

                                rebuildRobotPerception(rx, ry, grid, robotGrid, N); // 'Xnknown', 'Robot', 'Space', 'Cube', 'Explored', 'Goal'
                            }
                            if (!valid && first){
                                std::cout << "Goal not possible\n";
                                path.resize(0);
                                break;
                            }
                        }
                        if (path.size() == 0 && !goal){
                            break;
                        }               
                    }
                csvFile << N << ',' << density << ',' << generationTime << ',' << goal << ',' << nodesTravelled << ',' << pat << '\n';
                
                    if (goal){
                        std::cout << "Goal found\n";
                    }
                    if (!goal){
                        std::cout << "Goal not possible\n";
                    }
                }
            }
        }
    }
    csvFile.close();

    return EXIT_SUCCESS;
}