#define _GLIBCXX_USE_NANOSLEEP
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include <cstdio>

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

std::vector<int> PATMove(int rx, int ry, int N, int M, int goalIndex, std::vector<int> robotListCubes, double& generationTime, double& PATTime){
    std::clock_t start = clock();
    int robotIndex = rx + N*ry;
    std::vector<int> path;
    int outputLength;
    
    std::string inputPAT;
    std::ofstream outFile("mazeNavHeader.csp");
    std::ifstream skeleton("mazeNavSkeleton.csp");

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
    
    char command[] = "PAT3.Console.exe -csp -engine 0 C:/ws/catkin_ws/src/assignment3/src/mazeNavHeader.csp C:/ws/catkin_ws/src/assignment3/src/outputPAT.txt";
    std::cout << command << '\n';
    callCommand(command);
    
    std::ifstream inFile("outputPAT.txt");
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

    for (int i=0; i<10; i++){
        std::getline(inFile, line);
    }
    std::getline(inFile, line);
    line = line.substr(10,line.size() - 11);
    std::cout << line << '\n';

    PATTime += std::stod(line);
    generationTime += (double) (clock() - start) / CLOCKS_PER_SEC;
    return path;
}

std::vector<int> AStarMove(int rx, int ry, int N, int M, int goalIndex, std::vector<int> robotListCubes, double& generationTime, double& PATTime){
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
    generationTime += (double) (clock() - start) / CLOCKS_PER_SEC;
    
    return path;
}

int main(int argc, char** args){
    std::vector<int> robotListCubes = {};
    double generationTime = 0;
    double PATTime = 0;
    std::vector<int> path = AStarMove(0, 0, 9, 9, 80, robotListCubes, generationTime, PATTime);

    for (int val : path){
        std::cout << val << ' ';
    }
    std::cout << '\n' << PATTime << ' ' << generationTime << '\n';

    return 0;
}