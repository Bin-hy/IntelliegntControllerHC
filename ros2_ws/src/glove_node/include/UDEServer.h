#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netdb.h>
#include <net/if.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include <thread>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <jsoncpp/json/json.h>

using namespace std;


struct Parameter 
{
	string name;
	float value;
};

struct Bone
{
	string name;
	int parent = 0;
	vector<float> location = {};
	vector<float> rotation = {};
	vector<float> scale = {};	
};

struct handsfingerJoint
{
	vector<Parameter> fingerJoint_L;
	vector<Parameter> fingerJoint_R;
};

struct ControllerValue
{
    vector<Parameter> JoyValue_L;
	vector<Parameter> JoyValue_R;
    vector<Parameter> ButtonValue_L;
	vector<Parameter> ButtonValue_R;
};

struct HandData
{
	Bone bones;
	handsfingerJoint fingerJoints;
};

struct GloveData
{
	string roleName;
	HandData handDatas;
};

struct GloveControllerData
{
    string roleName;
	ControllerValue controllerDatas;
};

struct Vector3Float
{
    float x;
    float y;
    float z;

    Vector3Float(float X, float Y, float Z)
    {
        this->x = X;
        this->y = Y;
        this->z = Z;
    }
};

enum Fingers
{
    Thumb1,Thumb2,Thumb3,
    Index1,Index2,Index3,
    Middle1,Middle2,Middle3,
    Ring1,Ring2,Ring3,
    Pinky1,Pinky2,Pinky3,
};

enum HandJointIndex
{
    Thumb1X = 3,
    Thumb1Y = 4,
    Thumb1Z = 21,
    Thumb2X = 2,
    Thumb3X = 1,

    Index1X = 7,
    Index1Y = 8,
    Index1Z = 22,
    Index2X = 6,
    Index3X = 5,

    Middle1X = 11,
    Middle1Y = 12,
    Middle2X = 10,
    Middle3X = 9,

    Ring1X = 15,
    Ring1Y = 16,
    Ring2X = 14,
    Ring3X = 13,

    Pinky1X = 19,
    Pinky1Y = 20,
    Pinky1Z = 23,
    Pinky2X = 18,
    Pinky3X = 17,
};

enum ServerStatus
{
    NO_INIT, READY, IN_LISTENING, END
};

static string GloveDataHeaders[30] = 
{
    "LeftThumb1", 
    "LeftThumb2", 
    "LeftThumb3", 
    "LeftIndex1", 
    "LeftIndex2", 
    "LeftIndex3", 
    "LeftMiddle1",
    "LeftMiddle2",
    "LeftMiddle3",
    "LeftRing1",
    "LeftRing2",
    "LeftRing3",
    "LeftPinky1", 
    "LeftPinky2", 
    "LeftPinky3", 
    "RightThumb1", 
    "RightThumb2", 
    "RightThumb3", 
    "RightIndex1", 
    "RightIndex2", 
    "RightIndex3",
    "RightMiddle1",
    "RightMiddle2",
    "RightMiddle3",
    "RightRing1",
    "RightRing2",
    "RightRing3",
    "RightPinky1", 
    "RightPinky2", 
    "RightPinky3", 
};

static string ControllerHeaders[12] = 
{
    "Left Joy X",
    "Left Joy Y",
    "Left A Button",
    "Left B Button",
    "Left Joy Button",
    "Left Menu Button",
    "Right Joy X",
    "Right Joy Y",
    "Right A Button",
    "Right B Button",
    "Right Joy Button",
    "Right Menu Button"
};

class UDEGloveSDK
{
    public:
        int Initialize();
        void SetPortNum(int port);
        int GetPortNum() { return Port; };
        void StartListening();
        void EndListening();
        ServerStatus GetStatus();

        vector<string> GetRoleNameList();
        vector<Vector3Float> GetVecFingerData(string RoleName);
        float* GetVecControllerData(string RoleName);
        
        map<string, vector<float>> GetFingerData(string RoleName);
        map<string, float> GetControllerData(string RoleName);

    private:
        int sock_fd;
        int Port = 5555;
        vector<string> NameList;
        float* ControllerRes = new float[12];
        vector<Vector3Float> GloveVecRes;
        ServerStatus CurStatus = ServerStatus::NO_INIT;
        std::thread recv_t;
        sockaddr_in server_addr;
        void static recv_func(int sock_fd_);
        static vector<GloveData>gloveDataList;
        static vector<GloveControllerData>gloveControllerDataList;
        
        map<string, vector<float>> GloveRes;
        map<string, float> ControllerMapRes;

        //string Send_IP = "127.0.0.1";
        //sockaddr_in client_addr;
        //void static send_func(int sock_fd_);
};
