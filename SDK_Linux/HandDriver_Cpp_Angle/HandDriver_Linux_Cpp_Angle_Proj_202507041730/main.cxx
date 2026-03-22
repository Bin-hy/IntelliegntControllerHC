#include "UDEServer.h"

int main(int agrc, char** argv)
{
    UDEGloveSDK sdk;
    sdk.Initialize();

    sdk.StartListening();
    cout << "Server Start Listening.." << endl;
    int index = 1;
    Json::Value::Members list;
    bool quit = false;

    while (!quit)
    {
        try
        {
            list = sdk.GetRoleNameList();
            if (list.size() > 0)
            {
                auto FingerData = sdk.GetVecFingerData(list[0]);
                auto ControllerMapData = sdk.GetVecControllerData(list[0]);
                if (sdk.GetStatus() != 2) continue;
                cout << "--------------" << index++ << "-----------" << endl;
                for (int i = 0; i < (int)sizeof(GloveDataHeaders)/sizeof(GloveDataHeaders[0]); ++i)
                {
                    cout << GloveDataHeaders[i] << ": (" << FingerData[i].x << "," << FingerData[i].y << "," << FingerData[i].z << ")" << endl;
                }
                for(int i = 0; i < (int)sizeof(ControllerHeaders)/sizeof(ControllerHeaders[0]); ++i)
                {
                    cout << ControllerHeaders[i] << ": " << ControllerMapData[i] << endl;
                }
                cout << "-------------------------" << endl;

                list.clear();
                //Terminate server after listening for 20min
                /*if (index >= 7200 * 20)
                {
                    cout << "End Listening." << index << endl;
                    sdk.EndListening();
                    quit = true;
                    break;
                }*/
            }
            usleep(1000000 / 120); //receiving rate about 120hz
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    return 0;
}

// build command:
// sudo sh ./build.sh