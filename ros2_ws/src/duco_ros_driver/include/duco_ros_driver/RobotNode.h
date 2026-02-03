#ifndef ROBOTNODE_H
#define ROBOTNODE_H
#include <memory>
#include <string>
namespace DucoRPC
{
class DucoCobot;
};

class RobotNode
{
public:
#define PORT 7003

   RobotNode(const std::string &ip);

   ~RobotNode();
   std::shared_ptr<DucoRPC::DucoCobot> duco_robot=nullptr;

   /*主机IP地址*/

   bool is_connected=false;

   bool connect();

   bool disconnect();

private:
   std::string server_host="127.0.0.1";

};



#endif /* DUCODRIVER_H */

