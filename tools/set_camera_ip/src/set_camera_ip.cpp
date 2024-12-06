#include <iostream>
#include <time.h>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <tof.hpp>

bool validate_ip(std::string ip, IPAddress* ip_address)
{
  std::vector<int> ip_vect;
  std::stringstream ip_ss(ip);
  for (int i; ip_ss >> i;) {
      ip_vect.push_back(i);    
      if (ip_ss.peek() == '.')
          ip_ss.ignore();
  }

  if (ip_vect.size() != 4)
  {
    return false;
  }

  bool is_valid = true;
  for (std::size_t i = 0; i < ip_vect.size(); i++)
  {
      if (ip_vect[i] >= 0 && ip_vect[i] <= 255)
      {
          //
      } else 
      {
          is_valid = false;
          break;
      }
  }
  if (! is_valid)
  {
    return false;
  }

  ip_address->ip0 = ip_vect[0];
  ip_address->ip1 = ip_vect[1];
  ip_address->ip2 = ip_vect[2];
  ip_address->ip3 = ip_vect[3];

  return true;
}
int main(int argc, char** argv) {

  if (argc != 5)
  {
    std::cout << "Usage:   ./set_camera_ip <current IP address> <new IP address> <new subnet mask> <new gateway>" << std::endl;
    std::cout << "Example: ./set_camera_ip 10.10.10.180 10.10.31.180 255.255.255.0 192.168.0.100" << std::endl;
    return 1;
  }
  std::string cur_ip(argv[1]);
  std::string new_ip(argv[2]);
  std::string n_mask(argv[3]);
  std::string gateway(argv[4]);

  IPAddress new_ip_address(0, 0, 0, 0);
  if (! validate_ip(new_ip, &new_ip_address))
  {
    std::cout << "invalid new IP address: " << new_ip << std::endl;
    return 1;
  }

  IPAddress n_mask_addr(0, 0, 0, 0);
  if (! validate_ip(n_mask, &n_mask_addr))
  {
    std::cout << "invalid subnet mask: " << n_mask << std::endl;
    return 1;
  }

  IPAddress gateway_address(0, 0, 0, 0);
  if (! validate_ip(gateway, &gateway_address))
  {
    std::cout << "invalid gateway: " << gateway << std::endl;
    return 1;
  }

  std::cout << "Connecting to LiMu camera at IP address: " << cur_ip << std::endl;

  ToF* tof = ToF::tof320(cur_ip.c_str(), "50660");

  std::cout << "Camera connected at IP address: " << cur_ip << std::endl;

  usleep(2000000);

  std::cout << "Set camera new IP address: " << (int) new_ip_address.ip0 << "." << (int) new_ip_address.ip1 << "." << (int) new_ip_address.ip2 << "." << (int) new_ip_address.ip3 << std::endl;

  tof->setIPAddress(new_ip_address, n_mask_addr, gateway_address);

  usleep(2000000);

  delete tof;

  std::cout << "LiMu camera changed to: \n\n" << std::endl;

  std::cout << "IP address:      " << (int) new_ip_address.ip0 << "." << (int) new_ip_address.ip1 << "." << (int) new_ip_address.ip2 << "." << (int) new_ip_address.ip3 << std::endl;
  std::cout << "Subnet Mask:     " << (int) n_mask_addr.ip0 << "." << (int) n_mask_addr.ip1 << "." << (int) n_mask_addr.ip2 << "." << (int) n_mask_addr.ip3 << std::endl;
  std::cout << "Default Gateway: " << (int) gateway_address.ip0 << "." << (int) gateway_address.ip1 << "." << (int) gateway_address.ip2 << "." << (int) gateway_address.ip3 << std::endl;

  std::cout << "\n" << std::endl;

	std::cout << "Please power off the camera and wait for a couple of minutes, \n" 
            << "then power on the camera again to connect to the camera at the new IP address \n\n" 
            << (int) new_ip_address.ip0 << "." << (int) new_ip_address.ip1 << "." << (int) new_ip_address.ip2 << "." << (int) new_ip_address.ip3 
            <<  "\n\n" << std::endl;

	return 0;
}
