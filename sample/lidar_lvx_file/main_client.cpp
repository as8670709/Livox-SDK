#include <string>
#include <iostream>
#include <ctime>
#include "buttonrpc.hpp"
#include "lvx_file.h"
#ifdef _WIN32
#include <Windows.h>  // use sleep
#else
 #include <unistd.h>
#endif
#include <iostream>  
#include <thread>  

// using namespace std;  

DeviceItem devices[kMaxLidarCount];
LvxFileHandle lvx_file_handler;
std::list<LvxBasePackDetail> point_packet_list;
std::vector<std::string> broadcast_code_rev;
std::condition_variable lidar_arrive_condition;
std::condition_variable extrinsic_condition;
std::condition_variable point_pack_condition;
std::mutex mtx;
int lvx_file_save_time = 10;
bool is_finish_extrinsic_parameter = false;
bool is_read_extrinsic_from_xml = false;
uint8_t connected_lidar_count = 0;
int Continue = 1;
#define FRAME_RATE 10


using namespace std::chrono;

int WriteLvxFile()
{
	printf("Start initialize lvx file.\n");
  if (!lvx_file_handler.InitLvxFile()) {
    // Uninit();
    return -1;
  }

  lvx_file_handler.InitLvxFileHeader();

  int i = 0;
  steady_clock::time_point last_time = steady_clock::now();
  for (i = 0; i < lvx_file_save_time * FRAME_RATE; ++i) {
    std::list<LvxBasePackDetail> point_packet_list_temp;
    {
      std::unique_lock<std::mutex> lock(mtx);
      point_pack_condition.wait_for(lock, milliseconds(kDefaultFrameDurationTime) - (steady_clock::now() - last_time));
      last_time = steady_clock::now();
      point_packet_list_temp.swap(point_packet_list);
    }
    if(point_packet_list_temp.empty()) {
      printf("Point cloud packet is empty.\n");
      break;
    }

    printf("Finish save %d frame to lvx file.\n", i);
    lvx_file_handler.SaveFrameToLvxFile(point_packet_list_temp);
  }

  lvx_file_handler.CloseLvxFile();


  Continue = 0;


}


/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(string ip) {
	buttonrpc client;
	client.as_client(ip, 3555);
	client.set_timeout(2000);

	while(Continue){	
		LvxBasePackDetail res = client.call<LvxBasePackDetail>("GetData").val();
		{
			std::unique_lock<std::mutex> lock(mtx);      
			point_packet_list.push_back(res);
		}
	}
}

int main()
{
	string ip = "127.0.0.1";
	thread get_data(GetLidarData,ip);	
	sleep(1);
	WriteLvxFile();
	printf("write over.\n");
	get_data.join();
	return 0;
}