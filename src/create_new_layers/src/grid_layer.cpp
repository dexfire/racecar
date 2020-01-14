#include <create_new_layers/grid_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;
using namespace std;
namespace simple_layer_namespace
{
 
unsigned flag = 0;
int num=0 ;

GridLayer::GridLayer() {}
 
void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();
  //obstacle_sub = nh.subscribe("/obstacle", 1, &GridLayer::obstacleCB, this);
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}
 
 
void GridLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}
 
 
void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}
//void GridLayer::obstacleCB(const geometry_msgs::Point pointMsg)
//{
    //obstacle_x = pointMsg.x;
    //obstacle_y = pointMsg.y;
//}
void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  
  if (!enabled_)
    return;
  
 //  if (flag == 0)
 //  {
	//   flag = 1;
 //  }else
	// return;
 
  mark_x = robot_x , mark_y = robot_y ;
  //double mark_x_se , mark_y_se;
  // mark_x =  4.0; 
  // //mark_x = obstacle_x;
  // mark_y = -0.25;
  // //mark_y = obstacle_y;
  // mark_x_se = 4.0;
  // mark_y_se = -0.75;
  //if (mark_y>-0.2&&mark_y<0.2){
//mark_y = -0.5;
//}
//else if(mark_y>-2.2&&mark_y<-1.8){
//mark_y = -2.5;
//}
//else{
//mark_y = mark_y;
// //}
//   unsigned int mx;
//   unsigned int my;
//   if(worldToMap(mark_x, mark_y, mx, my)){
// 	       setCost(mx, my,LETHAL_OBSTACLE);
//   }
  
  // *min_x = std::min(*min_x, mark_x);
  // *min_y = std::min(*min_y, mark_y);
  // *max_x = std::max(*max_x, mark_x);
  // *max_y = std::max(*max_y, mark_y);

   *min_x = *min_x -1;
   *min_y = *min_y -1;
   *max_x = *max_x +1;
   *max_y = *max_y +1;

//   if(worldToMap(mark_x_se, mark_y_se, mx, my)){
// 	       setCost(mx, my,LETHAL_OBSTACLE);
//   }
  
//   *min_x = std::min(*min_x, mark_x);
//   *min_y = std::min(*min_y, mark_y);
//   *max_x = std::max(*max_x, mark_x);
//   *max_y = std::max(*max_y, mark_y);
 }

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
 // fstream x_file;
 // fstream y_file;
 // x_file.open("x.txt",ios::out | ios::app | ios::binary);
 
 // y_file.open("y.txt",ios::out | ios::app | ios::binary);
 double wx;
 double wy;
 double wx_one;
 double wy_one;
 double wx_two;
 double wy_two;
 double wx_three;
 double wy_three;
 double wx_four;
 double wy_four;
 unsigned int mx;
 unsigned int my;
 unsigned int mxx[100000];
 unsigned int myy[100000];
 unsigned char* master_array = master_grid.getCharMap();
 //for(int k)
 // distance_oringe = mark_x*mark_x + mark_y*mark_y;
 // distance_goal = mark_x*mark_x + (mark_y+2)*(mark_y+2) ;
 // if (distance_oringe < 1)
 // {
 // 	for (int k=0; k<num; k++){
 //         x_file >> mxx[k] ;
 //         cout<< mxx[k] <<endl;
 //         y_file >> myy[k] ;
 //         cout<< myy[k] << endl;
 //         distance_goal = 1.5;
 //         }
 // }
 // if (distance_goal < 1)
 // {
 // 	for (int k=0; k<num; k++){
 		
 // 	 x_file << mxx[k] << endl;
 //     y_file << myy[k] << endl;
 // 	}
 	 
 //     distance_goal = 1.5;
 // }
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE)
      { 
        mxx[num] = i;
        myy[num] = j;
        num ++;
        if (num>99999)
        	num=99999;
        mapToWorld(i,j,wx,wy);
        if (wx>0&&wx<7.5&&wy>-0.22&&wy<0.22)
        { 

              wy_one = wy+0.25;
              wy_two = wy+0.4;
              wx_three = wx + 0.4;
              wx_four = wx - 0.4;
              if(master_grid.worldToMap(wx, wy_one, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
              if(master_grid.worldToMap(wx, wy_two, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
              if(master_grid.worldToMap(wx_three, wy-0.11, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
              if(master_grid.worldToMap(wx_four, wy-0.11, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
 
        }
        
        else if (wx>0&&wx<7.5&&wy>-2.4&&wy<-1.8)
        { 

              wy_one = wy-0.25;
              wy_two = wy-0.4;
              wx_three = wx + 0.2;
              wx_four = wx - 0.2;
              if(master_grid.worldToMap(wx, wy_one, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
              if(master_grid.worldToMap(wx, wy_two, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
              if(master_grid.worldToMap(wx_three, wy+0.1, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
              if(master_grid.worldToMap(wx_four, wy+0.1, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
        }
        else if (wx>11&&wx<14.5&&wy>-0.25&&wy<0.25)
        { 

              wy_one = wy+0.25;
              wy_two = wy+0.4;
              wx_three = wx + 0.4;
              wx_four = wx - 0.4;
              if(master_grid.worldToMap(wx, wy_one, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
              if(master_grid.worldToMap(wx, wy_two, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
              if(master_grid.worldToMap(wx_three, wy-0.15, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
              if(master_grid.worldToMap(wx_four, wy-0.15, mx, my)){
	                  master_grid.setCost(mx, my,LETHAL_OBSTACLE);
              }
        }
        // else if (wx>11&&wx<14.5&&wy>-2.2&&wy<-1.8)
        // { 

        //       wy_one = wy-0.25;
        //       wy_two = wy-0.4;
        //       wx_three = wx + 0.4;
        //       wx_four = wx - 0.4;
        //       if(master_grid.worldToMap(wx, wy_one, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        //       if(master_grid.worldToMap(wx, wy_two, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        //       if(master_grid.worldToMap(wx_three, wy-0.25, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        //       if(master_grid.worldToMap(wx_four, wy-0.25, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        // }
        //   else if (wx>22&&wx<26&&wy>-0.22&&wy<0.22)
        // { 

        //       wy_one = wy-0.25;
        //       wy_two = wy-0.4;
        //       wx_three = wx + 0.4;
        //       wx_four = wx - 0.4;
        //       if(master_grid.worldToMap(wx, wy_one, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        //       if(master_grid.worldToMap(wx, wy_two, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        //       if(master_grid.worldToMap(wx_three, wy-0.25, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        //       if(master_grid.worldToMap(wx_four, wy-0.25, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        // }
        //  else if (wx>22&&wx<26&&wy>-2.2&&wy<-1.8)
        // { 

        //       wy_one = wy-0.25;
        //       wy_two = wy-0.4;
        //       wx_three = wx + 0.4;
        //       wx_four = wx - 0.4;
        //       if(master_grid.worldToMap(wx, wy_one, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        //       if(master_grid.worldToMap(wx, wy_two, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        //       if(master_grid.worldToMap(wx_three, wy-0.25, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        //       if(master_grid.worldToMap(wx_four, wy-0.25, mx, my)){
	       //            master_grid.setCost(mx, my,LETHAL_OBSTACLE);
        //       }
        // }
      }
    }
  }
}
 
} // end namespace

