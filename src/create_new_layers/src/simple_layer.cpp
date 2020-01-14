#include<create_new_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
 
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
 
namespace simple_layer_namespace
{
 
SimpleLayer::SimpleLayer() {}
 
void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
 
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}
 
 
void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}
 
void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
 
  // mark_x_ = robot_x + cos(robot_yaw);
  // mark_y_ = robot_y + sin(robot_yaw);
   mark_x_[0] = 4;
   mark_y_[0]=  0;
   mark_x_[1] = 4;
   mark_y_[1]= 0;
   mark_x_[2] = 26.3;
   mark_y_[2]= -1.7;
   mark_x_[3] = 4.1;
   mark_y_[3]= -1.3;
   mark_x_[4] = 4.1;
   mark_y_[4]= -1.4;
  // mark_x_[5] = 26.8;
  // mark_y_[5]= -0.4;
  // mark_x_[6] = 26.8;
  // mark_y_[6]= -0.4;
  for (int i=0;i<1;i++){
  *min_x = std::min(*min_x, mark_x_[i]);
  *min_y = std::min(*min_y, mark_y_[i]);
  *max_x = std::max(*max_x, mark_x_[i]);
  *max_y = std::max(*max_y, mark_y_[i]);
  }
  
  
}
 
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  for (int i=0;i<1;i++){
	   if(master_grid.worldToMap(mark_x_[i], mark_y_[i], mx, my)){
	    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
	  }
  }
 
}
 
} // end namespace

