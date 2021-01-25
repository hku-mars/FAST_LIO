#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

// Feature will be updated in next version

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;

ros::Publisher pub_full, pub_surf, pub_corn;

enum LID_TYPE{MID, HORIZON, VELO16, OUST64};

enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct orgtype
{
  double range;
  double dista; 
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

const double rad2deg = 180*M_1_PI;

int lidar_type;
double blind, inf_bound;
int N_SCANS;
int group_size;
double disA, disB;
double limit_maxmid, limit_midmin, limit_maxmin;
double p2l_ratio;
double jump_up_limit, jump_down_limit;
double cos160;
double edgea, edgeb;
double smallp_intersect, smallp_ratio;
int point_filter_num;

void mid_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void horizon_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
void velo16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types, pcl::PointCloud<PointType> &pl_corn, pcl::PointCloud<PointType> &pl_surf);
void pub_func(pcl::PointCloud<PointType> &pl, ros::Publisher pub, const ros::Time &ct);
int plane_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
bool small_plane(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
bool edge_jump_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, Surround nor_dir);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_extract");
  ros::NodeHandle n;

  n.param<int>("lidar_type", lidar_type, 0);
  n.param<double>("blind", blind, 0.5);
  n.param<double>("inf_bound", inf_bound, 10);
  n.param<int>("N_SCANS", N_SCANS, 1);
  n.param<int>("group_size", group_size, 8);
  n.param<double>("disA", disA, 0.01);
  n.param<double>("disB", disB, 0.1);
  n.param<double>("p2l_ratio", p2l_ratio, 400);
  n.param<double>("limit_maxmid", limit_maxmid, 9);
  n.param<double>("limit_midmin", limit_midmin, 16);
  n.param<double>("limit_maxmin", limit_maxmin, 3.24);
  n.param<double>("jump_up_limit", jump_up_limit, 175.0);
  n.param<double>("jump_down_limit", jump_down_limit, 5.0);
  n.param<double>("cos160", cos160, 160.0);
  n.param<double>("edgea", edgea, 3);
  n.param<double>("edgeb", edgeb, 0.05);
  n.param<double>("smallp_intersect", smallp_intersect, 170.0);
  n.param<double>("smallp_ratio", smallp_ratio, 1.2);
  n.param<int>("point_filter_num", point_filter_num, 4);

  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);

  ros::Subscriber sub_points;
  
  switch(lidar_type)
  {
  case MID:
    printf("MID40\n");
    sub_points = n.subscribe("/livox/lidar", 1000, mid_handler);
    // sub_points = n.subscribe("/livox/lidar_1LVDG1S006J5GZ3", 1000, mid_handler);
    break;

  case HORIZON:
    printf("HORIZON\n");
    sub_points = n.subscribe("/livox/lidar", 1000, horizon_handler);
    break;

  case VELO16:
    printf("VELO16\n");
    sub_points = n.subscribe("/velodyne_points", 1000, velo16_handler);
    break;

  case OUST64:
    printf("OUST64\n");
    sub_points = n.subscribe("/os1_cloud_node/points", 1000, oust64_handler);
    break;
  
  default:
    printf("Lidar type is wrong.\n");
    exit(0);
    break;
  }

  pub_full = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 100);
  pub_surf = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
  pub_corn = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

  ros::spin();
  return 0;
}


double vx, vy, vz;
void mid_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<PointType> pl;
  pcl::fromROSMsg(*msg, pl);

  pcl::PointCloud<PointType> pl_corn, pl_surf;
  vector<orgtype> types;
  uint plsize = pl.size()-1;
  pl_corn.reserve(plsize); pl_surf.reserve(plsize);
  types.resize(plsize+1);

  for(uint i=0; i<plsize; i++)
  {
    types[i].range = pl[i].x;
    vx = pl[i].x - pl[i+1].x;
    vy = pl[i].y - pl[i+1].y;
    vz = pl[i].z - pl[i+1].z;
    types[i].dista = vx*vx + vy*vy + vz*vz;
  }
  // plsize++;
  types[plsize].range = sqrt(pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y);

  give_feature(pl, types, pl_corn, pl_surf);

  ros::Time ct(ros::Time::now());
  pub_func(pl, pub_full, msg->header.stamp);
  pub_func(pl_surf, pub_surf, msg->header.stamp);
  pub_func(pl_corn, pub_corn, msg->header.stamp);
}

void horizon_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  double t1 = omp_get_wtime();
  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);
  pcl::PointCloud<PointType> pl_full, pl_corn, pl_surf;

  uint plsize = msg->point_num;

  pl_corn.reserve(plsize); pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].reserve(plsize);
  }
  
  for(uint i=1; i<plsize; i++)
  {
    if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10)
        && (!IS_VALID(msg->points[i].x)) && (!IS_VALID(msg->points[i].y)) && (!IS_VALID(msg->points[i].z)))
    {
      pl_full[i].x = msg->points[i].x;
      pl_full[i].y = msg->points[i].y;
      pl_full[i].z = msg->points[i].z;
      pl_full[i].intensity = msg->points[i].reflectivity;
      pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

      if((std::abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
          || (std::abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
          || (std::abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
      {
        pl_buff[msg->points[i].line].push_back(pl_full[i]);
      }
    }
  }

  if(pl_buff[0].size() <= 7) {return;}

  for(int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pl = pl_buff[j];
    vector<orgtype> &types = typess[j];
    plsize = pl.size();
    types.resize(plsize);
    plsize--;
    for(uint i=0; i<plsize; i++)
    {

      types[i].range = sqrt(pl[i].x*pl[i].x + pl[i].y*pl[i].y);
      vx = pl[i].x - pl[i+1].x;
      vy = pl[i].y - pl[i+1].y;
      vz = pl[i].z - pl[i+1].z;
      types[i].dista = vx*vx + vy*vy + vz*vz;
      // std::cout<<vx<<" "<<vx<<" "<<vz<<" "<<std::endl;
    }
    // plsize++;
    types[plsize].range = sqrt(pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y);

    give_feature(pl, types, pl_corn, pl_surf);
  }

  ros::Time ct;
  ct.fromNSec(msg->timebase);
  pub_func(pl_full, pub_full, msg->header.stamp);
  pub_func(pl_surf, pub_surf, msg->header.stamp);
  pub_func(pl_corn, pub_corn, msg->header.stamp);
  std::cout<<"[~~~~~~~~~~~~ Feature Extract ]: time: "<< omp_get_wtime() - t1<<" "<<msg->header.stamp.toSec()<<std::endl;
}

int orders[16] = {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15};

void velo16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<PointType> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  uint plsize = pl_orig.size();

  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);
  pcl::PointCloud<PointType> pl_corn, pl_surf, pl_full;

  int scanID;
  int last_stat = -1;
  int idx = 0;

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].resize(plsize);
    typess[i].resize(plsize);
  }

  for(uint i=0; i<plsize; i++)
  {
    PointType &ap = pl_orig[i];
    double leng = sqrt(ap.x*ap.x + ap.y*ap.y);
    if(leng < blind)
    {
      continue;
    }

    double ang = atan(ap.z / leng)*rad2deg;
    scanID = int((ang + 15) / 2 + 0.5);

    if(scanID>=N_SCANS || scanID<0)
    {
      continue;
    }

    if(orders[scanID] <= last_stat)
    {
      idx++;
    }

    pl_buff[scanID][idx].x = ap.x;
    pl_buff[scanID][idx].y = ap.y;
    pl_buff[scanID][idx].z = ap.z;
    pl_buff[scanID][idx].intensity = ap.intensity;
    typess[scanID][idx].range = leng;
    last_stat = orders[scanID];
  }

  // for(int i=0; i<N_SCANS; i++)
  // {
  //   pl_full += pl_buff[i];
  //   pub_func(pl_buff[i], pub_full, msg->header.stamp);
  // }
  

  // for(int i=0; i<10; i++)
  // {
  //   printf("%f %f %f\n", pl_buff[0][i].x, pl_buff[0][i].y, pl_buff[0][i].z);
  // }
  // cout << endl;

  idx++;

  for(int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pl = pl_buff[j];
    vector<orgtype> &types = typess[j];
    pl.erase(pl.begin()+idx, pl.end());
    types.erase(types.begin()+idx, types.end());
    plsize = idx - 1;
    for(uint i=0; i<plsize; i++)
    {
      // types[i].range = sqrt(pl[i].x*pl[i].x + pl[i].y*pl[i].y);
      vx = pl[i].x - pl[i+1].x;
      vy = pl[i].y - pl[i+1].y;
      vz = pl[i].z - pl[i+1].z;
      types[i].dista = vx*vx + vy*vy + vz*vz;
    }

    types[plsize].range = sqrt(pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y);
    
    give_feature(pl, types, pl_corn, pl_surf);
  }

  // printf("%zu %zu\n", pl_surf.size(), pl_corn.size());

  pub_func(pl_orig, pub_full, msg->header.stamp);
  pub_func(pl_surf, pub_surf, msg->header.stamp);
  pub_func(pl_corn, pub_corn, msg->header.stamp);

}


void velo16_handler1(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<PointType> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  pcl::PointCloud<PointType> pl_corn, pl_surf, pl_full;
  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);

  uint plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.reserve(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].resize(plsize);
    typess[i].resize(plsize);
  }

  int idx = -1;
  int stat = 0; // 0代表上一次为0
  int scanID = 0;

  for(uint i=0; i<plsize; i++)
  {
    PointType &ap = pl_orig[i];
    double leng = sqrt(ap.x*ap.x + ap.y*ap.y);
    
    if(leng > blind)
    {
      if(stat == 0)
      {
        stat = 1;
        idx++;
      }

      double ang = atan(ap.z / leng)*rad2deg;
      scanID = int((ang + 15) / 2 + 0.5);
      if(scanID>=N_SCANS || scanID <0)
      {
        continue;
      }
      pl_buff[scanID][idx] = ap;
      typess[scanID][idx].range = leng;
      pl_full.push_back(ap);
    }
    else
    {
      stat = 0;
    }
  }

  // idx = 0;
  // int last_stat = -1;
  // for(uint i=0; i<plsize; i++)
  // {
  //   PointType &ap = pl_orig[i];
  //   // pl_full.push_back(ap);
  //   double leng = sqrt(ap.x*ap.x + ap.y*ap.y);
  //   if(leng < blind)
  //   {
  //     continue;
  //   }

  //   double ang = atan(ap.z / leng)*rad2deg;
  //   scanID = int((ang + 15) / 2 + 0.5);

  //   if(scanID>=N_SCANS || scanID<0)
  //   {
  //     continue;
  //   }

  //   if(orders[scanID] <= last_stat)
  //   {
  //     idx++;
  //   }

  //   pl_buff[scanID][idx].x = ap.x;
  //   pl_buff[scanID][idx].y = ap.y;
  //   pl_buff[scanID][idx].z = ap.z;
  //   pl_buff[scanID][idx].intensity = ap.intensity;

  //   last_stat = orders[scanID];
  // }

  idx++;

  for(int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pl = pl_buff[j];
    vector<orgtype> &types = typess[j];
    pl.erase(pl.begin()+idx, pl.end());
    types.erase(types.begin()+idx, types.end());
    plsize = idx - 1;
    for(uint i=0; i<plsize; i++)
    {
      // types[i].range = sqrt(pl[i].x*pl[i].x + pl[i].y*pl[i].y);
      vx = pl[i].x - pl[i+1].x;
      vy = pl[i].y - pl[i+1].y;
      vz = pl[i].z - pl[i+1].z;
      types[i].dista = vx*vx + vy*vy + vz*vz;
    }
    types[plsize].range = sqrt(pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y);
    
    
    give_feature(pl, types, pl_corn, pl_surf);
  }

  pub_func(pl_full, pub_full, msg->header.stamp);
  pub_func(pl_surf, pub_surf, msg->header.stamp);
  pub_func(pl_corn, pub_corn, msg->header.stamp);
}

void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<PointType> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  
  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);
  pcl::PointCloud<PointType> pl_corn, pl_surf;

  uint plsize = pl_orig.size();

  pl_corn.reserve(plsize); pl_surf.reserve(plsize);
  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].reserve(plsize);
  }

  for(uint i=0; i<plsize; i+=N_SCANS)
  {
    for(int j=0; j<N_SCANS; j++)
    {
      pl_buff[j].push_back(pl_orig[i+j]);
    }
  }

  for(int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pl = pl_buff[j];
    vector<orgtype> &types = typess[j];
    plsize = pl.size() - 1;
    types.resize(plsize+1);
    for(uint i=0; i<plsize; i++)
    {
      types[i].range = sqrt(pl[i].x*pl[i].x + pl[i].y*pl[i].y);
      vx = pl[i].x - pl[i+1].x;
      vy = pl[i].y - pl[i+1].y;
      vz = pl[i].z - pl[i+1].z;
      types[i].dista = vx*vx + vy*vy + vz*vz;
    }
    types[plsize].range = sqrt(pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y);
    give_feature(pl, types, pl_corn, pl_surf);
  }

  pub_func(pl_orig, pub_full, msg->header.stamp);
  pub_func(pl_surf, pub_surf, msg->header.stamp);
  pub_func(pl_corn, pub_corn, msg->header.stamp);
}


void give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types, pcl::PointCloud<PointType> &pl_corn, pcl::PointCloud<PointType> &pl_surf)
{
  uint plsize = pl.size();
  uint plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while(types[head].range < blind)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind)
    {
      continue;
    }
    // i_nex = i; 
    i2 = i;
    // std::cout<<" i: "<<i<<" i_nex "<<i_nex<<"group_size: "<<group_size<<" plsize "<<plsize<<" plsize2 "<<plsize2<<std::endl;
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
    if(plane_type == 1)
    {
      for(uint j=i; j<=i_nex; j++)
      { 
        if(j!=i && j!=i_nex)
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }
      
      i = i_nex - 1;
      last_state = 1;
    }
    else if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    else if(plane_type == 0)
    {
      if(last_state == 1)
      {
        uint i_nex_tem;
        uint j;
        for(j=last_i+1; j<=last_i_nex; j++)
        {
          uint i_nex_tem2 = i_nex_tem;
          Eigen::Vector3d curr_direct2;

          uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

          if(ttem != 1)
          {
            i_nex_tem = i_nex_tem2;
            break;
          }
          curr_direct = curr_direct2;
        }

        if(j == last_i+1)
        {
          last_state = 0;
        }
        else
        {
          for(uint k=last_i_nex; k<=i_nex_tem; k++)
          {
            if(k != i_nex_tem)
            {
              types[k].ftype = Real_Plane;
            }
            else
            {
              types[k].ftype = Poss_Plane;
            }
          }
          i = i_nex_tem-1;
          i_nex = i_nex_tem;
          i2 = j-1;
          last_state = 1;
        }

      }
    }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;

  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }

    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for(int j=0; j<2; j++)
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }

      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;
        }
        else
        {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;
      
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if(types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;
      }
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
     
    }
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }


  // int last_surface = 0;
  // for(uint i=0; i<plsize; i++)
  // {
  //   if(types[i].ftype==Poss_Plane || types[i].ftype==Real_Plane)
  //   {
  //     if(last_surface == 0)
  //     {
  //       pl_surf.push_back(pl[i]);
  //       last_surface = 1;
  //     }
  //     else
  //     {
  //       last_surface = 0;
  //     }
      
  //   }
  //   else if(types[i].ftype==Edge_Jump || types[i].ftype==Edge_Plane)
  //   {
  //     pl_corn.push_back(pl[i]);
  //   }

  // }

  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
      
      if(j == uint(last_surface+point_filter_num-1))
      {
        // for(uint k=last_surface; k<=j; k++)
        // {
        //   PointType ap;
        //   ap.x = pl[k].x;
        //   ap.y = pl[k].y;
        //   ap.z = pl[k].z;
        //   ap.curvature = pl[k].curvature;
        //   ap.intensity = pl[k].intensity;
        //   pl_surf.push_back(ap);
        // }

        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void pub_func(pcl::PointCloud<PointType> &pl, ros::Publisher pub, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
  pub.publish(output);
}

int plane_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }
  
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;
    if(two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }


  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if(lidar_type==MID || lidar_type==HORIZON)
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}


bool edge_jump_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

 
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}



