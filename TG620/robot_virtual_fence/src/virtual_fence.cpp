#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <fstream>

using namespace std::chrono_literals;

enum class FenceMode
{
  INCLUSIVE,  // 围栏内允许，外部禁止（不允许出去）
  EXCLUSIVE   // 围栏内禁止，外部允许（不允许进入）
};

class VirtualFenceNode : public rclcpp::Node
{
public:
  VirtualFenceNode() : Node("virtual_fence_node")
  {
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // 声明参数
    this->declare_parameter<bool>("fence.enable", true);
    this->declare_parameter<std::string>("fence_mode", "inclusive");
    this->declare_parameter<double>("fence_thickness", 0.02);
    this->declare_parameter<double>("fence_height", 0.5);
    this->declare_parameter<double>("fence_size_x", 0.5);
    this->declare_parameter<double>("fence_size_y", 0.5);
    this->declare_parameter<double>("fence_center_x", 0.3);
    this->declare_parameter<double>("fence_center_y", 0.3);
    this->declare_parameter<std::string>("frame_id", "base_link");

    // 获取参数
    bool fence_enabled = this->get_parameter("fence.enable").as_bool();
    std::string mode_str = this->get_parameter("fence_mode").as_string();
    double fence_thickness = this->get_parameter("fence_thickness").as_double();
    double fence_height = this->get_parameter("fence_height").as_double();
    double fence_size_x = this->get_parameter("fence_size_x").as_double();
    double fence_size_y = this->get_parameter("fence_size_y").as_double();
    double fence_center_x = this->get_parameter("fence_center_x").as_double();
    double fence_center_y = this->get_parameter("fence_center_y").as_double();
    std::string frame_id = this->get_parameter("frame_id").as_string();

    // 确定围栏模式
    FenceMode fence_mode = FenceMode::INCLUSIVE;
    if (mode_str == "exclusive")
    {
      fence_mode = FenceMode::EXCLUSIVE;
    }

    rclcpp::sleep_for(2s); // 等待规划场景初始化

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    if (fence_enabled && fence_mode == FenceMode::INCLUSIVE)
    {
      // Inclusive模式：创建外围墙壁，限制机器人不能出去
      RCLCPP_INFO(this->get_logger(), "Creating INCLUSIVE fence (robot cannot exit)...");

      // ---- 前墙 ----
      collision_objects.push_back(createWall("fence_front", fence_thickness, fence_size_y, fence_height,
                                             fence_center_x + fence_size_x / 2.0, fence_center_y, fence_height / 2.0, frame_id));

      // ---- 后墙 ----
      collision_objects.push_back(createWall("fence_back", fence_thickness, fence_size_y, fence_height,
                                             fence_center_x - fence_size_x / 2.0, fence_center_y, fence_height / 2.0, frame_id));

      // ---- 左墙 ----
      collision_objects.push_back(createWall("fence_left", fence_size_x, fence_thickness, fence_height,
                                             fence_center_x, fence_center_y + fence_size_y / 2.0, fence_height / 2.0, frame_id));

      // ---- 右墙 ----
      collision_objects.push_back(createWall("fence_right", fence_size_x, fence_thickness, fence_height,
                                             fence_center_x, fence_center_y - fence_size_y / 2.0, fence_height / 2.0, frame_id));

      RCLCPP_INFO(this->get_logger(),
                  "Inclusive fence: center=[%.3f, %.3f], size=[%.3fx%.3f], height=%.3f",
                  fence_center_x, fence_center_y, fence_size_x, fence_size_y, fence_height);
    }
    else if (fence_enabled && fence_mode == FenceMode::EXCLUSIVE)
    {
      // Exclusive模式：创建实心禁区，机器人不能进入这个区域
      RCLCPP_INFO(this->get_logger(), "Creating EXCLUSIVE fence (robot cannot enter)...");

      collision_objects.push_back(createSolidBox("forbidden_zone", fence_size_x, fence_size_y, fence_height,
                                                 fence_center_x, fence_center_y, fence_height / 2.0, frame_id));

      RCLCPP_INFO(this->get_logger(),
                  "Exclusive fence: center=[%.3f, %.3f], size=[%.3fx%.3f], height=%.3f",
                  fence_center_x, fence_center_y, fence_size_x, fence_size_y, fence_height);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Virtual fence disabled.");
    }

    // 添加到规划场景
    if (!collision_objects.empty())
    {
      planning_scene_interface_->applyCollisionObjects(collision_objects);
      RCLCPP_INFO(this->get_logger(), "✅ Virtual fence added to planning scene.");
    }

    // 保存围栏和障碍物ID
    if (fence_enabled)
    {
      if (fence_mode == FenceMode::INCLUSIVE)
      {
        fence_object_ids_ = {"fence_front", "fence_back", "fence_left", "fence_right"};
      }
      else
      {
        fence_object_ids_ = {"forbidden_zone"};
      }
    }

    // 添加额外的障碍物
    addCustomObstacles(frame_id);

    // 创建服务
    remove_all_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "remove_all_obstacles",
        std::bind(&VirtualFenceNode::removeAllObstaclesCallback, this, std::placeholders::_1, std::placeholders::_2));

    remove_obstacles_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "remove_custom_obstacles",
        std::bind(&VirtualFenceNode::removeCustomObstaclesCallback, this, std::placeholders::_1, std::placeholders::_2));

    enable_obstacles_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "enable_obstacles",
        std::bind(&VirtualFenceNode::enableObstaclesCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Services ready:");
    RCLCPP_INFO(this->get_logger(), "  - /remove_all_obstacles (移除所有障碍物和围栏)");
    RCLCPP_INFO(this->get_logger(), "  - /remove_custom_obstacles (只移除自定义障碍物)");
    RCLCPP_INFO(this->get_logger(), "  - /enable_obstacles (启用/禁用自定义障碍物)");
  }

private:
  // 创建墙壁（用于inclusive模式）
  moveit_msgs::msg::CollisionObject createWall(const std::string &id,
                                               double size_x, double size_y, double size_z,
                                               double pos_x, double pos_y, double pos_z,
                                               const std::string &frame_id)
  {
    moveit_msgs::msg::CollisionObject wall;
    wall.id = id;
    wall.header.frame_id = frame_id;
    wall.operation = wall.ADD;

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {size_x, size_y, size_z};

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos_x;
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    pose.orientation.w = 1.0; // 无旋转

    wall.primitives.push_back(box);
    wall.primitive_poses.push_back(pose);

    return wall;
  }

  // 创建实心盒子（用于exclusive模式）
  moveit_msgs::msg::CollisionObject createSolidBox(const std::string &id,
                                                   double size_x, double size_y, double size_z,
                                                   double pos_x, double pos_y, double pos_z,
                                                   const std::string &frame_id)
  {
    moveit_msgs::msg::CollisionObject solid_box;
    solid_box.id = id;
    solid_box.header.frame_id = frame_id;
    solid_box.operation = solid_box.ADD;

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {size_x, size_y, size_z};

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos_x;
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    pose.orientation.w = 1.0; // 无旋转

    solid_box.primitives.push_back(box);
    solid_box.primitive_poses.push_back(pose);

    return solid_box;
  }

  // 创建圆柱体障碍物
  moveit_msgs::msg::CollisionObject createCylinder(const std::string &id,
                                                    double radius, double height,
                                                    double pos_x, double pos_y, double pos_z,
                                                    const std::string &frame_id)
  {
    moveit_msgs::msg::CollisionObject cylinder;
    cylinder.id = id;
    cylinder.header.frame_id = frame_id;
    cylinder.operation = cylinder.ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions = {height, radius};

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos_x;
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    pose.orientation.w = 1.0;

    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(pose);

    return cylinder;
  }

  // 创建球体障碍物
  moveit_msgs::msg::CollisionObject createSphere(const std::string &id,
                                                  double radius,
                                                  double pos_x, double pos_y, double pos_z,
                                                  const std::string &frame_id)
  {
    moveit_msgs::msg::CollisionObject sphere;
    sphere.id = id;
    sphere.header.frame_id = frame_id;
    sphere.operation = sphere.ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.SPHERE;
    primitive.dimensions = {radius};

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos_x;
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    pose.orientation.w = 1.0;

    sphere.primitives.push_back(primitive);
    sphere.primitive_poses.push_back(pose);

    return sphere;
  }

  // 创建圆锥体障碍物
  moveit_msgs::msg::CollisionObject createCone(const std::string &id,
                                                double radius, double height,
                                                double pos_x, double pos_y, double pos_z,
                                                const std::string &frame_id)
  {
    moveit_msgs::msg::CollisionObject cone;
    cone.id = id;
    cone.header.frame_id = frame_id;
    cone.operation = cone.ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CONE;
    primitive.dimensions = {height, radius};

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos_x;
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    pose.orientation.w = 1.0;

    cone.primitives.push_back(primitive);
    cone.primitive_poses.push_back(pose);

    return cone;
  }

  // 从STL文件创建Mesh障碍物
  moveit_msgs::msg::CollisionObject createMeshFromSTL(const std::string &id,
                                                       const std::string &stl_file_path,
                                                       double pos_x, double pos_y, double pos_z,
                                                       double scale,
                                                       const std::string &frame_id)
  {
    moveit_msgs::msg::CollisionObject mesh_object;
    mesh_object.id = id;
    mesh_object.header.frame_id = frame_id;
    mesh_object.operation = mesh_object.ADD;

    shape_msgs::msg::Mesh mesh;

    std::ifstream file(stl_file_path, std::ios::binary);
    if (!file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open STL file: %s", stl_file_path.c_str());
      return mesh_object;
    }

    char header[80];
    file.read(header, 80);

    uint32_t num_triangles;
    file.read(reinterpret_cast<char*>(&num_triangles), sizeof(uint32_t));

    for (uint32_t i = 0; i < num_triangles; ++i)
    {
      float normal[3];
      file.read(reinterpret_cast<char*>(normal), 3 * sizeof(float));

      for (int j = 0; j < 3; ++j)
      {
        float vertex[3];
        file.read(reinterpret_cast<char*>(vertex), 3 * sizeof(float));

        geometry_msgs::msg::Point p;
        p.x = vertex[0] * scale;
        p.y = vertex[1] * scale;
        p.z = vertex[2] * scale;
        mesh.vertices.push_back(p);
      }

      shape_msgs::msg::MeshTriangle triangle;
      triangle.vertex_indices[0] = i * 3;
      triangle.vertex_indices[1] = i * 3 + 1;
      triangle.vertex_indices[2] = i * 3 + 2;
      mesh.triangles.push_back(triangle);

      uint16_t attribute_byte_count;
      file.read(reinterpret_cast<char*>(&attribute_byte_count), sizeof(uint16_t));
    }

    file.close();

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos_x;
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    pose.orientation.w = 1.0;

    mesh_object.meshes.push_back(mesh);
    mesh_object.mesh_poses.push_back(pose);

    RCLCPP_INFO(this->get_logger(), "Loaded STL mesh with %u triangles from: %s",
                num_triangles, stl_file_path.c_str());

    return mesh_object;
  }

  // 服务回调：移除所有障碍物
  void removeAllObstaclesCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    std::vector<std::string> all_ids = fence_object_ids_;
    all_ids.insert(all_ids.end(), obstacle_object_ids_.begin(), obstacle_object_ids_.end());

    planning_scene_interface_->removeCollisionObjects(all_ids);

    response->success = true;
    response->message = "Removed all obstacles and fence (" + std::to_string(all_ids.size()) + " objects)";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  // 服务回调：只移除自定义障碍物
  void removeCustomObstaclesCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    planning_scene_interface_->removeCollisionObjects(obstacle_object_ids_);

    response->success = true;
    response->message = "Removed custom obstacles (" + std::to_string(obstacle_object_ids_.size()) + " objects)";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  // 服务回调：启用/禁用自定义障碍物
  void enableObstaclesCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data)
    {
      // 重新添加障碍物
      std::string frame_id = this->get_parameter("frame_id").as_string();
      addCustomObstacles(frame_id);
      response->success = true;
      response->message = "Custom obstacles enabled";
    }
    else
    {
      // 移除障碍物
      planning_scene_interface_->removeCollisionObjects(obstacle_object_ids_);
      response->success = true;
      response->message = "Custom obstacles disabled";
    }
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  // 添加自定义障碍物
  void addCustomObstacles(const std::string &frame_id)
  {
    std::vector<moveit_msgs::msg::CollisionObject> obstacles;
    obstacle_object_ids_.clear();

    // 声明障碍物配置参数
    this->declare_parameter<bool>("obstacles.enable", false);
    this->declare_parameter<int>("obstacles.num_cylinders", 0);
    this->declare_parameter<int>("obstacles.num_boxes", 0);
    this->declare_parameter<int>("obstacles.num_spheres", 0);
    this->declare_parameter<int>("obstacles.num_cones", 0);
    this->declare_parameter<int>("obstacles.num_meshes", 0);

    bool obstacles_enabled = this->get_parameter("obstacles.enable").as_bool();
    if (!obstacles_enabled)
    {
      RCLCPP_INFO(this->get_logger(), "Custom obstacles disabled.");
      return;
    }

    // 添加圆柱体
    int num_cylinders = this->get_parameter("obstacles.num_cylinders").as_int();
    for (int i = 0; i < num_cylinders; ++i)
    {
      std::string prefix = "obstacles.cylinder_" + std::to_string(i);
      this->declare_parameter<double>(prefix + ".radius", 0.05);
      this->declare_parameter<double>(prefix + ".height", 0.2);
      this->declare_parameter<double>(prefix + ".x", 0.0);
      this->declare_parameter<double>(prefix + ".y", 0.0);
      this->declare_parameter<double>(prefix + ".z", 0.1);

      double radius = this->get_parameter(prefix + ".radius").as_double();
      double height = this->get_parameter(prefix + ".height").as_double();
      double x = this->get_parameter(prefix + ".x").as_double();
      double y = this->get_parameter(prefix + ".y").as_double();
      double z = this->get_parameter(prefix + ".z").as_double();

      std::string id = "cylinder_" + std::to_string(i);
      obstacles.push_back(createCylinder(id, radius, height, x, y, z, frame_id));
      obstacle_object_ids_.push_back(id);
      RCLCPP_INFO(this->get_logger(), "Added cylinder_%d: radius=%.3f, height=%.3f, pos=[%.3f, %.3f, %.3f]",
                  i, radius, height, x, y, z);
    }

    // 添加长方体
    int num_boxes = this->get_parameter("obstacles.num_boxes").as_int();
    for (int i = 0; i < num_boxes; ++i)
    {
      std::string prefix = "obstacles.box_" + std::to_string(i);
      this->declare_parameter<double>(prefix + ".size_x", 0.1);
      this->declare_parameter<double>(prefix + ".size_y", 0.1);
      this->declare_parameter<double>(prefix + ".size_z", 0.1);
      this->declare_parameter<double>(prefix + ".x", 0.0);
      this->declare_parameter<double>(prefix + ".y", 0.0);
      this->declare_parameter<double>(prefix + ".z", 0.05);

      double size_x = this->get_parameter(prefix + ".size_x").as_double();
      double size_y = this->get_parameter(prefix + ".size_y").as_double();
      double size_z = this->get_parameter(prefix + ".size_z").as_double();
      double x = this->get_parameter(prefix + ".x").as_double();
      double y = this->get_parameter(prefix + ".y").as_double();
      double z = this->get_parameter(prefix + ".z").as_double();

      std::string id = "box_obstacle_" + std::to_string(i);
      obstacles.push_back(createSolidBox(id, size_x, size_y, size_z, x, y, z, frame_id));
      obstacle_object_ids_.push_back(id);
      RCLCPP_INFO(this->get_logger(), "Added box_%d: size=[%.3f, %.3f, %.3f], pos=[%.3f, %.3f, %.3f]",
                  i, size_x, size_y, size_z, x, y, z);
    }

    // 添加球体
    int num_spheres = this->get_parameter("obstacles.num_spheres").as_int();
    for (int i = 0; i < num_spheres; ++i)
    {
      std::string prefix = "obstacles.sphere_" + std::to_string(i);
      this->declare_parameter<double>(prefix + ".radius", 0.05);
      this->declare_parameter<double>(prefix + ".x", 0.0);
      this->declare_parameter<double>(prefix + ".y", 0.0);
      this->declare_parameter<double>(prefix + ".z", 0.1);

      double radius = this->get_parameter(prefix + ".radius").as_double();
      double x = this->get_parameter(prefix + ".x").as_double();
      double y = this->get_parameter(prefix + ".y").as_double();
      double z = this->get_parameter(prefix + ".z").as_double();

      std::string id = "sphere_" + std::to_string(i);
      obstacles.push_back(createSphere(id, radius, x, y, z, frame_id));
      obstacle_object_ids_.push_back(id);
      RCLCPP_INFO(this->get_logger(), "Added sphere_%d: radius=%.3f, pos=[%.3f, %.3f, %.3f]",
                  i, radius, x, y, z);
    }

    // 添加圆锥体
    int num_cones = this->get_parameter("obstacles.num_cones").as_int();
    for (int i = 0; i < num_cones; ++i)
    {
      std::string prefix = "obstacles.cone_" + std::to_string(i);
      this->declare_parameter<double>(prefix + ".radius", 0.05);
      this->declare_parameter<double>(prefix + ".height", 0.2);
      this->declare_parameter<double>(prefix + ".x", 0.0);
      this->declare_parameter<double>(prefix + ".y", 0.0);
      this->declare_parameter<double>(prefix + ".z", 0.1);

      double radius = this->get_parameter(prefix + ".radius").as_double();
      double height = this->get_parameter(prefix + ".height").as_double();
      double x = this->get_parameter(prefix + ".x").as_double();
      double y = this->get_parameter(prefix + ".y").as_double();
      double z = this->get_parameter(prefix + ".z").as_double();

      std::string id = "cone_" + std::to_string(i);
      obstacles.push_back(createCone(id, radius, height, x, y, z, frame_id));
      obstacle_object_ids_.push_back(id);
      RCLCPP_INFO(this->get_logger(), "Added cone_%d: radius=%.3f, height=%.3f, pos=[%.3f, %.3f, %.3f]",
                  i, radius, height, x, y, z);
    }

    // 添加STL网格
    int num_meshes = this->get_parameter("obstacles.num_meshes").as_int();
    for (int i = 0; i < num_meshes; ++i)
    {
      std::string prefix = "obstacles.mesh_" + std::to_string(i);
      this->declare_parameter<std::string>(prefix + ".file_path", "");
      this->declare_parameter<double>(prefix + ".x", 0.0);
      this->declare_parameter<double>(prefix + ".y", 0.0);
      this->declare_parameter<double>(prefix + ".z", 0.0);
      this->declare_parameter<double>(prefix + ".scale", 1.0);

      std::string file_path = this->get_parameter(prefix + ".file_path").as_string();
      double x = this->get_parameter(prefix + ".x").as_double();
      double y = this->get_parameter(prefix + ".y").as_double();
      double z = this->get_parameter(prefix + ".z").as_double();
      double scale = this->get_parameter(prefix + ".scale").as_double();

      if (!file_path.empty())
      {
        std::string id = "mesh_" + std::to_string(i);
        obstacles.push_back(createMeshFromSTL(id, file_path, x, y, z, scale, frame_id));
        obstacle_object_ids_.push_back(id);
        RCLCPP_INFO(this->get_logger(), "Added mesh_%d from: %s, pos=[%.3f, %.3f, %.3f], scale=%.3f",
                    i, file_path.c_str(), x, y, z, scale);
      }
    }

    if (!obstacles.empty())
    {
      planning_scene_interface_->applyCollisionObjects(obstacles);
      RCLCPP_INFO(this->get_logger(), "✅ Added %zu custom obstacles to planning scene.", obstacles.size());
    }
  }

  // 成员变量
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::vector<std::string> fence_object_ids_;
  std::vector<std::string> obstacle_object_ids_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_all_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_obstacles_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_obstacles_srv_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VirtualFenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
