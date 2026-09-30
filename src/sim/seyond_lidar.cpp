// CPU lidar for gz-sim 8 (Harmonic): Gazebo Classic's physics ray sensor, which Harmonic
// lacks (gz-sensors CpuLidarSensor exists from Jetty on). Ray-casts against every collision
// shape in the ECM with its own Bullet collision world, so the physics engine and its contact
// detector are untouched. Scans are gated to the update rate and run on a worker thread, so a
// scan never stalls the physics step. Attach to the world; the sensor is the link `link`.
//
// Output: gz.msgs.PointCloudPacked (x,y,z float32) in the sensor link frame on `topic`,
// stamped with the sim time of the pose snapshot the scan was cast against.

#include <btBulletCollisionCommon.h>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace concrete_block_behavior_tree
{

namespace
{

btTransform toBt(const gz::math::Pose3d & p)
{
  const auto & q = p.Rot();
  return btTransform(
    btQuaternion(q.X(), q.Y(), q.Z(), q.W()), btVector3(p.Pos().X(), p.Pos().Y(), p.Pos().Z()));
}

// One collision shape; the mesh data it may point into lives alongside it.
struct Shape
{
  std::unique_ptr<btTriangleMesh> mesh;
  std::unique_ptr<btCollisionShape> shape;
  std::unique_ptr<btCollisionObject> object;
};

std::unique_ptr<Shape> makeShape(const sdf::Geometry & g, const gz::common::Mesh * mesh)
{
  auto s = std::make_unique<Shape>();
  switch (g.Type()) {
    case sdf::GeometryType::BOX: {
        const auto h = g.BoxShape()->Size() / 2.0;
        s->shape = std::make_unique<btBoxShape>(btVector3(h.X(), h.Y(), h.Z()));
        break;
      }
    case sdf::GeometryType::SPHERE:
      s->shape = std::make_unique<btSphereShape>(g.SphereShape()->Radius());
      break;
    case sdf::GeometryType::CYLINDER: {
        const double r = g.CylinderShape()->Radius();
        s->shape = std::make_unique<btCylinderShapeZ>(
          btVector3(r, r, g.CylinderShape()->Length() / 2.0));
        break;
      }
    case sdf::GeometryType::CAPSULE:
      s->shape = std::make_unique<btCapsuleShapeZ>(
        g.CapsuleShape()->Radius(), g.CapsuleShape()->Length());
      break;
    case sdf::GeometryType::PLANE: {
        const auto n = g.PlaneShape()->Normal();
        s->shape = std::make_unique<btStaticPlaneShape>(btVector3(n.X(), n.Y(), n.Z()), 0.0);
        break;
      }
    case sdf::GeometryType::MESH: {
        if (mesh == nullptr) {
          return nullptr;
        }
        const auto k = g.MeshShape()->Scale();
        s->mesh = std::make_unique<btTriangleMesh>();
        for (unsigned int i = 0; i < mesh->SubMeshCount(); ++i) {
          const auto sub = mesh->SubMeshByIndex(i).lock();
          if (!sub || sub->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES) {
            continue;
          }
          const auto v = [&](unsigned int j) {
              const auto p = sub->Vertex(static_cast<unsigned int>(sub->Index(j)));
              return btVector3(p.X() * k.X(), p.Y() * k.Y(), p.Z() * k.Z());
            };
          for (unsigned int j = 0; j + 2 < sub->IndexCount(); j += 3) {
            s->mesh->addTriangle(v(j), v(j + 1), v(j + 2));
          }
        }
        if (s->mesh->getNumTriangles() == 0) {
          return nullptr;
        }
        s->shape = std::make_unique<btBvhTriangleMeshShape>(s->mesh.get(), true);
        break;
      }
    default:
      return nullptr;  // heightmap, polyline, ellipsoid: not in this scene
  }
  s->object = std::make_unique<btCollisionObject>();
  s->object->setCollisionShape(s->shape.get());
  return s;
}

}  // namespace

class SeyondLidar
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemReset
{
public:
  ~SeyondLidar() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    const auto get = [&](const char * key, double fallback) {
        return sdf->Get<double>(key, fallback).first;
      };
    link_ = sdf->Get<std::string>("link", "seyond").first;
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / get("update_rate", 5.0)));
    range_min_ = get("range_min", 0.5);
    range_max_ = get("range_max", 60.0);
    noise_ = get("range_noise_std", 0.02);
    // FOV and ray count measured on real Seyond captures (seibersdorf_dataset, static):
    // az -55.3..57.6 deg, el -55.8..14.1 deg in the `seyond` frame, ~118k points.
    const double az0 = GZ_DTOR(get("azimuth_min", -55.3)), az1 = GZ_DTOR(get("azimuth_max", 57.6));
    const double el0 = GZ_DTOR(get("elevation_min", -55.8)), el1 = GZ_DTOR(get("elevation_max", 14.1));
    const int n_az = static_cast<int>(get("azimuth_samples", 560));
    const int n_el = static_cast<int>(get("elevation_samples", 210));
    for (int i = 0; i < n_el; ++i) {
      const double el = el0 + (el1 - el0) * i / std::max(1, n_el - 1);
      for (int j = 0; j < n_az; ++j) {
        const double az = az0 + (az1 - az0) * j / std::max(1, n_az - 1);
        rays_.emplace_back(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
      }
    }
    pub_ = node_.Advertise<gz::msgs::PointCloudPacked>(
      sdf->Get<std::string>("topic", "/seyond/points").first);
    worker_ = std::thread([this] {work();});
  }

  void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
  {
    last_ = {};  // sim time rewinds on a world reset
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || busy_ || info.simTime - last_ < period_) {
      return;
    }
    if (sensor_ != gz::sim::kNullEntity && !ecm.HasEntity(sensor_)) {
      sensor_ = gz::sim::kNullEntity;
    }
    if (sensor_ == gz::sim::kNullEntity) {
      ecm.Each<gz::sim::components::Link, gz::sim::components::Name>(
        [&](const gz::sim::Entity & e, const gz::sim::components::Link *,
        const gz::sim::components::Name * n) {
          if (n->Data() == link_) {
            sensor_ = e;
          }
          return sensor_ == gz::sim::kNullEntity;
        });
      if (sensor_ == gz::sim::kNullEntity) {
        return;
      }
    }
    // Snapshot on the sim thread; new shapes are built here too, where MeshManager is safe.
    Scan scan;
    scan.stamp = info.simTime;
    scan.sensor = toBt(gz::sim::worldPose(sensor_, ecm));
    ecm.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
      [&](const gz::sim::Entity & e, const gz::sim::components::Collision *,
      const gz::sim::components::Geometry * geom) {
        if (!known_.count(e)) {
          known_.insert(e);
          const gz::common::Mesh * mesh = nullptr;
          if (geom->Data().Type() == sdf::GeometryType::MESH) {
            const auto * m = geom->Data().MeshShape();
            mesh = gz::common::MeshManager::Instance()->Load(
              gz::common::findFile(gz::sim::asFullPath(m->Uri(), m->FilePath())));
          }
          if (auto s = makeShape(geom->Data(), mesh)) {
            scan.added.emplace_back(e, std::move(s));
          } else {
            gzwarn << "SeyondLidar: collision " << e << " (geometry type "
                   << static_cast<int>(geom->Data().Type()) << ") is invisible to the lidar\n";
          }
        }
        scan.poses.emplace_back(e, toBt(gz::sim::worldPose(e, ecm)));
        return true;
      });
    last_ = info.simTime;
    busy_ = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_ = std::move(scan);
      has_pending_ = true;
    }
    cv_.notify_one();
  }

private:
  struct Scan
  {
    std::chrono::steady_clock::duration stamp{};
    btTransform sensor;
    std::vector<std::pair<gz::sim::Entity, std::unique_ptr<Shape>>> added;
    std::vector<std::pair<gz::sim::Entity, btTransform>> poses;
  };

  void work()
  {
    btDefaultCollisionConfiguration config;
    btCollisionDispatcher dispatcher(&config);
    btDbvtBroadphase broadphase;
    btCollisionWorld world(&dispatcher, &broadphase, &config);
    std::map<gz::sim::Entity, std::unique_ptr<Shape>> shapes;
    std::mt19937 rng(std::random_device{}());
    std::normal_distribution<float> noise(0.0f, static_cast<float>(noise_));

    while (true) {
      Scan scan;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] {return stop_ || has_pending_;});
        if (stop_) {
          break;
        }
        scan = std::move(pending_);
        has_pending_ = false;
      }
      for (auto & [e, s] : scan.added) {
        world.addCollisionObject(s->object.get());
        shapes[e] = std::move(s);
      }
      std::set<gz::sim::Entity> seen;
      for (const auto & [e, pose] : scan.poses) {
        seen.insert(e);
        if (auto it = shapes.find(e); it != shapes.end()) {
          it->second->object->setWorldTransform(pose);
        }
      }
      for (auto it = shapes.begin(); it != shapes.end(); ) {  // entity removed from the world
        if (!seen.count(it->first)) {
          world.removeCollisionObject(it->second->object.get());
          it = shapes.erase(it);
        } else {
          ++it;
        }
      }
      world.updateAabbs();

      // rays start at range_min: the sensor's own housing is never hit
      std::vector<float> xyz;
      xyz.reserve(rays_.size() * 3);
      const btVector3 origin = scan.sensor.getOrigin();
      const btMatrix3x3 & rot = scan.sensor.getBasis();
      for (const auto & d : rays_) {
        const btVector3 dw = rot * d;
        const btVector3 from = origin + dw * range_min_, to = origin + dw * range_max_;
        btCollisionWorld::ClosestRayResultCallback hit(from, to);
        world.rayTest(from, to, hit);
        if (hit.hasHit()) {
          const float r = static_cast<float>(
            range_min_ + hit.m_closestHitFraction * (range_max_ - range_min_)) + noise(rng);
          xyz.insert(xyz.end(), {d.x() * r, d.y() * r, d.z() * r});
        }
      }
      publish(scan.stamp, xyz);
      busy_ = false;
    }
    for (auto & [e, s] : shapes) {
      world.removeCollisionObject(s->object.get());
    }
  }

  void publish(std::chrono::steady_clock::duration stamp, const std::vector<float> & xyz)
  {
    gz::msgs::PointCloudPacked msg;
    gz::msgs::InitPointCloudPacked(msg, link_, false, {{"xyz",
        gz::msgs::PointCloudPacked::Field::FLOAT32}});
    *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(stamp);
    const auto n = static_cast<uint32_t>(xyz.size() / 3);
    msg.set_width(n);
    msg.set_height(1);
    msg.set_row_step(n * msg.point_step());
    msg.set_is_dense(true);
    msg.set_data(reinterpret_cast<const char *>(xyz.data()), xyz.size() * sizeof(float));
    pub_.Publish(msg);
  }

  std::string link_;
  std::chrono::steady_clock::duration period_{};
  std::chrono::steady_clock::duration last_{};
  double range_min_{0.5}, range_max_{60.0}, noise_{0.02};
  std::vector<btVector3> rays_;
  gz::sim::Entity sensor_{gz::sim::kNullEntity};
  std::set<gz::sim::Entity> known_;

  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable cv_;
  Scan pending_;
  bool has_pending_{false}, stop_{false};
  std::atomic<bool> busy_{false};
};

}  // namespace concrete_block_behavior_tree

GZ_ADD_PLUGIN(
  concrete_block_behavior_tree::SeyondLidar, gz::sim::System,
  concrete_block_behavior_tree::SeyondLidar::ISystemConfigure,
  concrete_block_behavior_tree::SeyondLidar::ISystemPostUpdate,
  concrete_block_behavior_tree::SeyondLidar::ISystemReset)
