-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- Dec 19th, 2017, 
-- The following config are based on cartographer developers' recommendation
-- this works well on maps that does not have huge openspace, like the obstacle course
-- and yorkdale, but it did poorly on maps like changi, for now we cannot find a universial
-- solution for both type of maps, the issue has been send to cartographer developers and
-- waiting for reply, the issue is on github at
-- https://github.com/googlecartographer/cartographer_ros/issues/628

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,
  min_range = 0.,
  
  -- max range for laser data, a small value (such as 10) works very well on small maps like
  -- yorkdale maps, as we consider any data outside this range to be noisy. However on large
  -- open space maps like Changi the small range cause the cartographer not be able to locate
  -- itselve very well, thus increase the range provide better results.
  max_range = 20.,
  
  min_z = -0.8,
  max_z = 2.,
  missing_data_ray_length = 5.,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.025,

  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,
    
    -- max range of voxel filter, any points beyond this range is being discared, usually
    -- this range go with max_range of Trajectory_builder_2d
    max_range = 40.,
  },

  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,

    -- max range of voxel filter, any points beyond this range is being discared, usually
    -- this range go with max_range of Trajectory_builder_2d
    max_range = 40.,
  },

  use_online_correlative_scan_matching = true,
  real_time_correlative_scan_matcher = {

    -- by increasing the search window below, cartographer have less break-map behaviours
    -- when the max_range is not large enough, however increasing this will also significantly
    -- slow down the runtime.
    -- Warning: 
    -- These number are roughly based upon our linear and angular velocities and a laser that
    -- publishes at 40hz.  Assuming a 1.6m/s linear velocity and 1.0 angular velocity the
    -- max change between two consecutive 40hz lasers would be: linear = 0.04m, angular = 1.4 degrees.
    -- These search ranges would need to be changed if the assumed velocities and laser rate change
    -- e.g if we were to ever use the merged scan which has an irregular rate.
    linear_search_window = 0.1,
    angular_search_window = math.rad(5.),

    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    -- these weights seems not only have impact of relative ratio to each other
    -- but also the scaling would impact the performance as well
    -- for example, set the following three weight to 10 10 10 would throw the scan matcher off the chart
    occupied_space_weight = 1e1, -- the weight of how important is laser scan data
    translation_weight = 1e1,    -- the weight of how costy it is to transfom from given initial pose
    rotation_weight = 1e2,       -- the weight of how costy it is to rotate from given initial pose
    
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  imu_gravity_time_constant = 10.,  
  submaps = {
    resolution = 0.05,
    -- this determins how many laser scans per submap, in our case it determines how big is the submaps
    -- after each submap is set, cartographer would stitch the submap back to the global, and resolve any
    -- disaligment. The assumption is the sumbaps won't be off by too much thus when the stitch happens
    -- it won't cause too much a problem on the aligment. But we have a known issues with odom when tunning
    -- therefore the default size of 90 doesn't work well as by that time the submap may be too off aligned 
    -- and the stitch is doing a poor job. Thus we set it to 30 
    num_range_data = 30,
    range_data_inserter = {
      insert_free_space = true,
      hit_probability = 0.55,
      miss_probability = 0.49,
    },
  },
}
