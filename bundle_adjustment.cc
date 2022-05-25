// 替换原来的AddImageToProblem的实现,注意要自己思考一下，那些变量需要在其他模块定义
void BundleAdjuster::AddImageToProblem(const image_t image_id,
                                       Reconstruction* reconstruction,
                                       ceres::LossFunction* loss_function) {
  Image& image = reconstruction->Image(image_id);
  Camera& camera = reconstruction->Camera(image.CameraId());

  // CostFunction assumes unit quaternions.
  image.NormalizeQvec();


  double* qvec_data = image.Qvec().data();
  double* tvec_data = image.Tvec().data();

  double* camera_params_data = camera.ParamsData();
  const bool constant_pose =
      !options_.refine_extrinsics || config_.HasConstantPose(image_id);

  // Add residuals to bundle adjustment problem.
  for (const Point2D& point2D : image.Points2D()) {
    if (!point2D.HasPoint3D()) {
      continue;
    }
  Point3D& point3D = reconstruction->Point3D(point2D.Point3DId());

  point3D_track_length[point2D.Point3DId()] = point3D.Track().Length();
  }
//  auto length_max_=findMaxValuePair(point3D_track_length);

  /////////////////////////////////////////////////////////////

  size_t num_observations = 0;
  for (const Point2D& point2D : image.Points2D()) {
    if (!point2D.HasPoint3D()) {
      continue;
    }
    num_observations += 1;
    point3D_num_observations_[point2D.Point3DId()] += 1; //这个point3D_num_observation记录：3D点是由几个像点贡献

    Point3D& point3D = reconstruction->Point3D(point2D.Point3DId());

    assert(point3D.Track().Length() > 1); // 确保至少双像前方交会

    ceres::CostFunction* cost_function = nullptr;

    if (constant_pose) {
      switch (camera.ModelId()) {
#define CAMERA_MODEL_CASE(CameraModel)                                 \
  case CameraModel::kModelId:                                          \
    cost_function =                                                    \
        BundleAdjustmentConstantPoseCostFunction<CameraModel>::Create( \
            image.Qvec(), image.Tvec(), point2D.XY());                 \
    break;

        CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
      }

      problem_->AddResidualBlock(cost_function, loss_function,
                                 point3D.XYZ().data(), camera_params_data);//被优化量加入残差-3D点和相机内参
    } else {
      switch (camera.ModelId()) {
#define CAMERA_MODEL_CASE(CameraModel)                                   \
  case CameraModel::kModelId:                                            \
    cost_function =                                                      \
        BundleAdjustmentCostFunction<CameraModel>::Create(point2D.XY(),point2D.scale_x(),point2D.scale_y()); \
    break;

        CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
      }


      problem_->AddResidualBlock(cost_function, loss_function, qvec_data,
                                 tvec_data, point3D.XYZ().data(),
                                 camera_params_data);

    }



  }


  // 顾忌曝光延迟的gps 约束

    if(reconstruction->b_usable_prior and image.HasTvecPrior())
    {
        Vec3 weight(100,100,100);// rtk
        ceres::CostFunction * cost_function= PoseCenterConstraintCostFunction::Create(image.TvecPrior(),
                                                                                      weight,image.SpeedPrior());
        problem_->AddResidualBlock(cost_function,loss_function,
                                   qvec_data,tvec_data);
        reconstruction->b_usable_prior_ba=true; //说明第一次对齐后已经优化了!!!!!!!!!!


    }

//   image.TvecPrior().hasNaN()
  if (num_observations > 0) {
    camera_ids_.insert(image.CameraId());

    // Set pose parameterization.
    if (!constant_pose) {
      ceres::LocalParameterization* quaternion_parameterization =
          new ceres::QuaternionParameterization;
      problem_->SetParameterization(qvec_data, quaternion_parameterization);
      if (config_.HasConstantTvec(image_id)) {
        const std::vector<int>& constant_tvec_idxs =
            config_.ConstantTvec(image_id);
        ceres::SubsetParameterization* tvec_parameterization =
            new ceres::SubsetParameterization(3, constant_tvec_idxs);
        problem_->SetParameterization(tvec_data, tvec_parameterization);
      }
    }
  }
}
