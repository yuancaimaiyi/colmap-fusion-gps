/*
   by licheng

*/
// 替换原来的GBA
bool IncrementalMapper::AdjustGlobalBundle(
    const Options& options, const BundleAdjustmentOptions& ba_options) {
  CHECK_NOTNULL(reconstruction_);
  const std::vector<image_t>& reg_image_ids = reconstruction_->RegImageIds();

  CHECK_GE(reg_image_ids.size(), 2) << "At least two images must be "
                                       "registered for global "
                                       "bundle-adjustment";



  // Avoid degeneracies in bundle adjustment.
  // 过滤到深度为负的,避免退化
  reconstruction_->FilterObservationsWithNegativeDepth();




  // Configure bundle adjustment.
  // 配置ba

  BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);

  }

  // Fix the existing images, if option specified.
  // 固定已经register 的,默认参数是不固定,如果设置为true,只会对新进来的图像gba
  if (options.fix_existing_images) {
    for (const image_t image_id : reg_image_ids) {
      if (existing_image_ids_.count(image_id)) {
        ba_config.SetConstantPose(image_id);
      }
    }
  }

  // Fix 7-DOFs of the bundle adjustment problem.
  ba_config.SetConstantPose(reg_image_ids[0]);
  if (!options.fix_existing_images ||
      !existing_image_ids_.count(reg_image_ids[1])) {
    ba_config.SetConstantTvec(reg_image_ids[1], {0});
  }







// Image& image = reconstruction->Image(image_id);
   int x_gps_prior_size=0;
   std::vector<Eigen::Vector3d> X_SfM;
   std::vector<Eigen::Vector3d> X_GPS;
   X_SfM.clear();
   X_GPS.clear();
   for(image_t image_id :reg_image_ids)
   {
       Image& image = reconstruction_->Image(image_id);
       // 已经在feature extract 环节将wgs84转为utm 了;
       if(image.HasTvecPrior())
       {

           X_SfM.push_back(image.ProjectionCenter());

           X_GPS.push_back(image.TvecPrior());
           x_gps_prior_size++;

       }
       else
       {
           x_gps_prior_size--;
       }
   }

   std::vector<std::string> opti_name;
   for (const image_t image_id :ba_config.Images()) {
     Image& image = reconstruction_->Image(image_id);
     opti_name.push_back(image.Name());
   }


   //删除重复元素
  if (X_GPS.size()>0)
  {
      std::vector<Eigen::Vector3d>::iterator it,it1;
      for (it=++X_GPS.begin(); it != X_GPS.end();)
      {
          it1 = find(X_GPS.begin(),it,*it);
          if(it1 != it)
          {
               auto pos =find(X_GPS.begin(),X_GPS.end(),*it);
               auto index =std::distance(std::begin(X_GPS),pos);
               X_SfM.erase(X_SfM.begin()+index);
               it=X_GPS.erase(it);
               x_gps_prior_size--;

          }
          else
          {
               it++;
          }
      }
  }





   //  align sfm data

//   if (X_GPS.size()>3 && x_gps_prior_size>3)
   if (X_GPS.size()>3)
   {
       int min_common=3;
       RANSACOptions ransac_options;
       ransac_options.max_error=0.3;
       if (reconstruction_->AlignRobust_data(X_SfM,X_GPS,min_common,ransac_options))
       {
            reconstruction_->b_usable_prior=true; // align sucessful ,then use position constraint
            std::vector<double> errors;
            for (image_t image_id :reg_image_ids) {
                Image& image = reconstruction_->Image(image_id);
                if(image.HasTvecPrior())
                {
                    errors.push_back((image.ProjectionCenter() -image.TvecPrior()).norm());
                }


            }
           reconstruction_->align_error_before=Mean(errors);
       }


   }
   else
   {
      reconstruction_->b_usable_prior=false;
   }

/*
 *先对齐后bundle adjustment
 *
*/


   // Run bundle adjustment.
   BundleAdjuster bundle_adjuster(ba_options, ba_config);
   if (!bundle_adjuster.Solve(reconstruction_)) {
     return false;
   }

   if(reconstruction_->b_usable_prior_ba)
   {
//      reconstruction_->UnNormalize(reconstruction_->trans,reconstruction_->scale);
      std::vector<double> ba_after_errors;
      for (image_t image_id :reg_image_ids) {
          Image& image = reconstruction_->Image(image_id);
          if(image.HasTvecPrior())
          {
              ba_after_errors.push_back((image.ProjectionCenter() -image.TvecPrior()).norm());
          }
      }
    reconstruction_->align_error=Mean(ba_after_errors);
    std::cout
            <<"pose prior statistics :\n"
            <<" ...........................\n"
            <<" - starting fitting error: "<<reconstruction_->align_error_before<<"\n"
            <<" - final fitting error: "<<reconstruction_->align_error<<"\n"
            <<" ...........................\n";

   }



  // Normalize scene for numerical stability and
  // to avoid large scale changes in viewer.
  //  reconstruction_->Normalize(); // 归一化去调,作者之所以这样做是因为单目本身就没尺度,所以归一化不影响
  return true;
}
