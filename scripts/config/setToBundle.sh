#!/bin/bash
rsync -avzC --include="mars::Task.yml" --include="skid4_control::SimpleController.yml"  --include="asguard::BodyTask.yml" --include="joint_dispatcher::Task.yml" --include="mars::ForceTorque6DOF.yml" --include="mars::IMU.yml" --include="mars::Joints.yml" --include="mars::RotatingLaserRangeFinder.yml" --include="mars::Task.yml" --include="skid4_control::SimpleController.yml" --exclude="*" ./ $AUTOPROJ_CURRENT_ROOT/bundles/entern/config/orogen/ 

