#include <k4a/k4a.hpp>
#include <k4a/k4atypes.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

// 前置声明
uint16_t getDepthValue(const k4a::image& depth_image, int32_t x, int32_t y);
k4a_float2_t mapColorToDepthCoordinates(const k4a::calibration& calibration,
    const k4a::image& depth_image,
    int32_t color_x, int32_t color_y);

uint16_t getDepthValue(const k4a::image& depth_image, int32_t x, int32_t y) {
    if (!depth_image.is_valid() ||
        x < 0 || x >= depth_image.get_width_pixels() ||
        y < 0 || y >= depth_image.get_height_pixels()) {
        return 0;
    }
    const uint16_t* buffer = reinterpret_cast<const uint16_t*>(depth_image.get_buffer());
    return buffer[y * depth_image.get_width_pixels() + x];
}

k4a_float2_t mapColorToDepthCoordinates(const k4a::calibration& calibration,
    const k4a::image& depth_image,
    int32_t color_x, int32_t color_y) {
    k4a_float2_t source_point = { static_cast<float>(color_x), static_cast<float>(color_y) };
    k4a_float2_t target_point = { -1.0f, -1.0f };

    if (!calibration.convert_color_2d_to_depth_2d(source_point, depth_image, &target_point)) {
        return { -1.0f, -1.0f };
    }
    return target_point;
}

int main() {
    k4a::device device;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;

    try {
        device = k4a::device::open(0);
        device.start_cameras(&config);

        k4a::calibration calibration = device.get_calibration(config.depth_mode, config.color_resolution);

        // 棋盘格参数（单位：米）
        const cv::Size pattern_size(11, 8);
        const float square_size = 0.02f;  // 20毫米 = 0.02米
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));

        k4a::capture capture;
        while (device.get_capture(&capture, std::chrono::milliseconds(1000))) {
            k4a::image color_image = capture.get_color_image();
            k4a::image depth_image = capture.get_depth_image();

            if (color_image.is_valid() && depth_image.is_valid()) {
                cv::Mat color_frame(
                    color_image.get_height_pixels(),
                    color_image.get_width_pixels(),
                    CV_8UC4,
                    color_image.get_buffer()
                );

                // ================== 预处理 ==================
                cv::Mat gray, denoised;
                cv::cvtColor(color_frame, gray, cv::COLOR_BGRA2GRAY);
                cv::fastNlMeansDenoising(gray, denoised, 10, 7, 21);
                clahe->apply(denoised, denoised);

                // ================== 棋盘检测 ==================
                std::vector<cv::Point2f> corners;
                bool found = cv::findChessboardCorners(
                    denoised, pattern_size, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
                );

                if (found) {
                    cv::cornerSubPix(denoised, corners, cv::Size(11, 11), cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01));

                    // ============== 标定参数处理 ================
                    const auto& intrinsics = calibration.color_camera_calibration.intrinsics;

                    // 验证畸变模型类型
                    if (intrinsics.type != K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY) {
                        std::cerr << "错误：不支持的畸变模型类型" << std::endl;
                        continue;
                    }

                    const auto& param = intrinsics.parameters.param;

                    // 构建相机矩阵（像素单位）
                    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
                        param.fx, 0, param.cx,
                        0, param.fy, param.cy,
                        0, 0, 1);

                    // Brown-Conrady畸变参数 (k1, k2, p1, p2, k3)
                    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) <<
                        param.k1, param.k2,
                        param.p1, param.p2,
                        param.k3);

                    // ============== 去畸变处理 ================
                    std::vector<cv::Point2f> undistorted_corners;
                    cv::undistortPoints(
                        corners,
                        undistorted_corners,
                        camera_matrix,
                        dist_coeffs,
                        cv::noArray(),
                        camera_matrix
                    );

                    // ============== 生成世界坐标系点 ================
                    std::vector<cv::Point3f> object_points;
                    for (int i = 0; i < pattern_size.height; ++i) {
                        for (int j = 0; j < pattern_size.width; ++j) {
                            object_points.emplace_back(
                                j * square_size,  // X (米)
                                i * square_size,  // Y (米)
                                0.0f              // Z (米)
                            );
                        }
                    }

                    // ============== 姿态估计 ================
                    cv::Mat rvec, tvec;
                    if (!cv::solvePnP(
                        object_points,
                        undistorted_corners,
                        camera_matrix,
                        cv::Mat(),
                        rvec,
                        tvec,
                        false,
                        cv::SOLVEPNP_ITERATIVE
                    )) {
                        std::cerr << "solvePnP失败" << std::endl;
                        continue;
                    }

                    // ============== 理论深度计算 ================
                    cv::Mat rot_mat;
                    cv::Rodrigues(rvec, rot_mat);

                    // 计算外参矩阵
                    cv::Mat extrinsic = cv::Mat::eye(4, 4, CV_64F);
                    rot_mat.copyTo(extrinsic(cv::Rect(0, 0, 3, 3)));
                    tvec.copyTo(extrinsic(cv::Rect(3, 0, 1, 3)));

                    std::vector<cv::Point3d> theoretical_coords_mm;
                    for (const auto& obj_pt : object_points) {
                        cv::Mat world_point = (cv::Mat_<double>(4, 1) <<
                            obj_pt.x, obj_pt.y, obj_pt.z, 1.0);
                        cv::Mat camera_point = extrinsic * world_point;
                        theoretical_coords_mm.emplace_back(
                            camera_point.at<double>(0) * 1000.0,
                            camera_point.at<double>(1) * 1000.0,
                            camera_point.at<double>(2) * 1000.0
                        );
                    }

                    // ============== 坐标系转换：RGB坐标系 -> 深度坐标系 ================

                    std::vector<cv::Point3d> depth_coords_mm;
                    for (const auto& rgb_coord : theoretical_coords_mm) {
                        // RGB坐标系下的点（毫米转米）
                        k4a_float3_t source_point = {
                            static_cast<float>(rgb_coord.x),
                            static_cast<float>(rgb_coord.y),
                            static_cast<float>(rgb_coord.z)
                        };

                        // 转换到深度坐标系
                        k4a_float3_t target_point;
                        target_point = calibration.convert_3d_to_3d(
                            source_point,
                            K4A_CALIBRATION_TYPE_COLOR,
                            K4A_CALIBRATION_TYPE_DEPTH
                        );



                        // 保存深度坐标系下的坐标（毫米单位）
                        depth_coords_mm.emplace_back(
                            target_point.xyz.x,
                            target_point.xyz.y,
                            target_point.xyz.z
                        );
                    }

                    // ============== 深度验证 ================
                    std::cout << "\n======== 深度验证 (深度坐标系) ========\n";
                    double total_error = 0;
                    int valid_points = 0;
                    for (size_t i = 0; i < corners.size(); ++i) {
                        k4a_float2_t depth_coord = mapColorToDepthCoordinates(
                            calibration, depth_image,
                            static_cast<int>(corners[i].x),
                            static_cast<int>(corners[i].y)
                        );

                        if (depth_coord.xy.x >= 0 && depth_coord.xy.y >= 0) {
                            uint16_t measured_depth_mm = getDepthValue(
                                depth_image,
                                static_cast<int>(depth_coord.xy.x),
                                static_cast<int>(depth_coord.xy.y)
                            );

                            const auto& depth_coord_mm = depth_coords_mm[i];
                            double error = abs(depth_coord_mm.z - measured_depth_mm);

                            std::cout << "角点 " << i
                                << " 理论坐标(深度系): ("
                                << depth_coord_mm.x << ", "
                                << depth_coord_mm.y << ") mm | "
                                << "理论深度: " << depth_coord_mm.z << " mm | "
                                << "实测深度: " << measured_depth_mm << " mm | "
                                << "差值: " << error << " mm\n";

                            total_error += error;
                            valid_points++;
                        }
                    }

                    // ============== 可视化 ================
                    cv::drawChessboardCorners(color_frame, pattern_size, corners, found);

                    // 显示统计信息
                    if (valid_points > 0) {
                        std::stringstream info;
                        info << "平均误差: " << (total_error / valid_points) << " mm\n"
                            << "平移向量: [" << tvec.at<double>(0) * 1000 << ", "
                            << tvec.at<double>(1) * 1000 << ", "
                            << tvec.at<double>(2) * 1000 << "] mm";
                        cv::putText(color_frame, info.str(), cv::Point(20, 80),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    }

                    cv::namedWindow("Calibration", cv::WINDOW_NORMAL);
                    cv::resizeWindow("Calibration", 1280, 720);
                    cv::imshow("Calibration", color_frame);
                }

                if (cv::waitKey(1) == 27) break;
            }
        }
    }
    catch (const k4a::error& e) {
        std::cerr << "Kinect错误: " << e.what() << std::endl;
        return -1;
    }
    catch (const cv::Exception& e) {
        std::cerr << "OpenCV错误: " << e.what() << std::endl;
        return -2;
    }

    device.stop_cameras();
    return 0;
}
