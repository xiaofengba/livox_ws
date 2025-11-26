#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <map>
#include <vector>
#include <chrono>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using namespace std::chrono_literals;

class FastPointCloudRepublisher : public rclcpp::Node
{
public:
    FastPointCloudRepublisher() : Node("fast_pointcloud_republisher")
    {
        // 创建订阅和发布
        subscription_ = create_subscription<PointCloud2>(
            "/rslidar_points", 10,
            [this](const PointCloud2::SharedPtr msg) { process_pointcloud(msg); });
        
        publisher_ = create_publisher<PointCloud2>("/rslidar_points_repub", 10);
        
        // 预定义输出字段
        output_fields_ = create_output_fields();
        
        // 创建定时器来监控性能
        monitor_timer_ = create_wall_timer(2s, [this]() { monitor_performance(); });
        
        RCLCPP_INFO(get_logger(), "PointCloud republisher started");
    }

private:
    rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    std::vector<PointField> output_fields_;
    std::map<std::string, uint32_t> field_offsets_;
    
    // 性能监控变量
    rclcpp::Time last_process_time_;
    rclcpp::Time last_publish_time_;
    size_t processed_count_ = 0;
    size_t published_count_ = 0;
    double total_process_time_ = 0.0;
    double max_process_time_ = 0.0;
    std::vector<double> recent_intervals_;

    // 创建输出点云字段定义
    std::vector<PointField> create_output_fields()
    {
        return {
            create_field("x", 0, PointField::FLOAT32),
            create_field("y", 4, PointField::FLOAT32),
            create_field("z", 8, PointField::FLOAT32),
            create_field("intensity", 12, PointField::FLOAT32),
            create_field("ring", 16, PointField::UINT16),
            create_field("time", 18, PointField::FLOAT32)
        };
    }

    // 辅助函数：创建单个字段
    PointField create_field(const std::string& name, uint32_t offset, uint8_t datatype)
    {
        PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = datatype;
        field.count = 1;
        return field;
    }

    // 主处理函数
    void process_pointcloud(const PointCloud2::SharedPtr input_msg)
    {
        auto start_time = this->now();
        
        // 计算与前帧的时间间隔
        if (last_process_time_.nanoseconds() > 0) {
            double interval = (start_time - last_process_time_).seconds() * 1000.0; // 毫秒
            recent_intervals_.push_back(interval);
            if (recent_intervals_.size() > 10) {
                recent_intervals_.erase(recent_intervals_.begin());
            }
        }
        
        try {
            // 只在字段变化时更新
            if (field_offsets_.empty() || need_update_field_info(*input_msg)) {
                update_field_info(*input_msg);
            }
            
            auto output_msg = convert_pointcloud(*input_msg);
            
            auto before_publish_time = this->now();
            publisher_->publish(*output_msg);
            auto after_publish_time = this->now();
            
            // 记录发布间隔
            if (last_publish_time_.nanoseconds() > 0) {
                double publish_interval = (before_publish_time - last_publish_time_).seconds() * 1000.0;
                RCLCPP_DEBUG(get_logger(), "Publish interval: %.3fms", publish_interval);
            }
            last_publish_time_ = before_publish_time;
            
            published_count_++;
            
        } 
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Processing error: %s", e.what());
        }
        
        // 计算处理时间
        auto end_time = this->now();
        double process_time = (end_time - start_time).seconds() * 1000.0; // 毫秒
        total_process_time_ += process_time;
        max_process_time_ = std::max(max_process_time_, process_time);
        
        processed_count_++;
        last_process_time_ = start_time;
        
        // 偶尔打印处理时间
        if (processed_count_ % 100 == 0) {
            RCLCPP_INFO(get_logger(), 
                       "Avg process time: %.3fms, Max: %.3fms, Points: %u", 
                       total_process_time_ / processed_count_, 
                       max_process_time_,
                       input_msg->width * input_msg->height);
        }
    }

    // 检查是否需要更新字段信息
    bool need_update_field_info(const PointCloud2& msg)
    {
        if (msg.fields.size() != field_offsets_.size()) return true;
        
        for (const auto& field : msg.fields) {
            if (field_offsets_.find(field.name) == field_offsets_.end() ||
                field_offsets_[field.name] != field.offset) {
                return true;
            }
        }
        return false;
    }

    // 更新输入点云字段信息
    void update_field_info(const PointCloud2& msg)
    {
        std::map<std::string, uint32_t> new_offsets;
        for (const auto& field : msg.fields) {
            new_offsets[field.name] = field.offset;
        }
        field_offsets_.swap(new_offsets);
        
        RCLCPP_INFO(get_logger(), "Field info updated, %zu fields", field_offsets_.size());
    }

    // 转换点云数据
    std::unique_ptr<PointCloud2> convert_pointcloud(const PointCloud2& input_msg)
    {
        auto output_msg = std::make_unique<PointCloud2>();
        setup_output_message(*output_msg, input_msg);
        
        uint32_t num_points = input_msg.width * input_msg.height;
        if (num_points == 0) return output_msg;

        output_msg->data.resize(num_points * 22);
        
        convert_points_data(input_msg, output_msg->data, num_points);
        return output_msg;
    }

    // 设置输出消息的基本属性
    void setup_output_message(PointCloud2& output_msg, const PointCloud2& input_msg)
    {
        output_msg.header = input_msg.header;
        output_msg.height = input_msg.height;
        output_msg.width = input_msg.width;
        output_msg.fields = output_fields_;
        output_msg.is_bigendian = false;
        output_msg.point_step = 22;
        output_msg.row_step = output_msg.point_step * input_msg.width;
        output_msg.is_dense = input_msg.is_dense;
    }

    // 转换点数据 - 优化版本
    void convert_points_data(const PointCloud2& input_msg, 
                           std::vector<uint8_t>& output_data, uint32_t num_points)
    {
        double first_timestamp = 0.0;
        bool has_timestamp = field_offsets_.count("timestamp") > 0;
        
        if (has_timestamp) {
            first_timestamp = get_first_timestamp(input_msg);
        }

        const uint32_t input_point_step = input_msg.point_step;
        const uint32_t output_point_step = 22;

        // 批量处理优化
        for (uint32_t i = 0; i < num_points; ++i) {
            const uint8_t* input_point = &input_msg.data[i * input_point_step];
            uint8_t* output_point = &output_data[i * output_point_step];
            
            // 使用memcpy提高性能
            copy_field_memcpy<float>(input_point, output_point, "x");
            copy_field_memcpy<float>(input_point, output_point + 4, "y");
            copy_field_memcpy<float>(input_point, output_point + 8, "z");
            copy_field_memcpy<float>(input_point, output_point + 12, "intensity");
            copy_field_memcpy<uint16_t>(input_point, output_point + 16, "ring");
            
            if (has_timestamp) {
                convert_timestamp_memcpy(input_point, output_point + 18, first_timestamp);
            } else {
                // 如果没有时间戳，填充0
                *reinterpret_cast<float*>(output_point + 18) = 0.0f;
            }
        }
    }

    // 获取第一个点的时间戳作为参考
    double get_first_timestamp(const PointCloud2& msg)
    {
        if (field_offsets_.count("timestamp")) {
            const uint8_t* first_point = &msg.data[field_offsets_.at("timestamp")];
            return *reinterpret_cast<const double*>(first_point);
        }
        return 0.0;
    }

    // 使用memcpy的字段复制函数
    template<typename T>
    void copy_field_memcpy(const uint8_t* input_point, uint8_t* output_point, 
                          const std::string& field_name)
    {
        if (field_offsets_.count(field_name)) {
            const void* src = input_point + field_offsets_.at(field_name);
            std::memcpy(output_point, src, sizeof(T));
        }
    }

    // 使用memcpy的时间戳转换
    void convert_timestamp_memcpy(const uint8_t* input_point, uint8_t* output_point, double first_timestamp)
    {
        if (field_offsets_.count("timestamp")) {
            double timestamp;
            std::memcpy(&timestamp, input_point + field_offsets_.at("timestamp"), sizeof(double));
            float relative_time = static_cast<float>((timestamp - first_timestamp) * 1000.0);
            std::memcpy(output_point, &relative_time, sizeof(float));
        }
    }

    // 性能监控函数
    void monitor_performance()
    {
        if (recent_intervals_.empty()) return;
        
        double avg_interval = 0.0;
        double min_interval = std::numeric_limits<double>::max();
        double max_interval = 0.0;
        
        for (double interval : recent_intervals_) {
            avg_interval += interval;
            min_interval = std::min(min_interval, interval);
            max_interval = std::max(max_interval, interval);
        }
        avg_interval /= recent_intervals_.size();
        
        RCLCPP_INFO(get_logger(), 
                   "Performance - Intervals(ms): Avg=%.3f, Min=%.3f, Max=%.3f, "
                   "Processed: %zu, Published: %zu",
                   avg_interval, min_interval, max_interval,
                   processed_count_, published_count_);
        
        // 检测卡顿
        if (max_interval > avg_interval * 2.0 && max_interval > 10.0) {
            RCLCPP_WARN(get_logger(), "Detected stuttering: max interval %.3fms is much larger than avg %.3fms", 
                       max_interval, avg_interval);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastPointCloudRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}