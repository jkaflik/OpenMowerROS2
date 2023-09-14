#pragma once

#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

class ClickHouseStorage : public rosbag2_storage::storage_interfaces::ReadWriteInterface {
public:
    ClickHouseStorage() = default;

    ~ClickHouseStorage() override;

    void open(
            const rosbag2_storage::StorageOptions & storage_options,
            rosbag2_storage::storage_interfaces::IOFlag io_flag =
            rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;

    void update_metadata(const rosbag2_storage::BagMetadata & metadata) override;

    void remove_topic(const rosbag2_storage::TopicMetadata & topic) override;

    void create_topic(
            const rosbag2_storage::TopicMetadata & topic,
            const rosbag2_storage::MessageDefinition & message_definition) override;

    void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message) override;

    void write(
            const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & messages)
    override;

    bool set_read_order(const rosbag2_storage::ReadOrder &) override;

    bool has_next() override;

    std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;

    std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() override;

    void get_all_message_definitions(
            std::vector<rosbag2_storage::MessageDefinition> & definitons) override;

    rosbag2_storage::BagMetadata get_metadata() override;

    std::string get_relative_file_path() const override;

    uint64_t get_bagfile_size() const override;

    std::string get_storage_identifier() const override;

    uint64_t get_minimum_split_file_size() const override;

    void set_filter(const rosbag2_storage::StorageFilter & storage_filter) override;

    void reset_filter() override;

    void seek(const rcutils_time_point_value_t & timestamp) override;
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ClickHouseStorage,
        rosbag2_storage::storage_interfaces::ReadWriteInterface);
